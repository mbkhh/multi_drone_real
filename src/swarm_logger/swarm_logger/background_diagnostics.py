"""Non-blocking subprocess helpers for local system diagnostics."""

from __future__ import annotations

import shutil
import subprocess
import time
from collections import Counter, deque
from dataclasses import dataclass
from threading import Event, Lock, Thread


@dataclass(frozen=True)
class ProbeResult:
    """Result of one optional local command probe."""

    status: str
    duration_s: float
    stdout: str = ''
    stderr: str = ''
    returncode: int | None = None


def run_probe(command, timeout=0.4) -> ProbeResult:
    """Run an optional command and preserve why it did not produce data."""
    started = time.monotonic()
    if not command or shutil.which(command[0]) is None:
        return ProbeResult('missing', time.monotonic() - started)
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        return ProbeResult(
            'timeout',
            time.monotonic() - started,
            stdout=str(error.stdout or ''),
            stderr=str(error.stderr or ''),
        )
    except OSError as error:
        return ProbeResult(
            'error', time.monotonic() - started, stderr=str(error)
        )
    stdout = completed.stdout.strip()
    stderr = completed.stderr.strip()
    if completed.returncode != 0:
        return ProbeResult(
            'nonzero_exit',
            time.monotonic() - started,
            stdout=stdout,
            stderr=stderr,
            returncode=completed.returncode,
        )
    if not stdout:
        return ProbeResult(
            'empty',
            time.monotonic() - started,
            stderr=stderr,
            returncode=completed.returncode,
        )
    return ProbeResult(
        'ok',
        time.monotonic() - started,
        stdout=stdout,
        stderr=stderr,
        returncode=completed.returncode,
    )


def classify_iw_event(line: str) -> str:
    """Classify a line emitted by ``iw event -t -f``."""
    text = (line or '').lower()
    if 'disconnected' in text or 'disconnect' in text:
        return 'disconnect'
    if 'deauth' in text:
        return 'deauth'
    if 'disassoc' in text:
        return 'disassoc'
    if 'roam' in text:
        return 'roam'
    if 'connected to' in text or 'connect ' in text:
        return 'connect'
    if 'cqm' in text:
        return 'cqm'
    if 'scan' in text:
        return 'scan'
    return 'other'


class WifiEventMonitor:
    """Collect a bounded stream of kernel nl80211 events in the background."""

    def __init__(self, interface: str, max_events: int = 50):
        self.interface = interface
        self._lock = Lock()
        self._stop = Event()
        self._events = deque(maxlen=max(1, int(max_events)))
        self._counts = Counter()
        self._process = None
        self._status = 'not_started'
        self._thread = Thread(
            target=self._run,
            name='swarm-logger-iw-event',
            daemon=True,
        )

    def start(self):
        self._thread.start()

    def _run(self):
        if shutil.which('iw') is None:
            with self._lock:
                self._status = 'missing'
            return
        try:
            process = subprocess.Popen(
                ['iw', 'event', '-t', '-f'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError:
            with self._lock:
                self._status = 'start_error'
            return
        self._process = process
        with self._lock:
            self._status = 'running'
        if process.stdout is not None:
            for raw_line in process.stdout:
                if self._stop.is_set():
                    break
                line = raw_line.strip()
                if not line:
                    continue
                if self.interface and self.interface not in line:
                    continue
                category = classify_iw_event(line)
                with self._lock:
                    self._counts[category] += 1
                    self._events.append((time.monotonic(), category, line[:300]))
        returncode = process.poll()
        with self._lock:
            if self._stop.is_set():
                self._status = 'stopped'
            else:
                self._status = f'exited:{returncode}'

    def snapshot(self, clear=True):
        """Return event counters and recent lines without unbounded storage."""
        with self._lock:
            result = {
                'status': self._status,
                'counts': dict(self._counts),
                'events': tuple(self._events),
            }
            if clear:
                self._counts.clear()
                self._events.clear()
        return result

    def stop(self):
        """Terminate only the child ``iw event`` process owned by this object."""
        self._stop.set()
        process = self._process
        if process is not None and process.poll() is None:
            try:
                process.terminate()
                process.wait(timeout=0.5)
            except (OSError, subprocess.SubprocessError):
                try:
                    process.kill()
                except OSError:
                    pass
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
