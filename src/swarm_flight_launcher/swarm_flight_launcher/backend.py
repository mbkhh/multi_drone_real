"""Command construction and process backends for the flight launcher GUI."""

from __future__ import annotations

import base64
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import queue
import shlex
import signal
import subprocess
import threading
import time
from typing import Callable, Dict, Optional

import paramiko


PID_MARKER = '__SWARM_LAUNCH_PID__:'
DEFAULT_ROS_SETUP = '/opt/ros/rolling/setup.sh'
DEFAULT_REMOTE_WORKSPACE = '~/multi_drone_real'
COMPONENT_ORDER = ('agent', 'control', 'vision')


@dataclass(frozen=True)
class ComponentSpec:
    """One process that is run on a companion computer."""

    name: str
    command: str
    duplicate_probe: str


@dataclass
class RemoteProcess:
    """Runtime information retained for one remote process."""

    component: str
    channel: paramiko.Channel
    pid: Optional[int] = None
    state: str = 'STARTING'


def build_component_specs(drone_id: int) -> Dict[str, ComponentSpec]:
    """Return the exact real-flight commands for one drone ID."""
    drone_id = int(drone_id)
    if drone_id <= 0:
        raise ValueError('drone_id must be positive')

    return {
        'agent': ComponentSpec(
            name='Micro XRCE Agent',
            command=(
                'MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600'
            ),
            duplicate_probe=(
                "pgrep -af '[M]icroXRCEAgent.*serial.*ttyAMA0.*921600'"
            ),
        ),
        'control': ComponentSpec(
            name='No-TF yaw controller',
            command=(
                'ros2 run swarm_single_no_tf_yaw control_node --ros-args '
                f'-p frame_id:={drone_id} '
                f'-r __node:=control_node_no_tf_yaw_d{drone_id}'
            ),
            duplicate_probe=(
                "pgrep -af '[s]warm_single_no_tf_yaw.*control_node'"
            ),
        ),
        'vision': ComponentSpec(
            name='Flight vision',
            command=(
                './run_swarm_vision.sh flight --ros-args '
                f'-p uav_id:=UAV_{drone_id} '
                '-p camera_index:=0 -p input_size:=320 '
                '-p inference_threads:=1'
            ),
            duplicate_probe=(
                "pgrep -af '[s]warm_vision.flight_vision_node'"
            ),
        ),
    }


def build_remote_script(
    workspace: str,
    command: str,
    ros_setup: str = DEFAULT_ROS_SETUP,
    source_ros_environment: bool = True,
) -> str:
    """Build an isolated Bash script for one remote component."""
    workspace = workspace.strip()
    if not workspace:
        raise ValueError('remote workspace cannot be empty')
    if not command.strip():
        raise ValueError('remote command cannot be empty')

    # Keep a leading ~/ usable on the remote host while still quoting the rest.
    if workspace == '~':
        workspace_expression = '"$HOME"'
    elif workspace.startswith('~/'):
        workspace_expression = '"$HOME"/' + shlex.quote(workspace[2:])
    else:
        workspace_expression = shlex.quote(workspace)

    script_lines = [
        'set -e',
        f'cd -- {workspace_expression}',
        'export PYTHONUNBUFFERED=1',
        'echo "[launcher] remote host=$(hostname) workspace=$(pwd)"',
    ]
    if source_ros_environment:
        script_lines.extend((
            'unset ROS_DOMAIN_ID',
            'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            f'source {shlex.quote(ros_setup)}',
            'source install/setup.sh',
            f'echo "[launcher] sourced {shlex.quote(ros_setup)}"',
            'echo "[launcher] sourced install/setup.sh"',
        ))
    else:
        # MicroXRCEAgent is installed in /usr/local with its own Fast DDS.
        # ROS Rolling's LD_LIBRARY_PATH substitutes a different libfastdds
        # while retaining the /usr/local Micro XRCE libraries, which can abort
        # with "stack smashing detected" once PX4 establishes a session.
        script_lines.extend((
            'unset ROS_DOMAIN_ID RMW_IMPLEMENTATION',
            'unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH',
            'unset PYTHONPATH LD_LIBRARY_PATH',
            'echo "[launcher] standalone environment; ROS setup skipped"',
        ))
    script_lines.extend((
        f'echo {shlex.quote(PID_MARKER)}$$',
        f'exec {command}',
    ))
    script = '\n'.join(script_lines)
    return 'setsid --wait bash --noprofile --norc -lc ' + shlex.quote(script)


def build_station_script(
    workspace: str,
    ros_setup: str = DEFAULT_ROS_SETUP,
) -> str:
    """Build the local interactive leader/follower station command."""
    workspace_path = str(Path(workspace).expanduser())
    return '\n'.join((
        'set -e',
        f'cd -- {shlex.quote(workspace_path)}',
        'unset ROS_DOMAIN_ID',
        'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
        'export PYTHONUNBUFFERED=1',
        f'source {shlex.quote(ros_setup)}',
        'source install/setup.bash',
        'exec ros2 run swarm_station station',
    ))


def safe_settings_payload(settings):
    """Return persistable settings with all password-like keys removed."""
    if isinstance(settings, dict):
        return {
            key: safe_settings_payload(value)
            for key, value in settings.items()
            if 'password' not in str(key).lower()
        }
    if isinstance(settings, list):
        return [safe_settings_payload(value) for value in settings]
    return settings


def save_settings(path: Path, settings: dict) -> None:
    """Save non-secret launcher settings with user-only permissions."""
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(safe_settings_payload(settings), indent=2) + '\n',
        encoding='utf-8',
    )
    os.chmod(temporary, 0o600)
    temporary.replace(path)


def load_settings(path: Path) -> dict:
    """Load launcher settings, returning an empty mapping when absent."""
    path = Path(path).expanduser()
    if not path.exists():
        return {}
    value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError('launcher settings must contain a JSON object')
    return safe_settings_payload(value)


class LocalStationProcess:
    """Manage the interactive station subprocess on the laptop."""

    def __init__(self, emit: Callable[[str, str, object], None]):
        self.emit = emit
        self.process: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()

    def is_running(self) -> bool:
        with self._lock:
            return self.process is not None and self.process.poll() is None

    def start(self, workspace: str, ros_setup: str) -> None:
        with self._lock:
            if self.process is not None and self.process.poll() is None:
                self.emit('output', 'station', 'Station is already running.\n')
                return
            script = build_station_script(workspace, ros_setup)
            self.process = subprocess.Popen(
                ['bash', '--noprofile', '--norc', '-lc', script],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=0,
                start_new_session=True,
            )
            process = self.process
        self.emit('status', 'station', 'RUNNING')
        self.emit('output', 'station', f'[launcher] station pid={process.pid}\n')
        threading.Thread(
            target=self._read_output,
            args=(process,),
            name='station-output',
            daemon=True,
        ).start()

    def _read_output(self, process: subprocess.Popen) -> None:
        assert process.stdout is not None
        while True:
            chunk = os.read(process.stdout.fileno(), 4096)
            if not chunk:
                break
            self.emit(
                'output',
                'station',
                chunk.decode('utf-8', errors='replace'),
            )
        exit_code = process.wait()
        self.emit('output', 'station', f'\n[launcher] station exited {exit_code}\n')
        self.emit('status', 'station', f'EXITED ({exit_code})')

    def send(self, command: str) -> None:
        with self._lock:
            process = self.process
            if process is None or process.poll() is not None or process.stdin is None:
                raise RuntimeError('station is not running')
            process.stdin.write((command.rstrip('\n') + '\n').encode('utf-8'))
            process.stdin.flush()
        self.emit('output', 'station', f'> {command.rstrip()}\n')

    def stop(self, timeout: float = 5.0) -> None:
        with self._lock:
            process = self.process
        if process is None or process.poll() is not None:
            return
        self.emit('status', 'station', 'STOPPING')
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


class RemoteDroneSession:
    """Own one SSH connection and its three managed remote processes."""

    def __init__(
        self,
        drone_id: int,
        emit: Callable[[str, str, object], None],
    ):
        self.drone_id = int(drone_id)
        self.emit = emit
        self.client: Optional[paramiko.SSHClient] = None
        self.processes: Dict[str, RemoteProcess] = {}
        self._lock = threading.RLock()

    @property
    def prefix(self) -> str:
        return f'd{self.drone_id}'

    def key(self, component: str) -> str:
        return f'{self.prefix}/{component}'

    def connect(
        self,
        host: str,
        username: str,
        password: str,
        port: int,
        trust_new_host: bool,
    ) -> str:
        """Connect or reuse a live SSH transport and return its fingerprint."""
        host = host.strip()
        username = username.strip()
        if not host or not username:
            raise ValueError('host and username are required')

        with self._lock:
            if (
                self.client is not None
                and self.client.get_transport() is not None
                and self.client.get_transport().is_active()
            ):
                fingerprint = self._fingerprint()
                self.emit(
                    'connection', self.prefix, f'CONNECTED {fingerprint}'
                )
                return fingerprint

            client = paramiko.SSHClient()
            client.load_system_host_keys()
            known_hosts = Path.home() / '.ssh' / 'known_hosts'
            if trust_new_host and not known_hosts.exists():
                known_hosts.parent.mkdir(parents=True, exist_ok=True)
                os.chmod(known_hosts.parent, 0o700)
                known_hosts.touch(mode=0o600)
                os.chmod(known_hosts, 0o600)
            if known_hosts.exists():
                client.load_host_keys(str(known_hosts))
            client.set_missing_host_key_policy(
                paramiko.AutoAddPolicy()
                if trust_new_host
                else paramiko.RejectPolicy()
            )
            client.connect(
                hostname=host,
                port=int(port),
                username=username,
                password=password or None,
                timeout=8.0,
                banner_timeout=8.0,
                auth_timeout=8.0,
                allow_agent=True,
                look_for_keys=True,
            )
            self.client = client
            fingerprint = self._fingerprint()

        self.emit('connection', self.prefix, f'CONNECTED {fingerprint}')
        return fingerprint

    def _fingerprint(self) -> str:
        assert self.client is not None
        transport = self.client.get_transport()
        if transport is None:
            return 'unknown-key'
        digest = hashlib.sha256(
            transport.get_remote_server_key().asbytes()
        ).digest()
        return 'SHA256:' + base64.b64encode(digest).decode().rstrip('=')

    def _run_short(self, command: str, timeout: float = 5.0) -> tuple[int, str]:
        with self._lock:
            if self.client is None:
                raise RuntimeError('SSH is not connected')
            _, stdout, stderr = self.client.exec_command(
                command,
                timeout=timeout,
            )
        exit_code = stdout.channel.recv_exit_status()
        output = stdout.read().decode(errors='replace')
        output += stderr.read().decode(errors='replace')
        return exit_code, output.strip()

    def start_component(
        self,
        component: str,
        workspace: str,
        ros_setup: str,
    ) -> None:
        """Refuse duplicates, then start and stream one remote component."""
        specs = build_component_specs(self.drone_id)
        if component not in specs:
            raise ValueError(f'unknown component {component!r}')
        spec = specs[component]
        key = self.key(component)

        with self._lock:
            existing = self.processes.get(component)
            if existing is not None and not existing.channel.exit_status_ready():
                self.emit('output', key, '[launcher] already managed and running\n')
                return

        _, duplicate = self._run_short(spec.duplicate_probe + ' || true')
        if duplicate:
            self.emit('status', key, 'DUPLICATE FOUND')
            self.emit(
                'output',
                key,
                '[launcher] REFUSED: a matching process already exists:\n'
                + duplicate + '\n',
            )
            return

        remote_command = build_remote_script(
            workspace,
            spec.command,
            ros_setup,
            source_ros_environment=(component != 'agent'),
        )
        with self._lock:
            if self.client is None:
                raise RuntimeError('SSH is not connected')
            transport = self.client.get_transport()
            if transport is None or not transport.is_active():
                raise RuntimeError('SSH connection is not active')
            channel = transport.open_session(timeout=8.0)
            channel.get_pty(term='xterm', width=160, height=48)
            channel.exec_command(remote_command)
            process = RemoteProcess(component=component, channel=channel)
            self.processes[component] = process

        self.emit('status', key, 'STARTING')
        self.emit('output', key, f'[launcher] starting {spec.name}\n')
        self.emit(
            'output',
            key,
            f'[launcher] command: {spec.command}\n',
        )
        threading.Thread(
            target=self._read_remote_output,
            args=(process,),
            name=f'{key}-output',
            daemon=True,
        ).start()

    def _read_remote_output(self, process: RemoteProcess) -> None:
        key = self.key(process.component)
        pending = ''
        channel = process.channel
        while not channel.exit_status_ready() or channel.recv_ready():
            if channel.recv_ready():
                text = channel.recv(4096).decode('utf-8', errors='replace')
                pending += text.replace('\r\n', '\n').replace('\r', '\n')
                lines = pending.split('\n')
                pending = lines.pop()
                for line in lines:
                    if line.startswith(PID_MARKER):
                        try:
                            process.pid = int(line[len(PID_MARKER):].strip())
                            process.state = 'RUNNING'
                            self.emit('status', key, f'RUNNING pid={process.pid}')
                        except ValueError:
                            self.emit('output', key, line + '\n')
                    else:
                        self.emit('output', key, line + '\n')
            else:
                time.sleep(0.05)
        if pending:
            self.emit('output', key, pending + '\n')
        exit_code = channel.recv_exit_status()
        process.state = f'EXITED ({exit_code})'
        self.emit('output', key, f'[launcher] remote process exited {exit_code}\n')
        self.emit('status', key, process.state)

    def stop_component(self, component: str, timeout: float = 5.0) -> None:
        """Interrupt the exact process group launched by this GUI."""
        key = self.key(component)
        with self._lock:
            process = self.processes.get(component)
        if process is None or process.channel.exit_status_ready():
            return
        self.emit('status', key, 'STOPPING')
        if process.pid is None:
            process.channel.send('\x03')
        else:
            pid = int(process.pid)
            self._run_short(
                f'kill -INT -- -{pid} 2>/dev/null || kill -INT {pid} 2>/dev/null || true'
            )
        deadline = time.monotonic() + timeout
        while not process.channel.exit_status_ready() and time.monotonic() < deadline:
            time.sleep(0.1)
        if not process.channel.exit_status_ready() and process.pid is not None:
            pid = int(process.pid)
            self._run_short(
                f'kill -TERM -- -{pid} 2>/dev/null || kill -TERM {pid} 2>/dev/null || true'
            )

    def stop_all(self) -> None:
        for component in reversed(COMPONENT_ORDER):
            try:
                self.stop_component(component)
            except Exception as error:
                self.emit(
                    'output',
                    self.key(component),
                    f'[launcher] stop failed: {error}\n',
                )

    def close(self) -> None:
        with self._lock:
            client = self.client
            self.client = None
        if client is not None:
            client.close()


class EventBridge:
    """Small thread-safe event queue consumed by Tk's main thread."""

    def __init__(self):
        self.queue: queue.Queue = queue.Queue()

    def emit(self, kind: str, key: str, value: object) -> None:
        self.queue.put((kind, key, value))
