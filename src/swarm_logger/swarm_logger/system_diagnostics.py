"""Low-overhead parsers and readers for companion-computer diagnostics."""

from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple


_INTERFACE_NAME = re.compile(r'^[A-Za-z0-9_.:-]+$')
_NET_COUNTERS = (
    'rx_bytes',
    'tx_bytes',
    'rx_packets',
    'tx_packets',
    'rx_errors',
    'tx_errors',
    'rx_dropped',
    'tx_dropped',
)
_THROTTLE_FLAGS = (
    (0x1, 'under-voltage now'),
    (0x2, 'CPU frequency capped now'),
    (0x4, 'throttling active now'),
    (0x8, 'soft temperature limit active now'),
    (0x10000, 'under-voltage occurred since boot'),
    (0x20000, 'CPU frequency capping occurred since boot'),
    (0x40000, 'throttling occurred since boot'),
    (0x80000, 'soft temperature limit occurred since boot'),
)


def parse_throttled_output(output: str) -> Optional[int]:
    """Parse ``vcgencmd get_throttled`` output, or return None."""
    match = re.search(r'throttled=(0x[0-9a-fA-F]+|[0-9]+)', output)
    if match is None:
        return None
    try:
        return int(match.group(1), 0)
    except ValueError:
        return None


def decode_throttled_state(value: Optional[int]) -> Tuple[str, ...]:
    """Return human-readable Raspberry Pi throttle flags."""
    if value is None:
        return ()
    return tuple(label for mask, label in _THROTTLE_FLAGS if value & mask)


def parse_iw_link(output: str) -> Dict[str, object]:
    """Parse the stable fields emitted by ``iw dev IFACE link``."""
    stats: Dict[str, object] = {
        'connected': False,
        'ssid': None,
        'bssid': None,
        'frequency_mhz': None,
        'signal_dbm': None,
        'tx_bitrate_mbps': None,
        'rx_bitrate_mbps': None,
    }
    if not output or 'Not connected.' in output:
        return stats

    bssid = re.search(r'Connected to\s+([0-9a-fA-F:]{17})', output)
    ssid = re.search(r'^\s*SSID:\s*(.+?)\s*$', output, re.MULTILINE)
    frequency = re.search(r'^\s*freq:\s*(\d+)', output, re.MULTILINE)
    signal = re.search(
        r'^\s*signal:\s*(-?\d+(?:\.\d+)?)\s*dBm',
        output,
        re.MULTILINE,
    )
    tx_rate = re.search(
        r'^\s*tx bitrate:\s*([0-9.]+)\s*MBit/s',
        output,
        re.MULTILINE,
    )
    rx_rate = re.search(
        r'^\s*rx bitrate:\s*([0-9.]+)\s*MBit/s',
        output,
        re.MULTILINE,
    )

    stats['connected'] = bssid is not None
    stats['bssid'] = bssid.group(1).lower() if bssid else None
    stats['ssid'] = ssid.group(1) if ssid else None
    stats['frequency_mhz'] = int(frequency.group(1)) if frequency else None
    stats['signal_dbm'] = float(signal.group(1)) if signal else None
    stats['tx_bitrate_mbps'] = float(tx_rate.group(1)) if tx_rate else None
    stats['rx_bitrate_mbps'] = float(rx_rate.group(1)) if rx_rate else None
    return stats


def parse_iw_station_dump(output: str) -> Dict[str, int]:
    """Parse optional driver retry/failure counters from ``iw station dump``."""
    field_patterns = {
        'tx_retries': r'^\s*tx retries:\s*(\d+)',
        'tx_failed': r'^\s*tx failed:\s*(\d+)',
        'rx_drop_misc': r'^\s*rx drop misc:\s*(\d+)',
        'beacon_loss': r'^\s*beacon loss:\s*(\d+)',
    }
    parsed = {}
    for name, pattern in field_patterns.items():
        match = re.search(pattern, output or '', re.MULTILINE)
        if match:
            parsed[name] = int(match.group(1))
    return parsed


def signal_label(signal_dbm: Optional[float]) -> str:
    """Return a coarse label; packet-loss measurements remain authoritative."""
    if signal_dbm is None:
        return 'unknown'
    if signal_dbm >= -55.0:
        return 'strong'
    if signal_dbm >= -67.0:
        return 'good'
    if signal_dbm >= -75.0:
        return 'fair'
    return 'weak'


def read_cpu_times(path: str = '/proc/stat') -> Optional[Tuple[int, int]]:
    """Return Linux aggregate CPU idle and total tick counts."""
    try:
        first_line = Path(path).read_text(encoding='ascii').splitlines()[0]
        fields = first_line.split()
        if not fields or fields[0] != 'cpu':
            return None
        values = [int(value) for value in fields[1:]]
        if len(values) < 4:
            return None
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return idle, sum(values)
    except (OSError, ValueError, IndexError):
        return None


def cpu_usage_percent(
    previous: Optional[Tuple[int, int]],
    current: Optional[Tuple[int, int]],
) -> Optional[float]:
    """Calculate aggregate CPU use between two tick samples."""
    if previous is None or current is None:
        return None
    idle_delta = current[0] - previous[0]
    total_delta = current[1] - previous[1]
    if total_delta <= 0 or idle_delta < 0:
        return None
    busy_delta = max(0, total_delta - idle_delta)
    return 100.0 * busy_delta / total_delta


def read_memory(path: str = '/proc/meminfo') -> Optional[Dict[str, float]]:
    """Read total/available memory without starting a subprocess."""
    values = {}
    try:
        for line in Path(path).read_text(encoding='ascii').splitlines():
            key, separator, remainder = line.partition(':')
            if not separator:
                continue
            token = remainder.strip().split()[0]
            values[key] = int(token) * 1024
    except (OSError, ValueError, IndexError):
        return None

    total = values.get('MemTotal')
    available = values.get('MemAvailable')
    if not total or available is None:
        return None
    return {
        'total_bytes': float(total),
        'available_bytes': float(available),
        'used_percent': 100.0 * (total - available) / total,
    }


def read_temperature(
    path: str = '/sys/class/thermal/thermal_zone0/temp',
) -> Optional[float]:
    """Read the primary thermal-zone temperature in degrees Celsius."""
    try:
        return float(Path(path).read_text(encoding='ascii').strip()) / 1000.0
    except (OSError, ValueError):
        return None


def read_process_rss(path: str = '/proc/self/status') -> Optional[int]:
    """Read current resident memory for the logger process."""
    try:
        for line in Path(path).read_text(encoding='ascii').splitlines():
            if line.startswith('VmRSS:'):
                return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        return None
    return None


def read_net_counters(
    interface: str,
    sys_class_net: str = '/sys/class/net',
) -> Optional[Dict[str, int]]:
    """Read interface counters while rejecting path-like parameter values."""
    if not _INTERFACE_NAME.fullmatch(interface):
        return None
    statistics = Path(sys_class_net) / interface / 'statistics'
    counters = {}
    try:
        for name in _NET_COUNTERS:
            counters[name] = int(
                (statistics / name).read_text(encoding='ascii').strip()
            )
    except (OSError, ValueError):
        return None
    return counters


def counter_deltas(
    previous: Optional[Dict[str, int]],
    current: Optional[Dict[str, int]],
    names: Optional[Iterable[str]] = None,
) -> Dict[str, int]:
    """Calculate non-negative deltas, tolerating resets and missing fields."""
    if previous is None or current is None:
        return {}
    result = {}
    for name in names or current.keys():
        if name not in previous or name not in current:
            continue
        result[name] = max(0, current[name] - previous[name])
    return result


def read_interface_state(
    interface: str,
    sys_class_net: str = '/sys/class/net',
) -> str:
    """Read the kernel interface operational state."""
    if not _INTERFACE_NAME.fullmatch(interface):
        return 'invalid-interface'
    try:
        return (
            Path(sys_class_net, interface, 'operstate')
            .read_text(encoding='ascii')
            .strip()
        )
    except OSError:
        return 'unavailable'


def read_load_average() -> Optional[Tuple[float, float, float]]:
    """Wrap ``getloadavg`` for platforms where it may be unavailable."""
    try:
        return os.getloadavg()
    except OSError:
        return None
