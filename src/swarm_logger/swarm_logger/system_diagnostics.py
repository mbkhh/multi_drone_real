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


def parse_iw_station_dump(output: str) -> Dict[str, object]:
    """Parse counters and link gauges from ``iw station dump``."""
    field_patterns = {
        'inactive_time_ms': (r'^\s*inactive time:\s*(\d+)\s*ms', int),
        'rx_bytes': (r'^\s*rx bytes:\s*(\d+)', int),
        'rx_packets': (r'^\s*rx packets:\s*(\d+)', int),
        'tx_bytes': (r'^\s*tx bytes:\s*(\d+)', int),
        'tx_packets': (r'^\s*tx packets:\s*(\d+)', int),
        'tx_retries': (r'^\s*tx retries:\s*(\d+)', int),
        'tx_failed': (r'^\s*tx failed:\s*(\d+)', int),
        'rx_drop_misc': (r'^\s*rx drop misc:\s*(\d+)', int),
        'beacon_loss': (r'^\s*beacon loss:\s*(\d+)', int),
        'beacon_rx': (r'^\s*beacon rx:\s*(\d+)', int),
        'connected_time_s': (r'^\s*connected time:\s*(\d+)\s*seconds', int),
        'signal_avg_dbm': (
            r'^\s*signal avg:\s*(-?\d+(?:\.\d+)?)\s*dBm',
            float,
        ),
        'expected_throughput_mbps': (
            r'^\s*expected throughput:\s*([0-9.]+)\s*Mbps',
            float,
        ),
        'tx_duration_us': (r'^\s*tx duration:\s*(\d+)\s*us', int),
        'rx_duration_us': (r'^\s*rx duration:\s*(\d+)\s*us', int),
    }
    parsed = {}
    for name, (pattern, converter) in field_patterns.items():
        match = re.search(pattern, output or '', re.MULTILINE)
        if match:
            parsed[name] = converter(match.group(1))
    return parsed


def parse_iw_survey_dump(output: str) -> Dict[str, object]:
    """Parse the active-channel block from ``iw ... survey dump``."""
    blocks = re.split(r'(?=^\s*frequency:)', output or '', flags=re.MULTILINE)
    selected = None
    for block in blocks:
        if '[in use]' in block:
            selected = block
            break
    if selected is None:
        return {}

    result: Dict[str, object] = {}
    patterns = {
        'frequency_mhz': r'^\s*frequency:\s*(\d+)\s*MHz',
        'noise_dbm': r'^\s*noise:\s*(-?\d+)\s*dBm',
        'active_time_ms': r'^\s*channel active time:\s*(\d+)\s*ms',
        'busy_time_ms': r'^\s*channel busy time:\s*(\d+)\s*ms',
        'receive_time_ms': r'^\s*channel receive time:\s*(\d+)\s*ms',
        'transmit_time_ms': r'^\s*channel transmit time:\s*(\d+)\s*ms',
        'bss_receive_time_ms': (
            r'^\s*channel BSS receive time:\s*(\d+)\s*ms'
        ),
    }
    for name, pattern in patterns.items():
        match = re.search(pattern, selected, re.MULTILINE)
        if match:
            result[name] = int(match.group(1))
    return result


def wireless_ratios(delta: Dict[str, int]) -> Dict[str, float]:
    """Calculate accurately named Wi-Fi retry/failure ratios."""
    packets = int(delta.get('tx_packets', 0))
    if packets <= 0:
        return {}
    return {
        'retries_per_tx_packet': delta.get('tx_retries', 0) / packets,
        'failed_per_1000_tx_packets': (
            1000.0 * delta.get('tx_failed', 0) / packets
        ),
    }


def survey_percentages(delta: Dict[str, int]) -> Dict[str, float]:
    """Calculate active-channel airtime percentages from survey deltas."""
    active = int(delta.get('active_time_ms', 0))
    if active <= 0:
        return {}
    result = {}
    for source, destination in (
        ('busy_time_ms', 'busy_percent'),
        ('receive_time_ms', 'receive_percent'),
        ('transmit_time_ms', 'transmit_percent'),
        ('bss_receive_time_ms', 'bss_receive_percent'),
    ):
        if source in delta:
            result[destination] = 100.0 * delta[source] / active
    return result


def parse_proc_net_snmp(output: str) -> Dict[str, int]:
    """Parse paired protocol header/value lines from ``/proc/net/snmp``."""
    lines = [line.strip() for line in (output or '').splitlines() if line.strip()]
    parsed = {}
    index = 0
    while index + 1 < len(lines):
        header = lines[index].split()
        values = lines[index + 1].split()
        index += 2
        if not header or not values or header[0] != values[0]:
            continue
        protocol = header[0].rstrip(':')
        for name, value in zip(header[1:], values[1:]):
            try:
                parsed[f'{protocol}.{name}'] = int(value)
            except ValueError:
                continue
    return parsed


def read_proc_net_snmp(path: str = '/proc/net/snmp') -> Dict[str, int]:
    """Read kernel IP/UDP error and datagram counters."""
    try:
        return parse_proc_net_snmp(Path(path).read_text(encoding='ascii'))
    except OSError:
        return {}


def parse_softnet_stat(output: str) -> Dict[str, int]:
    """Sum Linux per-CPU softnet processed/drop/time-squeeze counters."""
    result = {'processed': 0, 'dropped': 0, 'time_squeeze': 0}
    valid = False
    for line in (output or '').splitlines():
        fields = line.split()
        if len(fields) < 3:
            continue
        try:
            result['processed'] += int(fields[0], 16)
            result['dropped'] += int(fields[1], 16)
            result['time_squeeze'] += int(fields[2], 16)
            valid = True
        except ValueError:
            continue
    return result if valid else {}


def read_softnet_stat(path: str = '/proc/net/softnet_stat') -> Dict[str, int]:
    """Read aggregate kernel softnet counters."""
    try:
        return parse_softnet_stat(Path(path).read_text(encoding='ascii'))
    except OSError:
        return {}


def parse_pressure(output: str) -> Dict[str, float]:
    """Parse Linux PSI ``some`` and ``full`` averages/totals."""
    result = {}
    for line in (output or '').splitlines():
        fields = line.split()
        if not fields:
            continue
        category = fields[0]
        for token in fields[1:]:
            name, separator, value = token.partition('=')
            if not separator:
                continue
            try:
                result[f'{category}_{name}'] = float(value)
            except ValueError:
                continue
    return result


def read_pressure(resource: str, root: str = '/proc/pressure') -> Dict[str, float]:
    """Read a Linux CPU, memory, or IO pressure file."""
    if resource not in ('cpu', 'memory', 'io'):
        return {}
    try:
        return parse_pressure(
            Path(root, resource).read_text(encoding='ascii')
        )
    except OSError:
        return {}


def read_cpu_times_per_core(path: str = '/proc/stat') -> Dict[str, Tuple[int, int]]:
    """Return idle/total ticks for the aggregate CPU and each core."""
    result = {}
    try:
        lines = Path(path).read_text(encoding='ascii').splitlines()
    except OSError:
        return result
    for line in lines:
        fields = line.split()
        if not fields or not re.fullmatch(r'cpu\d*', fields[0]):
            continue
        try:
            values = [int(value) for value in fields[1:]]
        except ValueError:
            continue
        if len(values) < 4:
            continue
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        result[fields[0]] = (idle, sum(values))
    return result


def cpu_usage_per_core(
    previous: Dict[str, Tuple[int, int]],
    current: Dict[str, Tuple[int, int]],
) -> Dict[str, float]:
    """Calculate CPU percentages for matching aggregate/core snapshots."""
    result = {}
    for name, current_times in current.items():
        value = cpu_usage_percent(previous.get(name), current_times)
        if value is not None:
            result[name] = value
    return result


def parse_process_stat(output: str) -> Dict[str, object]:
    """Parse selected fields from Linux ``/proc/PID/stat`` safely."""
    text = (output or '').strip()
    left = text.find('(')
    right = text.rfind(')')
    if left <= 0 or right <= left:
        return {}
    try:
        pid = int(text[:left].strip())
        tail = text[right + 1:].strip().split()
        return {
            'pid': pid,
            'comm': text[left + 1:right],
            'state': tail[0],
            'ppid': int(tail[1]),
            'cpu_ticks': int(tail[11]) + int(tail[12]),
            'num_threads': int(tail[17]),
            'start_ticks': int(tail[19]),
            'rss_pages': int(tail[21]),
            'processor': int(tail[36]) if len(tail) > 36 else None,
        }
    except (IndexError, ValueError):
        return {}


def read_process_metrics(pid: int, proc_root: str = '/proc') -> Dict[str, object]:
    """Read bounded process CPU/state/RSS/scheduler diagnostics."""
    root = Path(proc_root, str(int(pid)))
    try:
        result = parse_process_stat(
            (root / 'stat').read_text(encoding='ascii')
        )
    except (OSError, ValueError):
        return {}
    if not result:
        return {}

    try:
        result['cmdline'] = (
            (root / 'cmdline')
            .read_bytes()
            .replace(b'\x00', b' ')
            .decode('utf-8', errors='replace')
            .strip()
        )
    except OSError:
        result['cmdline'] = ''
    try:
        result['wchan'] = (root / 'wchan').read_text(encoding='ascii').strip()
    except OSError:
        result['wchan'] = 'unavailable'

    try:
        schedstat = (root / 'schedstat').read_text(encoding='ascii').split()
        result['runtime_ns'] = int(schedstat[0])
        result['runqueue_ns'] = int(schedstat[1])
        result['timeslices'] = int(schedstat[2])
    except (OSError, ValueError, IndexError):
        result['runtime_ns'] = None
        result['runqueue_ns'] = None
        result['timeslices'] = None

    try:
        page_size = os.sysconf('SC_PAGE_SIZE')
    except (ValueError, OSError):
        page_size = 4096
    result['rss_bytes'] = max(0, result['rss_pages']) * page_size

    try:
        for line in (root / 'status').read_text(encoding='ascii').splitlines():
            if line.startswith('voluntary_ctxt_switches:'):
                result['voluntary_context_switches'] = int(line.split()[1])
            elif line.startswith('nonvoluntary_ctxt_switches:'):
                result['nonvoluntary_context_switches'] = int(line.split()[1])
    except (OSError, ValueError, IndexError):
        pass
    return result


def find_processes(
    required_tokens: Iterable[str],
    proc_root: str = '/proc',
) -> Tuple[Dict[str, object], ...]:
    """Find processes whose command lines contain every requested token."""
    tokens = tuple(str(token) for token in required_tokens if str(token))
    matches = []
    try:
        entries = Path(proc_root).iterdir()
    except OSError:
        return ()
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            cmdline = (
                (entry / 'cmdline')
                .read_bytes()
                .replace(b'\x00', b' ')
                .decode('utf-8', errors='replace')
                .strip()
            )
        except OSError:
            continue
        if tokens and not all(token in cmdline for token in tokens):
            continue
        metrics = read_process_metrics(int(entry.name), proc_root)
        if metrics:
            matches.append(metrics)
    return tuple(sorted(matches, key=lambda item: int(item['pid'])))


def process_deltas(
    previous: Optional[Dict[str, object]],
    current: Dict[str, object],
    elapsed: float,
) -> Dict[str, float]:
    """Calculate process CPU, run-queue, and context-switch deltas."""
    if not previous or elapsed <= 0.0:
        return {}
    if previous.get('start_ticks') != current.get('start_ticks'):
        return {'restarted': 1.0}
    result = {}
    try:
        ticks_per_second = float(os.sysconf('SC_CLK_TCK'))
        tick_delta = int(current['cpu_ticks']) - int(previous['cpu_ticks'])
        if tick_delta >= 0:
            result['cpu_percent_one_core'] = (
                100.0 * tick_delta / ticks_per_second / elapsed
            )
    except (KeyError, TypeError, ValueError, OSError):
        pass
    for source, destination, scale in (
        ('runqueue_ns', 'runqueue_percent_one_core', 1e9),
        ('runtime_ns', 'runtime_percent_one_core', 1e9),
        ('voluntary_context_switches', 'voluntary_context_switches', 1.0),
        (
            'nonvoluntary_context_switches',
            'nonvoluntary_context_switches',
            1.0,
        ),
    ):
        before = previous.get(source)
        after = current.get(source)
        if before is None or after is None:
            continue
        delta = int(after) - int(before)
        if delta < 0:
            continue
        if source.endswith('_ns'):
            result[destination] = 100.0 * delta / scale / elapsed
        else:
            result[destination] = float(delta)
    return result


def read_boot_id(path: str = '/proc/sys/kernel/random/boot_id') -> str:
    """Read the kernel boot identifier."""
    try:
        return Path(path).read_text(encoding='ascii').strip()
    except OSError:
        return 'unavailable'


def read_uptime(path: str = '/proc/uptime') -> Optional[float]:
    """Read monotonic uptime seconds from procfs."""
    try:
        return float(Path(path).read_text(encoding='ascii').split()[0])
    except (OSError, ValueError, IndexError):
        return None


def read_interface_carrier(
    interface: str,
    sys_class_net: str = '/sys/class/net',
) -> Optional[int]:
    """Return kernel carrier state without invoking a subprocess."""
    if not _INTERFACE_NAME.fullmatch(interface):
        return None
    try:
        return int(
            Path(sys_class_net, interface, 'carrier')
            .read_text(encoding='ascii')
            .strip()
        )
    except (OSError, ValueError):
        return None


def read_interface_metadata(
    interface: str,
    sys_class_net: str = '/sys/class/net',
) -> Dict[str, object]:
    """Read stable interface identity/driver metadata from sysfs."""
    if not _INTERFACE_NAME.fullmatch(interface):
        return {}
    root = Path(sys_class_net, interface)
    result = {}
    for name in ('address', 'mtu', 'tx_queue_len'):
        try:
            result[name] = (root / name).read_text(encoding='ascii').strip()
        except OSError:
            continue
    try:
        result['driver'] = (
            root / 'device' / 'driver'
        ).resolve(strict=True).name
    except OSError:
        result['driver'] = 'unavailable'
    return result


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


def counter_delta_details(
    previous: Optional[Dict[str, int]],
    current: Optional[Dict[str, int]],
    names: Optional[Iterable[str]] = None,
) -> Tuple[Dict[str, int], Tuple[str, ...]]:
    """Return deltas plus names whose cumulative counter moved backwards."""
    if previous is None or current is None:
        return {}, ()
    deltas = {}
    resets = []
    for name in names or current.keys():
        if name not in previous or name not in current:
            continue
        before = int(previous[name])
        after = int(current[name])
        if after < before:
            resets.append(name)
            continue
        deltas[name] = after - before
    return deltas, tuple(sorted(resets))


def parse_sockstat(output: str) -> Dict[str, int]:
    """Parse socket allocation/memory counters from ``/proc/net/sockstat``."""
    result = {}
    for line in (output or '').splitlines():
        protocol, separator, remainder = line.partition(':')
        if not separator:
            continue
        fields = remainder.split()
        for index in range(0, len(fields) - 1, 2):
            try:
                result[f'{protocol.strip()}.{fields[index]}'] = int(
                    fields[index + 1]
                )
            except ValueError:
                continue
    return result


def read_sockstat(path: str = '/proc/net/sockstat') -> Dict[str, int]:
    """Read system socket allocation and UDP memory use."""
    try:
        return parse_sockstat(Path(path).read_text(encoding='ascii'))
    except OSError:
        return {}


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
