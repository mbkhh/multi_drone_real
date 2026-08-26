"""Background-only collection of companion-computer health diagnostics."""

from __future__ import annotations

import os
import platform
import time
from pathlib import Path

from swarm_logger.background_diagnostics import run_probe
from swarm_logger.system_diagnostics import (
    counter_delta_details,
    cpu_usage_per_core,
    decode_throttled_state,
    find_processes,
    parse_iw_link,
    parse_iw_station_dump,
    parse_iw_survey_dump,
    parse_throttled_output,
    process_deltas,
    read_boot_id,
    read_cpu_times_per_core,
    read_interface_carrier,
    read_interface_metadata,
    read_interface_state,
    read_load_average,
    read_memory,
    read_net_counters,
    read_pressure,
    read_process_metrics,
    read_proc_net_snmp,
    read_sockstat,
    read_softnet_stat,
    read_temperature,
    read_uptime,
    survey_percentages,
    wireless_ratios,
)


_STATION_COUNTERS = (
    'rx_bytes',
    'rx_packets',
    'tx_bytes',
    'tx_packets',
    'tx_retries',
    'tx_failed',
    'rx_drop_misc',
    'beacon_loss',
    'beacon_rx',
    'connected_time_s',
    'tx_duration_us',
    'rx_duration_us',
)
_SURVEY_COUNTERS = (
    'active_time_ms',
    'busy_time_ms',
    'receive_time_ms',
    'transmit_time_ms',
    'bss_receive_time_ms',
)


def _probe_summary(result):
    """Return a compact, serializable subprocess result summary."""
    stderr = (result.stderr or '').replace('\n', ' ').strip()
    return {
        'status': result.status,
        'duration_s': result.duration_s,
        'returncode': result.returncode,
        'stderr': stderr[-160:],
    }


def _loaded_rmw_libraries(pid):
    """List RMW shared libraries mapped by a process, when procfs allows it."""
    try:
        lines = Path('/proc', str(int(pid)), 'maps').read_text(
            encoding='utf-8', errors='replace'
        ).splitlines()
    except (OSError, ValueError):
        return ()
    names = {
        Path(line.rsplit(None, 1)[-1]).name
        for line in lines
        if 'librmw_' in line and '/' in line
    }
    return tuple(sorted(names))


class LocalHealthCollector:
    """Collect blocking and procfs diagnostics away from ROS callbacks."""

    def __init__(
        self,
        interface,
        probe_timeout=0.4,
        controller_process_match='control_node',
        monitor_microxrce_agent=True,
    ):
        self.interface = str(interface)
        self.probe_timeout = float(probe_timeout)
        self.controller_process_match = str(controller_process_match)
        self.monitor_microxrce_agent = bool(monitor_microxrce_agent)

        self._last_sample_at = time.monotonic()
        self._previous_cpu = read_cpu_times_per_core()
        self._previous_net = read_net_counters(self.interface)
        self._previous_station = None
        self._previous_survey = None
        self._previous_snmp = read_proc_net_snmp()
        self._previous_softnet = read_softnet_stat()
        self._previous_processes = {}
        self._previous_bssid = None

        self._boot_id = read_boot_id()
        self._interface_metadata = read_interface_metadata(self.interface)

    @staticmethod
    def _advance_baseline(previous, current):
        """Keep a useful cumulative-counter baseline across failed reads."""
        return current if current else previous

    def _collect_wifi(self):
        commands = {
            'iw_link': ['iw', 'dev', self.interface, 'link'],
            'iw_station': [
                'iw', 'dev', self.interface, 'station', 'dump'
            ],
            'iw_survey': ['iw', 'dev', self.interface, 'survey', 'dump'],
            'iw_power_save': [
                'iw', 'dev', self.interface, 'get', 'power_save'
            ],
            'vcgencmd_throttled': ['vcgencmd', 'get_throttled'],
        }
        results = {
            name: run_probe(command, timeout=self.probe_timeout)
            for name, command in commands.items()
        }

        link_result = results['iw_link']
        link = parse_iw_link(
            link_result.stdout if link_result.status == 'ok' else ''
        )
        link['probe_status'] = link_result.status

        station_result = results['iw_station']
        station = parse_iw_station_dump(
            station_result.stdout if station_result.status == 'ok' else ''
        )
        station_delta, station_resets = counter_delta_details(
            self._previous_station, station, _STATION_COUNTERS
        )
        if station_result.status == 'ok' and station:
            self._previous_station = station

        survey_result = results['iw_survey']
        survey = parse_iw_survey_dump(
            survey_result.stdout if survey_result.status == 'ok' else ''
        )
        survey_delta, survey_resets = counter_delta_details(
            self._previous_survey, survey, _SURVEY_COUNTERS
        )
        if survey_result.status == 'ok' and survey:
            self._previous_survey = survey

        power_output = results['iw_power_save'].stdout.lower()
        if 'power save: on' in power_output:
            power_save = True
        elif 'power save: off' in power_output:
            power_save = False
        else:
            power_save = None

        throttle_result = results['vcgencmd_throttled']
        throttled_value = parse_throttled_output(throttle_result.stdout)
        bssid = link.get('bssid')
        bssid_changed = bool(
            self._previous_bssid
            and bssid
            and bssid != self._previous_bssid
        )
        if bssid:
            self._previous_bssid = bssid

        return {
            'link': link,
            'station': station,
            'station_delta': station_delta,
            'station_resets': station_resets,
            'station_ratios': wireless_ratios(station_delta),
            'survey': survey,
            'survey_delta': survey_delta,
            'survey_resets': survey_resets,
            'survey_percentages': survey_percentages(survey_delta),
            'power_save': power_save,
            'bssid_changed': bssid_changed,
            'throttled_value': throttled_value,
            'throttled_reasons': decode_throttled_state(throttled_value),
            'probes': {
                name: _probe_summary(result)
                for name, result in results.items()
            },
        }

    def _collect_processes(self, elapsed):
        selected = [('logger', read_process_metrics(os.getpid()))]
        if self.controller_process_match:
            selected.extend(
                ('controller', process)
                for process in find_processes(
                    (self.controller_process_match,)
                )
                if int(process.get('pid', -1)) != os.getpid()
            )
        if self.monitor_microxrce_agent:
            agent_matches = {}
            for token in ('MicroXRCEAgent', 'micro_ros_agent'):
                for process in find_processes((token,)):
                    pid = int(process.get('pid', -1))
                    if pid != os.getpid():
                        agent_matches[pid] = process
            selected.extend(
                ('microxrce_agent', process)
                for _, process in sorted(agent_matches.items())
            )

        current_processes = {}
        snapshots = []
        for label, process in selected:
            if not process:
                continue
            pid = int(process['pid'])
            key = (label, pid)
            process = dict(process)
            process['label'] = label
            process['delta'] = process_deltas(
                self._previous_processes.get(key), process, elapsed
            )
            process['rmw_libraries'] = _loaded_rmw_libraries(pid)
            current_processes[key] = process
            snapshots.append(process)
        self._previous_processes = current_processes
        return tuple(snapshots)

    def collect(self):
        """Collect one correlated local-health sample."""
        started = time.monotonic()
        elapsed = max(1e-6, started - self._last_sample_at)
        self._last_sample_at = started

        current_cpu = read_cpu_times_per_core()
        cpu_percent = cpu_usage_per_core(self._previous_cpu, current_cpu)
        self._previous_cpu = self._advance_baseline(
            self._previous_cpu, current_cpu
        )

        current_net = read_net_counters(self.interface)
        net_delta, net_resets = counter_delta_details(
            self._previous_net, current_net
        )
        self._previous_net = self._advance_baseline(
            self._previous_net, current_net
        )

        current_snmp = read_proc_net_snmp()
        snmp_delta, snmp_resets = counter_delta_details(
            self._previous_snmp, current_snmp
        )
        self._previous_snmp = self._advance_baseline(
            self._previous_snmp, current_snmp
        )

        current_softnet = read_softnet_stat()
        softnet_delta, softnet_resets = counter_delta_details(
            self._previous_softnet, current_softnet
        )
        self._previous_softnet = self._advance_baseline(
            self._previous_softnet, current_softnet
        )

        wifi = self._collect_wifi()
        processes = self._collect_processes(elapsed)
        completed = time.monotonic()
        return {
            'sample_started_monotonic': started,
            'sample_monotonic': completed,
            'window_s': elapsed,
            'collection_duration_s': completed - started,
            'hostname': platform.node(),
            'kernel': platform.release(),
            'machine': platform.machine(),
            'boot_id': self._boot_id,
            'uptime_s': read_uptime(),
            'load': read_load_average(),
            'cpu_count': os.cpu_count() or 1,
            'cpu_percent': cpu_percent,
            'memory': read_memory(),
            'temperature_c': read_temperature(),
            'pressure': {
                resource: read_pressure(resource)
                for resource in ('cpu', 'memory', 'io')
            },
            'interface': {
                'name': self.interface,
                'state': read_interface_state(self.interface),
                'carrier': read_interface_carrier(self.interface),
                'metadata': self._interface_metadata,
                'net_delta': net_delta,
                'net_resets': net_resets,
            },
            'wifi': wifi,
            'snmp_delta': snmp_delta,
            'snmp_resets': snmp_resets,
            'softnet_delta': softnet_delta,
            'softnet_resets': softnet_resets,
            'sockstat': read_sockstat(),
            'processes': processes,
        }
