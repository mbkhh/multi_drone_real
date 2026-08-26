import os
from pathlib import Path

import pytest

from swarm_logger.system_diagnostics import (
    counter_delta_details,
    counter_deltas,
    cpu_usage_per_core,
    cpu_usage_percent,
    decode_throttled_state,
    parse_iw_link,
    parse_iw_station_dump,
    parse_iw_survey_dump,
    parse_pressure,
    parse_proc_net_snmp,
    parse_process_stat,
    parse_sockstat,
    parse_softnet_stat,
    parse_throttled_output,
    process_deltas,
    read_cpu_times_per_core,
    read_cpu_times,
    read_memory,
    read_net_counters,
    signal_label,
    survey_percentages,
    wireless_ratios,
)


def test_throttled_output_is_strict_and_decoded():
    value = parse_throttled_output('throttled=0x50005\n')

    assert value == 0x50005
    assert decode_throttled_state(value) == (
        'under-voltage now',
        'throttling active now',
        'under-voltage occurred since boot',
        'throttling occurred since boot',
    )
    assert parse_throttled_output('unexpected output') is None
    assert decode_throttled_state(None) == ()


def test_iw_link_parser_handles_connected_and_disconnected_samples():
    connected = parse_iw_link(
        '''Connected to aa:bb:cc:dd:ee:ff (on wlan0)\n'''
        '''\tSSID: drone-net\n'''
        '''\tfreq: 5180\n'''
        '''\tsignal: -61 dBm\n'''
        '''\ttx bitrate: 433.3 MBit/s\n'''
        '''\trx bitrate: 390.0 MBit/s\n'''
    )

    assert connected == {
        'connected': True,
        'ssid': 'drone-net',
        'bssid': 'aa:bb:cc:dd:ee:ff',
        'frequency_mhz': 5180,
        'signal_dbm': -61.0,
        'tx_bitrate_mbps': 433.3,
        'rx_bitrate_mbps': 390.0,
    }
    assert parse_iw_link('Not connected.\n')['connected'] is False


def test_station_dump_parser_tolerates_unsupported_fields():
    parsed = parse_iw_station_dump(
        '''Station aa:bb:cc:dd:ee:ff (on wlan0)\n'''
        '''\ttx retries:\t77\n'''
        '''\ttx failed:\t3\n'''
        '''\trx drop misc:\t9\n'''
    )

    assert parsed == {'tx_retries': 77, 'tx_failed': 3, 'rx_drop_misc': 9}


def test_cpu_usage_uses_tick_deltas_and_rejects_resets():
    assert cpu_usage_percent((100, 200), (125, 300)) == 75.0
    assert cpu_usage_percent(None, (125, 300)) is None
    assert cpu_usage_percent((125, 300), (100, 200)) is None


def test_proc_readers_parse_fixture_files(tmp_path):
    stat = tmp_path / 'stat'
    stat.write_text('cpu  10 2 8 70 10 0 0 0 0 0\n', encoding='ascii')
    meminfo = tmp_path / 'meminfo'
    meminfo.write_text(
        'MemTotal:       1000 kB\nMemAvailable:    250 kB\n',
        encoding='ascii',
    )

    assert read_cpu_times(str(stat)) == (80, 100)
    assert read_memory(str(meminfo)) == {
        'total_bytes': 1024000.0,
        'available_bytes': 256000.0,
        'used_percent': 75.0,
    }


def test_network_counters_and_deltas_handle_reset(tmp_path):
    statistics = tmp_path / 'wlan0' / 'statistics'
    statistics.mkdir(parents=True)
    names = (
        'rx_bytes',
        'tx_bytes',
        'rx_packets',
        'tx_packets',
        'rx_errors',
        'tx_errors',
        'rx_dropped',
        'tx_dropped',
    )
    for index, name in enumerate(names):
        Path(statistics, name).write_text(str(100 + index), encoding='ascii')

    current = read_net_counters('wlan0', str(tmp_path))

    assert current['rx_bytes'] == 100
    assert read_net_counters('../wlan0', str(tmp_path)) is None
    assert counter_deltas({'rx_bytes': 200}, {'rx_bytes': 100}) == {
        'rx_bytes': 0
    }


def test_signal_labels_are_conservative():
    assert signal_label(None) == 'unknown'
    assert signal_label(-50) == 'strong'
    assert signal_label(-60) == 'good'
    assert signal_label(-70) == 'fair'
    assert signal_label(-80) == 'weak'


def test_station_counters_and_ratios_preserve_driver_meaning():
    parsed = parse_iw_station_dump(
        '''Station aa:bb:cc:dd:ee:ff (on wlan0)\n'''
        '''\tinactive time:\t12 ms\n'''
        '''\trx bytes:\t10000\n'''
        '''\trx packets:\t100\n'''
        '''\ttx bytes:\t20000\n'''
        '''\ttx packets:\t80\n'''
        '''\ttx retries:\t20\n'''
        '''\ttx failed:\t2\n'''
        '''\tbeacon loss:\t3\n'''
        '''\tbeacon rx:\t900\n'''
        '''\tsignal avg:\t-66 dBm\n'''
        '''\texpected throughput:\t120.5Mbps\n'''
        '''\ttx duration:\t1234 us\n'''
        '''\trx duration:\t4567 us\n'''
    )

    assert parsed['inactive_time_ms'] == 12
    assert parsed['signal_avg_dbm'] == -66.0
    assert parsed['expected_throughput_mbps'] == 120.5
    assert parsed['tx_duration_us'] == 1234
    assert wireless_ratios(parsed) == {
        'retries_per_tx_packet': 0.25,
        'failed_per_1000_tx_packets': 25.0,
    }
    assert wireless_ratios({'tx_packets': 0}) == {}


def test_survey_parser_selects_in_use_channel_and_calculates_airtime():
    parsed = parse_iw_survey_dump(
        '''Survey data from wlan0\n'''
        '''\tfrequency: 2412 MHz\n'''
        '''\tchannel active time: 999 ms\n'''
        '''\tfrequency: 5180 MHz [in use]\n'''
        '''\tnoise: -95 dBm\n'''
        '''\tchannel active time: 1000 ms\n'''
        '''\tchannel busy time: 800 ms\n'''
        '''\tchannel receive time: 300 ms\n'''
        '''\tchannel transmit time: 100 ms\n'''
        '''\tchannel BSS receive time: 200 ms\n'''
    )

    assert parsed == {
        'frequency_mhz': 5180,
        'noise_dbm': -95,
        'active_time_ms': 1000,
        'busy_time_ms': 800,
        'receive_time_ms': 300,
        'transmit_time_ms': 100,
        'bss_receive_time_ms': 200,
    }
    assert survey_percentages(parsed) == {
        'busy_percent': 80.0,
        'receive_percent': 30.0,
        'transmit_percent': 10.0,
        'bss_receive_percent': 20.0,
    }
    assert parse_iw_survey_dump('frequency: 2412 MHz\n') == {}


def test_kernel_network_and_pressure_parsers():
    snmp = parse_proc_net_snmp(
        'Ip: InReceives InDiscards\nIp: 100 3\n'
        'Udp: InDatagrams RcvbufErrors\nUdp: 80 4\n'
    )
    assert snmp == {
        'Ip.InReceives': 100,
        'Ip.InDiscards': 3,
        'Udp.InDatagrams': 80,
        'Udp.RcvbufErrors': 4,
    }
    assert parse_softnet_stat(
        '00000010 00000002 00000003 0\n'
        '00000020 00000004 00000005 0\n'
    ) == {'processed': 48, 'dropped': 6, 'time_squeeze': 8}
    assert parse_pressure(
        'some avg10=1.50 avg60=2.00 total=123\n'
        'full avg10=0.25 avg60=0.50 total=45\n'
    ) == {
        'some_avg10': 1.5,
        'some_avg60': 2.0,
        'some_total': 123.0,
        'full_avg10': 0.25,
        'full_avg60': 0.5,
        'full_total': 45.0,
    }
    assert parse_sockstat(
        'sockets: used 42\nUDP: inuse 7 mem 11\n'
    ) == {'sockets.used': 42, 'UDP.inuse': 7, 'UDP.mem': 11}


def test_per_core_cpu_reader_and_delta(tmp_path):
    before = tmp_path / 'before_stat'
    after = tmp_path / 'after_stat'
    before.write_text(
        'cpu 10 0 10 80 0\ncpu0 4 0 6 40 0\n', encoding='ascii'
    )
    after.write_text(
        'cpu 20 0 20 160 0\ncpu0 10 0 10 80 0\n', encoding='ascii'
    )

    previous = read_cpu_times_per_core(str(before))
    current = read_cpu_times_per_core(str(after))

    assert previous == {'cpu': (80, 100), 'cpu0': (40, 50)}
    assert cpu_usage_per_core(previous, current) == {
        'cpu': 20.0,
        'cpu0': 20.0,
    }


def test_counter_details_report_resets_without_fake_zero_deltas():
    delta, resets = counter_delta_details(
        {'ok': 10, 'reset': 20, 'missing': 30},
        {'ok': 15, 'reset': 2},
    )

    assert delta == {'ok': 5}
    assert resets == ('reset',)


def test_process_stat_parser_handles_parenthesis_in_command_name():
    tail = ['0'] * 37
    tail[0] = 'S'
    tail[1] = '10'
    tail[11] = '30'
    tail[12] = '12'
    tail[17] = '8'
    tail[19] = '999'
    tail[21] = '123'
    tail[36] = '3'

    parsed = parse_process_stat(
        '77 (control ) worker) ' + ' '.join(tail)
    )

    assert parsed == {
        'pid': 77,
        'comm': 'control ) worker',
        'state': 'S',
        'ppid': 10,
        'cpu_ticks': 42,
        'num_threads': 8,
        'start_ticks': 999,
        'rss_pages': 123,
        'processor': 3,
    }


def test_process_delta_calculates_cpu_queue_and_context_switches():
    previous = {
        'start_ticks': 10,
        'cpu_ticks': 100,
        'runtime_ns': 1_000_000_000,
        'runqueue_ns': 100_000_000,
        'voluntary_context_switches': 5,
        'nonvoluntary_context_switches': 2,
    }
    current = {
        'start_ticks': 10,
        'cpu_ticks': 150,
        'runtime_ns': 2_000_000_000,
        'runqueue_ns': 600_000_000,
        'voluntary_context_switches': 9,
        'nonvoluntary_context_switches': 5,
    }

    delta = process_deltas(previous, current, 2.0)
    expected_cpu = 100.0 * 50 / float(os.sysconf('SC_CLK_TCK')) / 2.0

    assert delta['cpu_percent_one_core'] == pytest.approx(expected_cpu)
    assert delta['runtime_percent_one_core'] == pytest.approx(50.0)
    assert delta['runqueue_percent_one_core'] == pytest.approx(25.0)
    assert delta['voluntary_context_switches'] == 4.0
    assert delta['nonvoluntary_context_switches'] == 3.0
    assert process_deltas(
        previous, {'start_ticks': 11}, 2.0
    ) == {'restarted': 1.0}
