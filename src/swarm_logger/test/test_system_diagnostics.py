from pathlib import Path

from swarm_logger.system_diagnostics import (
    counter_deltas,
    cpu_usage_percent,
    decode_throttled_state,
    parse_iw_link,
    parse_iw_station_dump,
    parse_throttled_output,
    read_cpu_times,
    read_memory,
    read_net_counters,
    signal_label,
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
