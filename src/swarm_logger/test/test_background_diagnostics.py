"""Tests for optional command and Wi-Fi event diagnostics."""

import sys

from swarm_logger.background_diagnostics import classify_iw_event, run_probe


def test_iw_event_classifier_orders_disruptive_events_first():
    assert classify_iw_event('wlan0: disconnected from aa:bb') == 'disconnect'
    assert classify_iw_event('wlan0: deauth event') == 'deauth'
    assert classify_iw_event('wlan0: disassoc event') == 'disassoc'
    assert classify_iw_event('wlan0: roam to aa:bb') == 'roam'
    assert classify_iw_event('wlan0: connected to aa:bb') == 'connect'
    assert classify_iw_event('wlan0: CQM RSSI threshold') == 'cqm'
    assert classify_iw_event('wlan0: scan started') == 'scan'
    assert classify_iw_event('wlan0: regulatory change') == 'other'


def test_run_probe_distinguishes_missing_success_empty_and_nonzero():
    missing = run_probe(['/definitely/not/a/real/command'])
    success = run_probe([sys.executable, '-c', 'print("ready")'])
    empty = run_probe([sys.executable, '-c', 'pass'])
    failed = run_probe(
        [sys.executable, '-c', 'import sys; print("bad"); sys.exit(7)']
    )

    assert missing.status == 'missing'
    assert success.status == 'ok'
    assert success.stdout == 'ready'
    assert empty.status == 'empty'
    assert failed.status == 'nonzero_exit'
    assert failed.returncode == 7
    assert failed.stdout == 'bad'


def test_run_probe_reports_timeout():
    result = run_probe(
        [sys.executable, '-c', 'import time; time.sleep(1)'], timeout=0.01
    )

    assert result.status == 'timeout'
