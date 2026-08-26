import math
from types import SimpleNamespace

import pytest

from swarm_logger.diagnostic_trackers import (
    DDSWriterTracker,
    StampedStreamTracker,
    TransformFrameTracker,
    format_gid,
    stamp_to_nanoseconds,
)


def _stamp(nanoseconds):
    seconds, remainder = divmod(int(nanoseconds), 1_000_000_000)
    return SimpleNamespace(sec=seconds, nanosec=remainder)


def _transform(
    *,
    parent='world',
    child='x500_depth_1/odom',
    stamp_ns=1_000_000_000,
    xyz=(0.0, 0.0, 0.0),
    quaternion=(0.0, 0.0, 0.0, 1.0),
):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=parent, stamp=_stamp(stamp_ns)),
        child_frame_id=child,
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
            rotation=SimpleNamespace(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3],
            ),
        ),
    )


def test_stamp_and_gid_helpers_accept_ros_like_values_and_bad_inputs():
    assert stamp_to_nanoseconds(_stamp(3_000_000_004)) == 3_000_000_004
    assert stamp_to_nanoseconds(SimpleNamespace(sec='2', nanosec='7')) == (
        2_000_000_007
    )
    assert stamp_to_nanoseconds(object()) == 0

    assert format_gid(bytes((0x01, 0xAB, 0xFF))) == '01abff'
    assert format_gid([256, -1, 2]) == '00ff02'
    assert format_gid([]) == 'unknown'
    assert format_gid(None) == 'unknown'
    assert format_gid(['not-an-integer']) == 'unknown'


def test_stamped_stream_separates_arrivals_from_source_stamp_progress():
    tracker = StampedStreamTracker()

    assert tracker.observe(1_000_000_000, 10.0, 3_000_000_000) == 'first'
    assert tracker.observe(1_000_000_000, 10.1, 3_200_000_000) == 'repeated'
    assert tracker.observe(500_000_000, 10.2, 3_300_000_000) == 'regressed'
    assert tracker.observe(2_000_000_000, 10.8, 4_400_000_000) == 'advanced'

    snapshot = tracker.snapshot(11.0, reset=False)

    assert snapshot['count'] == 4
    assert snapshot['changed'] == 3
    assert snapshot['advanced'] == 2
    assert snapshot['repeated'] == 1
    assert snapshot['regressed'] == 1
    assert snapshot['zero_stamps'] == 0
    assert snapshot['arrival_age'] == pytest.approx(0.2)
    assert snapshot['change_age'] == pytest.approx(0.2)
    assert snapshot['advance_age'] == pytest.approx(0.2)
    assert snapshot['max_arrival_gap'] == pytest.approx(0.6)
    assert snapshot['max_arrival_gap_at'] == pytest.approx(10.8)
    assert snapshot['max_change_gap'] == pytest.approx(0.6)
    assert snapshot['max_advance_gap'] == pytest.approx(0.8)
    assert snapshot['max_advance_gap_at'] == pytest.approx(10.8)
    assert snapshot['max_source_step'] == pytest.approx(1.5)
    assert snapshot['last_stamp_ns'] == 2_000_000_000
    assert snapshot['source_minus_local_s'] == pytest.approx(-2.4)
    assert snapshot['last_excess_delay_s'] == pytest.approx(0.4)
    assert snapshot['max_excess_delay_s'] == pytest.approx(0.8)


def test_repeated_and_zero_stamps_do_not_refresh_advance_freshness():
    tracker = StampedStreamTracker()
    tracker.observe(4_000_000_000, 20.0)
    tracker.observe(4_000_000_000, 20.4)
    tracker.observe(0, 20.6)

    snapshot = tracker.snapshot(20.7, reset=False)

    assert snapshot['arrival_age'] == pytest.approx(0.1)
    assert snapshot['advance_age'] == pytest.approx(0.7)
    assert snapshot['last_stamp_ns'] == 4_000_000_000
    assert snapshot['repeated'] == 1
    assert snapshot['zero_stamps'] == 1


def test_stream_snapshot_reset_clears_window_but_preserves_freshness():
    tracker = StampedStreamTracker()
    tracker.observe(1_000_000_000, 2.0)
    tracker.observe(2_000_000_000, 3.0)

    first = tracker.snapshot(4.0)
    second = tracker.snapshot(5.0, reset=False)

    assert first['count'] == 2
    assert first['advanced'] == 2
    assert first['max_advance_gap'] == pytest.approx(1.0)
    assert second['count'] == 0
    assert second['advanced'] == 0
    assert second['max_advance_gap'] is None
    assert second['last_stamp_ns'] == 2_000_000_000
    assert second['arrival_age'] == pytest.approx(2.0)
    assert second['advance_age'] == pytest.approx(2.0)


def test_arrival_only_stream_does_not_invent_zero_source_stamps():
    tracker = StampedStreamTracker()
    tracker.observe_arrival(1.0)
    tracker.observe_arrival(1.25)

    snapshot = tracker.snapshot(1.5, reset=False)

    assert snapshot['count'] == 2
    assert snapshot['max_arrival_gap'] == pytest.approx(0.25)
    assert snapshot['zero_stamps'] == 0
    assert snapshot['advance_age'] is None


def test_transform_tracker_measures_pose_steps_speed_and_rotation():
    tracker = TransformFrameTracker()
    first = _transform(stamp_ns=1_000_000_000)
    second = _transform(
        stamp_ns=3_000_000_000,
        xyz=(3.0, 4.0, 0.0),
        quaternion=(0.0, 0.0, 1.0, 0.0),
    )

    assert tracker.observe(first, 'writer-a', 10.0, 2_000_000_000) == []
    assert tracker.observe(second, 'writer-a', 10.1, 4_000_000_000) == []

    snapshot = tracker.snapshot(10.2, reset=False)
    assert snapshot['parent'] == 'world'
    assert snapshot['writer_count'] == 1
    assert snapshot['last_writer'] == 'writer-a'
    assert snapshot['max_position_step'] == pytest.approx(5.0)
    assert snapshot['max_linear_speed'] == pytest.approx(2.5)
    assert snapshot['max_angular_step'] == pytest.approx(math.pi)


def test_repeated_transform_stamp_does_not_replace_last_advanced_pose():
    tracker = TransformFrameTracker()
    tracker.observe(_transform(stamp_ns=1_000_000_000), 'writer-a', 1.0, 0)
    tracker.observe(
        _transform(stamp_ns=1_000_000_000, xyz=(100.0, 0.0, 0.0)),
        'writer-a',
        1.1,
        0,
    )
    tracker.observe(
        _transform(stamp_ns=2_000_000_000, xyz=(1.0, 0.0, 0.0)),
        'writer-a',
        1.2,
        0,
    )

    snapshot = tracker.snapshot(1.3, reset=False)
    assert snapshot['repeated'] == 1
    assert snapshot['max_position_step'] == pytest.approx(1.0)
    assert snapshot['max_linear_speed'] == pytest.approx(1.0)


def test_transform_tracker_reports_parent_and_writer_authority_changes():
    tracker = TransformFrameTracker()
    tracker.observe(_transform(), 'writer-a', 1.0, 2_000_000_000)

    anomalies = tracker.observe(
        _transform(parent='map', stamp_ns=2_000_000_000),
        'writer-b',
        2.0,
        3_000_000_000,
    )
    snapshot = tracker.snapshot(2.1, reset=False)

    assert "parent changed 'world'->'map'" in anomalies
    assert 'DDS writer authority changed' in anomalies
    assert snapshot['parent_changes'] == 1
    assert snapshot['writer_changes'] == 1
    assert snapshot['writer_count'] == 2
    assert snapshot['last_writer'] == 'writer-b'


@pytest.mark.parametrize(
    ('transform', 'expected_anomaly', 'counter'),
    (
        (
            _transform(parent='', child=''),
            'empty frame id',
            'empty_frame_ids',
        ),
        (
            _transform(parent='same', child='same'),
            'self-referencing TF edge',
            'self_edges',
        ),
        (
            _transform(xyz=(float('nan'), 0.0, 0.0)),
            'non-finite transform value',
            'nonfinite',
        ),
        (
            _transform(quaternion=(0.0, 0.0, 0.0, 0.0)),
            'zero quaternion',
            'zero_quaternions',
        ),
        (
            _transform(quaternion=(0.0, 0.0, 0.0, 2.0)),
            'quaternion norm=2.000',
            'nonunit_quaternions',
        ),
    ),
)
def test_transform_tracker_classifies_invalid_transforms(
    transform, expected_anomaly, counter
):
    tracker = TransformFrameTracker()

    anomalies = tracker.observe(transform, 'unknown', 1.0, 2_000_000_000)
    snapshot = tracker.snapshot(1.1, reset=False)

    assert expected_anomaly in anomalies
    assert snapshot[counter] == 1
    assert snapshot['writer_count'] == 0


def test_transform_snapshot_reset_keeps_authority_but_clears_window_anomalies():
    tracker = TransformFrameTracker()
    tracker.observe(_transform(), 'writer-a', 1.0, 2_000_000_000)
    tracker.observe(
        _transform(parent='map', stamp_ns=2_000_000_000),
        'writer-b',
        2.0,
        3_000_000_000,
    )

    first = tracker.snapshot(2.1)
    second = tracker.snapshot(2.2, reset=False)

    assert first['parent_changes'] == 1
    assert first['writer_changes'] == 1
    assert second['parent_changes'] == 0
    assert second['writer_changes'] == 0
    assert second['parent'] == 'map'
    assert second['writer_count'] == 2


def test_dds_writer_tracker_detects_sequence_loss_duplicate_and_regression():
    tracker = DDSWriterTracker()
    observations = (
        (10, 1_000_000_000, 1_100_000_000, 1.0, 1_300_000_000, ('a',)),
        (13, 2_000_000_000, 2_500_000_000, 2.0, 3_000_000_000, ('a', 'b')),
        (13, 2_000_000_000, 2_600_000_000, 2.1, 3_200_000_000, ('a', 'b')),
        (12, 1_500_000_000, 2_700_000_000, 2.2, 3_300_000_000, ()),
    )
    for observation in observations:
        tracker.observe(*observation)

    snapshot = tracker.snapshot(2.3, reset=False)

    assert snapshot['missing_sequences'] == 2
    assert snapshot['sequence_duplicates'] == 1
    assert snapshot['sequence_regressions'] == 1
    assert snapshot['max_callback_queue_delay_s'] == pytest.approx(0.6)
    assert snapshot['max_middleware_delay_s'] == pytest.approx(1.2)
    assert snapshot['children'] == ('a', 'b')
    assert snapshot['message_sizes'] == {1: 1, 2: 2, 0: 1}
    assert snapshot['repeated'] == 1
    assert snapshot['regressed'] == 1


def test_dds_writer_tracker_tolerates_unsupported_metadata_and_resets_window():
    tracker = DDSWriterTracker()
    tracker.observe(None, None, None, 1.0, 1_000_000_000, ('odom',))

    first = tracker.snapshot(1.1)
    second = tracker.snapshot(1.2, reset=False)

    assert first['zero_stamps'] == 1
    assert first['missing_sequences'] == 0
    assert first['max_callback_queue_delay_s'] is None
    assert first['max_middleware_delay_s'] is None
    assert first['message_sizes'] == {1: 1}
    assert second['count'] == 0
    assert second['message_sizes'] == {}
    assert second['children'] == ('odom',)
