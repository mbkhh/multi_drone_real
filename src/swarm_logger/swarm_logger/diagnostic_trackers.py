"""Bounded, ROS-independent stream trackers used by ``swarm_logger``."""

from __future__ import annotations

import math
from collections import Counter


def stamp_to_nanoseconds(stamp) -> int:
    """Convert a ROS ``builtin_interfaces/Time``-like object to nanoseconds."""
    try:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return 0


def format_gid(gid) -> str:
    """Return a stable hexadecimal DDS endpoint identifier when available."""
    if gid is None:
        return 'unknown'
    try:
        values = bytes(gid)
    except (TypeError, ValueError):
        try:
            values = bytes(int(item) & 0xff for item in gid)
        except (TypeError, ValueError):
            return 'unknown'
    return values.hex() if values else 'unknown'


class StampedStreamTracker:
    """Track arrival timing separately from source-stamp progression."""

    def __init__(self):
        self.last_arrival = None
        self.last_change = None
        self.last_advance = None
        self.last_stamp_ns = None
        self.minimum_arrival_minus_source_ns = None
        self.last_arrival_minus_source_ns = None
        self.last_excess_delay_ns = None
        self._reset_window()

    def _reset_window(self):
        self.count = 0
        self.changed = 0
        self.advanced = 0
        self.repeated = 0
        self.regressed = 0
        self.zero_stamps = 0
        self.max_arrival_gap = None
        self.max_change_gap = None
        self.max_advance_gap = None
        self.max_source_step = None
        self.max_excess_delay_ns = None
        self.max_arrival_gap_at = None
        self.max_advance_gap_at = None

    def observe_arrival(self, observed_monotonic: float):
        """Record delivery timing for a stream that has no single source stamp."""
        self.count += 1
        if self.last_arrival is not None:
            gap = max(0.0, observed_monotonic - self.last_arrival)
            if self.max_arrival_gap is None or gap > self.max_arrival_gap:
                self.max_arrival_gap = gap
                self.max_arrival_gap_at = observed_monotonic
        self.last_arrival = observed_monotonic

    @staticmethod
    def _maximum(current, candidate):
        if current is None or candidate > current:
            return candidate
        return current

    def observe(
        self,
        source_stamp_ns: int,
        observed_monotonic: float,
        observed_wall_ns: int | None = None,
    ):
        """Record one sample without retaining its payload."""
        self.observe_arrival(observed_monotonic)

        if source_stamp_ns <= 0:
            self.zero_stamps += 1
            return 'zero'

        relation = 'first'
        previous_stamp = self.last_stamp_ns
        if previous_stamp is None:
            self.changed += 1
            self.advanced += 1
            self.last_change = observed_monotonic
            self.last_advance = observed_monotonic
        elif source_stamp_ns == previous_stamp:
            relation = 'repeated'
            self.repeated += 1
        else:
            self.changed += 1
            if self.last_change is not None:
                change_gap = max(0.0, observed_monotonic - self.last_change)
                self.max_change_gap = self._maximum(
                    self.max_change_gap, change_gap
                )
            self.last_change = observed_monotonic

            if source_stamp_ns > previous_stamp:
                relation = 'advanced'
                self.advanced += 1
                if self.last_advance is not None:
                    advance_gap = max(
                        0.0, observed_monotonic - self.last_advance
                    )
                    if (
                        self.max_advance_gap is None
                        or advance_gap > self.max_advance_gap
                    ):
                        self.max_advance_gap = advance_gap
                        self.max_advance_gap_at = observed_monotonic
                source_step = (source_stamp_ns - previous_stamp) / 1e9
                self.max_source_step = self._maximum(
                    self.max_source_step, source_step
                )
                self.last_advance = observed_monotonic
            else:
                relation = 'regressed'
                self.regressed += 1

        self.last_stamp_ns = source_stamp_ns

        if observed_wall_ns is not None:
            offset_ns = int(observed_wall_ns) - source_stamp_ns
            self.last_arrival_minus_source_ns = offset_ns
            if (
                self.minimum_arrival_minus_source_ns is None
                or offset_ns < self.minimum_arrival_minus_source_ns
            ):
                self.minimum_arrival_minus_source_ns = offset_ns
            excess_ns = max(
                0, offset_ns - self.minimum_arrival_minus_source_ns
            )
            self.last_excess_delay_ns = excess_ns
            self.max_excess_delay_ns = self._maximum(
                self.max_excess_delay_ns, excess_ns
            )
        return relation

    def snapshot(self, now_monotonic: float, reset: bool = True):
        """Return persistent freshness plus current-window counters."""
        result = {
            'count': self.count,
            'changed': self.changed,
            'advanced': self.advanced,
            'repeated': self.repeated,
            'regressed': self.regressed,
            'zero_stamps': self.zero_stamps,
            'arrival_age': (
                None
                if self.last_arrival is None
                else max(0.0, now_monotonic - self.last_arrival)
            ),
            'change_age': (
                None
                if self.last_change is None
                else max(0.0, now_monotonic - self.last_change)
            ),
            'advance_age': (
                None
                if self.last_advance is None
                else max(0.0, now_monotonic - self.last_advance)
            ),
            'max_arrival_gap': self.max_arrival_gap,
            'max_change_gap': self.max_change_gap,
            'max_advance_gap': self.max_advance_gap,
            'max_source_step': self.max_source_step,
            'max_arrival_gap_at': self.max_arrival_gap_at,
            'max_advance_gap_at': self.max_advance_gap_at,
            'last_stamp_ns': self.last_stamp_ns,
            'source_minus_local_s': (
                None
                if self.last_arrival_minus_source_ns is None
                else -self.last_arrival_minus_source_ns / 1e9
            ),
            'last_excess_delay_s': (
                None
                if self.last_excess_delay_ns is None
                else self.last_excess_delay_ns / 1e9
            ),
            'max_excess_delay_s': (
                None
                if self.max_excess_delay_ns is None
                else self.max_excess_delay_ns / 1e9
            ),
        }
        if reset:
            self._reset_window()
        return result


class TransformFrameTracker:
    """Track timing, authority, parent, and numeric validity of a TF frame."""

    def __init__(self):
        self.stream = StampedStreamTracker()
        self.last_parent = None
        self.last_writer = None
        self.writers = set()
        self.last_translation = None
        self.last_quaternion = None
        self.last_pose_stamp_ns = None
        self._reset_window()

    def _reset_window(self):
        self.parent_changes = 0
        self.writer_changes = 0
        self.empty_frame_ids = 0
        self.self_edges = 0
        self.nonfinite = 0
        self.zero_quaternions = 0
        self.nonunit_quaternions = 0
        self.max_position_step = None
        self.max_angular_step = None
        self.max_linear_speed = None

    @staticmethod
    def _max(current, candidate):
        if current is None or candidate > current:
            return candidate
        return current

    def observe(
        self,
        transform,
        writer_gid: str,
        observed_monotonic: float,
        observed_wall_ns: int,
    ):
        """Observe one TransformStamped-like object and return anomalies."""
        anomalies = []
        parent = str(getattr(transform.header, 'frame_id', '')).strip()
        child = str(getattr(transform, 'child_frame_id', '')).strip()
        stamp_ns = stamp_to_nanoseconds(transform.header.stamp)
        relation = self.stream.observe(
            stamp_ns, observed_monotonic, observed_wall_ns
        )

        if not parent or not child:
            self.empty_frame_ids += 1
            anomalies.append('empty frame id')
        if parent and child and parent == child:
            self.self_edges += 1
            anomalies.append('self-referencing TF edge')
        if self.last_parent is not None and parent != self.last_parent:
            self.parent_changes += 1
            anomalies.append(
                f'parent changed {self.last_parent!r}->{parent!r}'
            )
        self.last_parent = parent

        if writer_gid != 'unknown':
            self.writers.add(writer_gid)
            if self.last_writer is not None and writer_gid != self.last_writer:
                self.writer_changes += 1
                anomalies.append('DDS writer authority changed')
            self.last_writer = writer_gid

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        xyz = (
            float(translation.x),
            float(translation.y),
            float(translation.z),
        )
        quaternion = (
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )
        if not all(math.isfinite(value) for value in xyz + quaternion):
            self.nonfinite += 1
            anomalies.append('non-finite transform value')
            return anomalies

        quaternion_norm = math.sqrt(
            sum(value * value for value in quaternion)
        )
        if quaternion_norm < 1e-9:
            self.zero_quaternions += 1
            anomalies.append('zero quaternion')
        elif abs(quaternion_norm - 1.0) > 0.05:
            self.nonunit_quaternions += 1
            anomalies.append(f'quaternion norm={quaternion_norm:.3f}')

        if relation in ('first', 'advanced') and self.last_translation is not None:
            position_step = math.sqrt(
                sum(
                    (current - previous) ** 2
                    for current, previous in zip(xyz, self.last_translation)
                )
            )
            self.max_position_step = self._max(
                self.max_position_step, position_step
            )

            if quaternion_norm >= 1e-9 and self.last_quaternion is not None:
                previous_norm = math.sqrt(
                    sum(value * value for value in self.last_quaternion)
                )
                if previous_norm >= 1e-9:
                    dot = abs(
                        sum(
                            current * previous
                            for current, previous in zip(
                                quaternion, self.last_quaternion
                            )
                        )
                        / (quaternion_norm * previous_norm)
                    )
                    angular_step = 2.0 * math.acos(min(1.0, dot))
                    self.max_angular_step = self._max(
                        self.max_angular_step, angular_step
                    )

            if self.last_pose_stamp_ns is not None:
                delta_seconds = (stamp_ns - self.last_pose_stamp_ns) / 1e9
                if delta_seconds > 0.0:
                    self.max_linear_speed = self._max(
                        self.max_linear_speed,
                        position_step / delta_seconds,
                    )

        if relation in ('first', 'advanced'):
            self.last_translation = xyz
            self.last_quaternion = quaternion
            self.last_pose_stamp_ns = stamp_ns
        return anomalies

    def snapshot(self, now_monotonic: float, reset: bool = True):
        result = self.stream.snapshot(now_monotonic, reset=reset)
        result.update({
            'parent': self.last_parent,
            'writer_count': len(self.writers),
            'last_writer': self.last_writer,
            'parent_changes': self.parent_changes,
            'writer_changes': self.writer_changes,
            'empty_frame_ids': self.empty_frame_ids,
            'self_edges': self.self_edges,
            'nonfinite': self.nonfinite,
            'zero_quaternions': self.zero_quaternions,
            'nonunit_quaternions': self.nonunit_quaternions,
            'max_position_step': self.max_position_step,
            'max_angular_step': self.max_angular_step,
            'max_linear_speed': self.max_linear_speed,
        })
        if reset:
            self._reset_window()
        return result


class DDSWriterTracker:
    """Track DDS MessageInfo sequence and timing for one writer GID."""

    def __init__(self):
        self.children = set()
        self.stream = StampedStreamTracker()
        self.last_publication_sequence = None
        self._reset_window()

    def _reset_window(self):
        self.missing_sequences = 0
        self.sequence_duplicates = 0
        self.sequence_regressions = 0
        self.max_callback_queue_delay_ns = None
        self.max_middleware_delay_ns = None
        self.message_sizes = Counter()

    @staticmethod
    def _max(current, candidate):
        if current is None or candidate > current:
            return candidate
        return current

    def observe(
        self,
        publication_sequence,
        source_timestamp_ns,
        received_timestamp_ns,
        observed_monotonic,
        observed_wall_ns,
        children,
    ):
        """Record one TFMessage's optional RMW metadata."""
        self.children.update(children)
        self.message_sizes[len(children)] += 1
        self.stream.observe(
            int(source_timestamp_ns or 0),
            observed_monotonic,
            observed_wall_ns,
        )

        try:
            sequence = int(publication_sequence)
        except (TypeError, ValueError):
            sequence = 0
        if sequence > 0:
            previous = self.last_publication_sequence
            if previous is not None:
                if sequence == previous:
                    self.sequence_duplicates += 1
                elif sequence < previous:
                    self.sequence_regressions += 1
                elif sequence > previous + 1:
                    self.missing_sequences += sequence - previous - 1
            self.last_publication_sequence = sequence

        try:
            received_ns = int(received_timestamp_ns)
        except (TypeError, ValueError):
            received_ns = 0
        try:
            source_ns = int(source_timestamp_ns)
        except (TypeError, ValueError):
            source_ns = 0
        if received_ns > 0:
            callback_delay = max(0, observed_wall_ns - received_ns)
            self.max_callback_queue_delay_ns = self._max(
                self.max_callback_queue_delay_ns, callback_delay
            )
        if received_ns > 0 and source_ns > 0:
            middleware_delay = max(0, received_ns - source_ns)
            self.max_middleware_delay_ns = self._max(
                self.max_middleware_delay_ns, middleware_delay
            )

    def snapshot(self, now_monotonic, reset=True):
        result = self.stream.snapshot(now_monotonic, reset=reset)
        result.update({
            'missing_sequences': self.missing_sequences,
            'sequence_duplicates': self.sequence_duplicates,
            'sequence_regressions': self.sequence_regressions,
            'max_callback_queue_delay_s': (
                None
                if self.max_callback_queue_delay_ns is None
                else self.max_callback_queue_delay_ns / 1e9
            ),
            'max_middleware_delay_s': (
                None
                if self.max_middleware_delay_ns is None
                else self.max_middleware_delay_ns / 1e9
            ),
            'children': tuple(sorted(self.children)),
            'message_sizes': dict(self.message_sizes),
        })
        if reset:
            self._reset_window()
        return result
