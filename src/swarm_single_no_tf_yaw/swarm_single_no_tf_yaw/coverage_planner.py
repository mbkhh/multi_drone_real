"""Online, formation-aware coverage path generation.

The planner in this module is deliberately independent of ROS.  The leader
controller can therefore generate a path after it receives a compact command,
without sending a large waypoint array over the swarm network.  It is the
runtime version of the boustrophedon/rounded-turn geometry that was previously
exported manually into the YAML mission file.

The returned waypoint format is the format accepted by ``start_mission``:
``[absolute ENU x, absolute ENU y, altitude, relative PX4/NED yaw degrees]``.
Positive relative yaw follows the controller's existing clockwise-positive
mission convention.
"""

from dataclasses import dataclass
from math import (
    atan2,
    ceil,
    cos,
    degrees,
    floor,
    hypot,
    inf,
    isfinite,
    pi,
    sin,
    sqrt,
)
from typing import Dict, List, Sequence, Tuple


_EPSILON = 1.0e-9
Point2 = Tuple[float, float]
State = Tuple[float, float, float]


@dataclass(frozen=True)
class CoverageConfig:
    """Geometry and output limits for one online coverage plan.

    ``Lx``/``Ly`` describe the rectangle in metres.  The generated leader
    starts at ``(origin_x, origin_y)`` and travels along the rectangle's
    positive local X direction; followers start at positive local Y offsets.
    ``d`` is the along-formation spacing, while ``coverage_spacing`` is the
    maximum allowed uncovered cross-track gap between adjacent passes.
    """

    Lx: float = 34.0
    Ly: float = 36.0
    num_quads: int = 3
    d: float = 4.0
    min_turn_radius: float = 4.0
    coverage_spacing: float = 4.0
    ds_waypoint: float = 1.0
    max_path_length: float = inf
    altitude: float = 3.0
    turn_waypoints: int = 8
    max_waypoints: int = 200
    max_leg_length: float = 10.0
    origin_x: float = 0.0
    origin_y: float = 0.0


@dataclass(frozen=True)
class CoveragePlan:
    """Generated mission and useful diagnostics for the station/logger."""

    leader_waypoints: List[List[float]]
    all_waypoints: Dict[int, List[List[float]]]
    offsets: List[float]
    n_passes: int
    row_spacing: float
    turn_radius: float
    coverage_gap: float
    leader_path_length: float
    spine_path_length: float


@dataclass(frozen=True)
class _Line:
    p1: Point2
    p2: Point2


@dataclass(frozen=True)
class _Arc:
    center: Point2
    radius: float
    theta1: float
    arcsign: float


Segment = Tuple[str, object]


class CoveragePlanner:
    """Generate an optimized rounded boustrophedon path for a swarm."""

    def __init__(self, config: CoverageConfig):
        self.config = config

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        return (angle + pi) % (2.0 * pi) - pi

    @staticmethod
    def _line(p1: Point2, p2: Point2) -> Segment:
        return ("line", _Line(tuple(p1), tuple(p2)))

    @staticmethod
    def _arc(center: Point2, radius: float, arcsign: float) -> Segment:
        return ("arc", _Arc(tuple(center), float(radius), pi / 2.0, arcsign))

    @staticmethod
    def _segment_length(segment: Segment) -> float:
        kind, value = segment
        if kind == "line":
            line = value
            return hypot(line.p2[0] - line.p1[0], line.p2[1] - line.p1[1])
        arc = value
        return pi * arc.radius

    @classmethod
    def _path_length(cls, segments: Sequence[Segment]) -> float:
        return sum(cls._segment_length(segment) for segment in segments)

    @staticmethod
    def _build_offsets(num_quads: int, spacing: float) -> List[float]:
        midpoint = (num_quads - 1) / 2.0
        # The first vehicle is the leader.  With three vehicles this is
        # [-d, 0, +d], which becomes [0, +d, +2d] in the preserved output
        # frame after the leader's -d normal offset is applied.
        return [(index - midpoint) * spacing for index in range(num_quads)]

    @classmethod
    def _build_spine(
        cls,
        length_x: float,
        length_y: float,
        turn_radius: float,
        max_offset: float,
        n_passes: int,
    ) -> List[Segment]:
        margin = turn_radius + max_offset
        x_min = margin
        x_max = length_x - margin
        y_start = length_y - max_offset
        row_step = -2.0 * turn_radius

        segments: List[Segment] = []
        going_right = False
        for index in range(n_passes):
            y = y_start + index * row_step
            if going_right:
                p1 = (x_min, y)
                p2 = (x_max, y)
            else:
                p1 = (x_max, y)
                p2 = (x_min, y)

            # Preserve the original planner's exact boundary convention.
            if index == 0:
                p1 = (length_x if not going_right else 0.0, y)
            if index == n_passes - 1:
                p2 = (0.0 if not going_right else length_x, y)
            segments.append(cls._line(p1, p2))

            if index < n_passes - 1:
                x_end = x_max if going_right else x_min
                center = (x_end, y + row_step / 2.0)
                arcsign = -1.0 if going_right else 1.0
                segments.append(cls._arc(center, turn_radius, arcsign))
            going_right = not going_right
        return segments

    @classmethod
    def _offset_path(
        cls,
        spine_segments: Sequence[Segment],
        offset: float,
    ) -> List[Segment] | None:
        result: List[Segment] = []
        for kind, value in spine_segments:
            if kind == "line":
                line = value
                dx = line.p2[0] - line.p1[0]
                dy = line.p2[1] - line.p1[1]
                length = hypot(dx, dy)
                if length <= _EPSILON:
                    return None
                heading = atan2(dy, dx)
                normal = (-sin(heading), cos(heading))
                result.append(
                    cls._line(
                        (
                            line.p1[0] + offset * normal[0],
                            line.p1[1] + offset * normal[1],
                        ),
                        (
                            line.p2[0] + offset * normal[0],
                            line.p2[1] + offset * normal[1],
                        ),
                    )
                )
            else:
                arc = value
                radius = arc.radius - arc.arcsign * offset
                if radius <= _EPSILON:
                    return None
                result.append(cls._arc(arc.center, radius, arc.arcsign))
        return result

    @classmethod
    def _state_at_segment_distance(
        cls,
        segment: Segment,
        distance: float,
    ) -> State:
        kind, value = segment
        if kind == "line":
            line = value
            dx = line.p2[0] - line.p1[0]
            dy = line.p2[1] - line.p1[1]
            length = hypot(dx, dy)
            if length <= _EPSILON:
                return (line.p1[0], line.p1[1], 0.0)
            ratio = max(0.0, min(1.0, distance / length))
            return (
                line.p1[0] + ratio * dx,
                line.p1[1] + ratio * dy,
                atan2(dy, dx),
            )

        arc = value
        theta = arc.theta1 + arc.arcsign * distance / arc.radius
        return (
            arc.center[0] + arc.radius * cos(theta),
            arc.center[1] + arc.radius * sin(theta),
            theta + arc.arcsign * pi / 2.0,
        )

    @classmethod
    def _sample_path_new_frame(
        cls,
        segments: Sequence[Segment],
        config: CoverageConfig,
    ) -> List[State]:
        """Sample every leg densely enough for the controller's goal limit."""
        states: List[State] = []
        for kind, value in segments:
            segment_length = cls._segment_length((kind, value))
            if segment_length <= _EPSILON:
                continue
            step = config.ds_waypoint
            if kind == "arc":
                # Respect the requested number of turn samples even when a
                # caller chooses a comparatively large straight-leg spacing.
                step = min(step, segment_length / config.turn_waypoints)
            count = max(1, int(ceil(segment_length / step - _EPSILON)))
            for sample_index in range(count + 1):
                if states and sample_index == 0:
                    continue
                local_distance = min(
                    segment_length,
                    sample_index * segment_length / count,
                )
                old_x, old_y, old_heading = cls._state_at_segment_distance(
                    (kind, value), local_distance
                )
                # Rotate the internal MATLAB frame by 180 degrees.  The
                # resulting frame starts at the leader's origin and has +Y
                # along the initial formation line.
                new_x = config.origin_x + config.Lx - old_x
                new_y = config.origin_y + config.Ly - old_y
                new_heading = cls._wrap_pi(old_heading + pi)
                state = (new_x, new_y, new_heading)
                if states:
                    previous = states[-1]
                    if hypot(state[0] - previous[0], state[1] - previous[1]) <= _EPSILON:
                        # Keep the most recent heading at a shared segment
                        # boundary, but do not create a duplicate mission goal.
                        states[-1] = state
                        continue
                states.append(state)
        return states

    @classmethod
    def _states_to_waypoints(
        cls,
        states: Sequence[State],
        altitude: float,
    ) -> List[List[float]]:
        waypoints: List[List[float]] = []
        previous_heading = None
        for x, y, heading in states:
            if previous_heading is None:
                yaw_delta = 0.0
            else:
                # Internal geometry uses mathematical CCW-positive angles;
                # PX4/NED mission yaw in this project is clockwise-positive.
                yaw_delta = -degrees(cls._wrap_pi(heading - previous_heading))
                if abs(yaw_delta) <= 1.0e-10:
                    yaw_delta = 0.0
            waypoints.append([float(x), float(y), float(altitude), float(yaw_delta)])
            previous_heading = heading
        return waypoints

    @classmethod
    def _validate(cls, config: CoverageConfig) -> None:
        positive = (
            ("Lx", config.Lx),
            ("Ly", config.Ly),
            ("d", config.d),
            ("min_turn_radius", config.min_turn_radius),
            ("coverage_spacing", config.coverage_spacing),
            ("ds_waypoint", config.ds_waypoint),
            ("altitude", config.altitude),
            ("max_leg_length", config.max_leg_length),
        )
        for name, value in positive:
            if not isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero.")
        if config.max_path_length != inf and (
            not isfinite(float(config.max_path_length))
            or float(config.max_path_length) <= 0.0
        ):
            raise ValueError("max_path_length must be positive or infinity.")
        for name, value in (
            ("origin_x", config.origin_x),
            ("origin_y", config.origin_y),
        ):
            if not isfinite(float(value)):
                raise ValueError(f"{name} must be finite.")
        if int(config.num_quads) != config.num_quads or config.num_quads < 1:
            raise ValueError("num_quads must be a positive integer.")
        if int(config.turn_waypoints) != config.turn_waypoints or config.turn_waypoints < 1:
            raise ValueError("turn_waypoints must be a positive integer.")
        if int(config.max_waypoints) != config.max_waypoints or config.max_waypoints < 1:
            raise ValueError("max_waypoints must be a positive integer.")
        if config.ds_waypoint > config.max_leg_length + _EPSILON:
            raise ValueError(
                "ds_waypoint cannot exceed max_leg_length; otherwise the "
                "controller may reject a generated leg."
            )

    @classmethod
    def _coverage_gap(
        cls,
        row_spacing: float,
        d: float,
        max_offset: float,
        num_quads: int,
    ) -> float:
        internal_gap = 0.0 if num_quads == 1 else d
        inter_pass_gap = max(0.0, row_spacing - 2.0 * max_offset)
        return max(internal_gap, inter_pass_gap)

    def plan(self) -> CoveragePlan:
        """Generate and validate one complete leader coverage mission."""
        config = self.config
        self._validate(config)
        offsets = self._build_offsets(config.num_quads, config.d)
        max_offset = max(abs(value) for value in offsets)

        if config.num_quads > 1 and config.d > config.coverage_spacing + _EPSILON:
            raise ValueError(
                "coverage_spacing is smaller than formation spacing d; "
                "the formation itself would leave gaps."
            )

        usable_y_span = config.Ly - 2.0 * max_offset
        if usable_y_span <= _EPSILON:
            raise ValueError("The formation does not fit inside the environment height.")

        required_radius = max(config.min_turn_radius, max_offset)
        max_row_spacing = 2.0 * max_offset + config.coverage_spacing
        n_pass_min = max(
            2,
            int(ceil(usable_y_span / max_row_spacing - _EPSILON)) + 1,
        )
        n_pass_max = int(
            floor(usable_y_span / (2.0 * required_radius) + _EPSILON)
        ) + 1
        if n_pass_max < n_pass_min:
            raise ValueError(
                f"No feasible coverage path: coverage needs at least {n_pass_min} "
                f"passes, but the turn radius allows at most {n_pass_max}."
            )

        candidates = []
        for n_passes in range(n_pass_min, n_pass_max + 1):
            row_spacing = usable_y_span / (n_passes - 1)
            turn_radius = row_spacing / 2.0
            if turn_radius < required_radius - _EPSILON:
                continue
            margin = turn_radius + max_offset
            if config.Lx <= 2.0 * margin + _EPSILON:
                continue
            spine = self._build_spine(
                config.Lx,
                config.Ly,
                turn_radius,
                max_offset,
                n_passes,
            )
            leader_path = self._offset_path(spine, offsets[0])
            if leader_path is None:
                continue
            leader_length = self._path_length(leader_path)
            spine_length = self._path_length(spine)
            gap = self._coverage_gap(
                row_spacing,
                config.d,
                max_offset,
                config.num_quads,
            )
            if gap > config.coverage_spacing + _EPSILON:
                continue
            if leader_length > config.max_path_length + _EPSILON:
                continue
            candidates.append(
                (
                    leader_length,
                    n_passes,
                    row_spacing,
                    turn_radius,
                    gap,
                    spine_length,
                    spine,
                    leader_path,
                )
            )

        if not candidates:
            raise ValueError("No feasible coverage candidate satisfies the configured limits.")

        (
            leader_length,
            n_passes,
            row_spacing,
            turn_radius,
            gap,
            spine_length,
            _spine,
            _leader_path,
        ) = min(candidates, key=lambda item: item[0])

        # Reconstruct the selected candidate by its identifying values.  This
        # avoids retaining a mutable planner state between calls.
        selected = next(
            item
            for item in candidates
            if item[0] == leader_length and item[1] == n_passes
        )
        spine = selected[6]
        leader_path = selected[7]
        leader_states = self._sample_path_new_frame(leader_path, config)
        leader_waypoints = self._states_to_waypoints(leader_states, config.altitude)
        if not leader_waypoints:
            raise ValueError("Coverage planner produced no leader waypoints.")
        if len(leader_waypoints) > config.max_waypoints:
            raise ValueError(
                f"Coverage path requires {len(leader_waypoints)} waypoints, "
                f"exceeding the configured limit of {config.max_waypoints}. "
                "Increase ds_waypoint or raise mission.max_waypoints."
            )

        all_waypoints: Dict[int, List[List[float]]] = {}
        for drone_index, offset in enumerate(offsets, start=1):
            path = self._offset_path(spine, offset)
            if path is None:
                raise ValueError(f"Unable to offset the path for drone {drone_index}.")
            states = self._sample_path_new_frame(path, config)
            waypoints = self._states_to_waypoints(states, config.altitude)
            if len(waypoints) > config.max_waypoints:
                raise ValueError(
                    f"Coverage path for drone {drone_index} requires "
                    f"{len(waypoints)} waypoints, exceeding the configured "
                    f"limit of {config.max_waypoints}."
                )
            all_waypoints[drone_index] = waypoints

        # The first row is a useful invariant: the leader starts at the
        # requested origin and followers are spaced along +Y.
        first = leader_waypoints[0]
        if hypot(first[0] - config.origin_x, first[1] - config.origin_y) > 1.0e-7:
            raise RuntimeError("Coverage frame error: leader does not start at origin.")
        for drone_index, waypoints in all_waypoints.items():
            expected_y = config.origin_y + (drone_index - 1) * config.d
            if hypot(
                waypoints[0][0] - config.origin_x,
                waypoints[0][1] - expected_y,
            ) > 1.0e-7:
                raise RuntimeError(
                    f"Coverage formation error for drone {drone_index}: "
                    f"start={waypoints[0][:2]} expected="
                    f"[{config.origin_x}, {expected_y}]"
                )

        return CoveragePlan(
            leader_waypoints=leader_waypoints,
            all_waypoints=all_waypoints,
            offsets=list(offsets),
            n_passes=n_passes,
            row_spacing=row_spacing,
            turn_radius=turn_radius,
            coverage_gap=gap,
            leader_path_length=leader_length,
            spine_path_length=spine_length,
        )


__all__ = ["CoverageConfig", "CoveragePlan", "CoveragePlanner"]
