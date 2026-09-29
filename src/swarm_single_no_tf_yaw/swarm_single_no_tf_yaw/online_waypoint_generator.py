"""Online coverage waypoint generation for the TF-free yaw controller.

The planner in this module is the non-plotting part of the coverage planner
script supplied with the project.  It deliberately has no ROS publishers or
timers: generation is a short, deterministic operation which is requested by
the leader and then handed to the existing mission controller.
"""

from dataclasses import dataclass
from math import ceil, floor, inf

import numpy as np


def wrap_pi(angle):
    """Wrap an angle in radians to ``[-pi, pi)``."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class Config:
    """Inputs accepted by the online coverage planner.

    The names and defaults mirror ``coverage_planner_numpy_v2 (1).py``.  The
    waypoint spacing is also used for straight legs so every generated leg is
    compatible with the controller's normal per-goal safety limit.
    """

    Lx: float = 34.0
    Ly: float = 36.0
    num_quads: int = 3
    d: float = 4.0
    min_turn_radius: float = 4.0
    coverage_spacing: float = 4.0
    max_path_length: float = inf
    ds_waypoint: float = 4.0
    altitude: float = 5.0
    turn_waypoints: int = 8


def build_offsets(num_quads, d):
    """Return the cross-track offsets, with index zero reserved for leader."""
    k = np.arange(num_quads, dtype=float) - (num_quads - 1) / 2.0
    return k * d


def line_segment(p1, p2):
    return {
        "type": "line",
        "p1": np.asarray(p1, dtype=float),
        "p2": np.asarray(p2, dtype=float),
    }


def arc_segment(center, radius, theta1, theta2, arcsign):
    return {
        "type": "arc",
        "center": np.asarray(center, dtype=float),
        "radius": float(radius),
        "theta1": float(theta1),
        "theta2": float(theta2),
        "arcsign": float(arcsign),
    }


def segment_length(segment):
    if segment["type"] == "line":
        return float(np.linalg.norm(segment["p2"] - segment["p1"]))
    return float(segment["radius"] * abs(segment["theta2"] - segment["theta1"]))


def path_length(segments):
    return float(sum(segment_length(segment) for segment in segments))


def build_spine(Lx, Ly, turn_radius, max_offset, n_passes):
    """Build the internal boustrophedon spine used by the supplied planner."""
    radius = turn_radius
    margin = radius + max_offset
    x_min = margin
    x_max = Lx - margin
    y_start = Ly - max_offset
    row_step = -2.0 * radius

    segments = []
    going_right = False
    for index in range(n_passes):
        y = y_start + index * row_step
        if going_right:
            p1 = np.array([x_min, y])
            p2 = np.array([x_max, y])
        else:
            p1 = np.array([x_max, y])
            p2 = np.array([x_min, y])

        if index == 0:
            p1[0] = Lx if not going_right else 0.0
        if index == n_passes - 1:
            p2[0] = 0.0 if not going_right else Lx
        segments.append(line_segment(p1, p2))

        if index < n_passes - 1:
            x_end = x_max if going_right else x_min
            center = np.array([x_end, y + row_step / 2.0])
            arcsign = -1.0 if going_right else 1.0
            segments.append(
                arc_segment(center, radius, np.pi / 2.0, -np.pi / 2.0, arcsign)
            )
        going_right = not going_right

    return segments


def offset_path(spine_segments, offset):
    """Build the parallel path for one vehicle in the formation."""
    result = []
    for segment in spine_segments:
        if segment["type"] == "line":
            delta = segment["p2"] - segment["p1"]
            length = np.linalg.norm(delta)
            tangent = delta / length
            heading = np.arctan2(tangent[1], tangent[0])
            normal = np.array([-np.sin(heading), np.cos(heading)])
            result.append(
                line_segment(
                    segment["p1"] + offset * normal,
                    segment["p2"] + offset * normal,
                )
            )
        else:
            radius = segment["radius"] - segment["arcsign"] * offset
            if radius <= 0.0:
                return None
            result.append(
                arc_segment(
                    segment["center"],
                    radius,
                    segment["theta1"],
                    segment["theta2"],
                    segment["arcsign"],
                )
            )
    return result


def state_at_s(segments, distance):
    """Return ``[x, y, heading]`` at an arc length along a path."""
    lengths = np.array([segment_length(segment) for segment in segments])
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    total = cumulative[-1]
    distance = np.clip(distance, 0.0, total)
    index = np.searchsorted(cumulative[1:], distance, side="right")
    index = min(index, len(segments) - 1)
    segment = segments[index]
    local_distance = distance - cumulative[index]

    if segment["type"] == "line":
        delta = segment["p2"] - segment["p1"]
        length = np.linalg.norm(delta)
        tangent = delta / length
        position = segment["p1"] + local_distance * tangent
        heading = np.arctan2(tangent[1], tangent[0])
        return np.array([position[0], position[1], heading])

    theta = (
        segment["theta1"]
        + segment["arcsign"] * local_distance / segment["radius"]
    )
    position = segment["center"] + segment["radius"] * np.array(
        [np.cos(theta), np.sin(theta)]
    )
    heading = theta + segment["arcsign"] * np.pi / 2.0
    return np.array([position[0], position[1], heading])


def sample_path(segments, ds):
    total = path_length(segments)
    distances = np.arange(0.0, total, ds)
    if len(distances) == 0 or distances[-1] < total - 1e-10:
        distances = np.append(distances, total)
    states = np.vstack([state_at_s(segments, distance) for distance in distances])
    return states, distances


def transform_to_new_frame(xy_old, Lx, Ly):
    xy_old = np.asarray(xy_old)
    result = np.empty_like(xy_old, dtype=float)
    result[:, 0] = Lx - xy_old[:, 0]
    result[:, 1] = Ly - xy_old[:, 1]
    return result


def sample_path_new_frame(segments, ds, Lx, Ly):
    states_old, distances = sample_path(segments, ds)
    xy_new = transform_to_new_frame(states_old[:, :2], Lx, Ly)
    heading_new = wrap_pi(states_old[:, 2] + np.pi)
    return np.column_stack((xy_new, heading_new, distances))


def point_to_new_frame(point_old, Lx, Ly):
    point = np.asarray(point_old, dtype=float).reshape(1, 2)
    return transform_to_new_frame(point, Lx, Ly)[0]


def path_to_flight_waypoints(segments, config):
    """Convert a geometric path to ``[x, y, z, relative_yaw_deg]`` rows.

    Turns retain the exact requested number of equal yaw increments.  Straight
    legs are split at ``ds_waypoint`` so the regular controller's
    ``max_goal_distance`` safety check is not bypassed by an online plan.
    """
    if not segments:
        return np.empty((0, 4), dtype=float)

    yaw_step = 180.0 / int(config.turn_waypoints)
    rows = []

    first = segments[0]
    if first["type"] == "line":
        start_old = first["p1"]
    else:
        start_old = first["center"] + first["radius"] * np.array(
            [np.cos(first["theta1"]), np.sin(first["theta1"])]
        )
    start_new = point_to_new_frame(start_old, config.Lx, config.Ly)
    rows.append([start_new[0], start_new[1], config.altitude, 0.0, False])

    for segment in segments:
        if segment["type"] == "line":
            end_new = point_to_new_frame(segment["p2"], config.Lx, config.Ly)
            previous = np.asarray(rows[-1][:2], dtype=float)
            distance = float(np.linalg.norm(end_new - previous))
            steps = max(1, int(ceil(distance / config.ds_waypoint)))
            for step in range(1, steps + 1):
                point = previous + (end_new - previous) * (step / steps)
                if np.linalg.norm(point - np.asarray(rows[-1][:2])) > 1e-10:
                    rows.append([point[0], point[1], config.altitude, 0.0,False])
            continue

        yaw_relative = -segment["arcsign"] * yaw_step
        for step in range(1, int(config.turn_waypoints) + 1):
            checkpoint=False
            if(step==1 or step==config.turn_waypoints/2 or step==config.turn_waypoints):
                checkpoint=True
            theta = (
                segment["theta1"]
                + segment["arcsign"] * (step / config.turn_waypoints) * np.pi
            )
            point_old = segment["center"] + segment["radius"] * np.array(
                [np.cos(theta), np.sin(theta)]
            )
            point_new = point_to_new_frame(point_old, config.Lx, config.Ly)
            rows.append(
                [point_new[0], point_new[1], config.altitude, yaw_relative,checkpoint]
            )

    return np.asarray(rows, dtype=float)


def build_waypoint_dictionary(quad_paths, config):
    return {
        index + 1: path_to_flight_waypoints(path, config).tolist()
        for index, path in enumerate(quad_paths)
    }


def max_coverage_gap(row_spacing, d, max_offset, num_quads):
    internal_gap = 0.0 if num_quads == 1 else d
    inter_pass_gap = max(0.0, row_spacing - 2.0 * max_offset)
    return max(internal_gap, inter_pass_gap)


def validate_config(config):
    if config.Lx <= 0 or config.Ly <= 0:
        raise ValueError("Lx and Ly must be positive.")
    if config.num_quads < 1 or int(config.num_quads) != config.num_quads:
        raise ValueError("num_quads must be a positive integer.")
    if config.d <= 0:
        raise ValueError("d must be positive.")
    if config.min_turn_radius <= 0:
        raise ValueError("min_turn_radius must be positive.")
    if config.coverage_spacing <= 0:
        raise ValueError("coverage_spacing must be positive.")
    if config.ds_waypoint <= 0:
        raise ValueError("ds_waypoint must be positive.")
    if config.altitude <= 0:
        raise ValueError("altitude must be positive.")
    if config.turn_waypoints < 1 or int(config.turn_waypoints) != config.turn_waypoints:
        raise ValueError("turn_waypoints must be a positive integer.")
    values = (
        config.Lx,
        config.Ly,
        config.d,
        config.min_turn_radius,
        config.coverage_spacing,
        config.ds_waypoint,
        config.altitude,
        config.max_path_length,
    )
    if not all(np.isfinite(value) for value in values[:-1]):
        raise ValueError("Planner dimensions and spacing values must be finite.")
    if not np.isfinite(config.max_path_length) and config.max_path_length != inf:
        raise ValueError("max_path_length must be finite or infinity.")


def plan(config):
    """Generate and validate all formation paths for ``config``."""
    validate_config(config)
    offsets = build_offsets(config.num_quads, config.d)
    leader_offset = offsets[0]
    max_offset = np.max(np.abs(offsets))

    if config.num_quads > 1 and config.d > config.coverage_spacing + 1e-9:
        raise ValueError(
            "coverage_spacing is smaller than d; the formation itself has coverage gaps."
        )

    usable_y_span = config.Ly - 2.0 * max_offset
    if usable_y_span <= 0:
        raise ValueError("Formation does not fit inside the environment height.")

    required_radius = max(config.min_turn_radius, max_offset)
    max_row_spacing = 2.0 * max_offset + config.coverage_spacing
    n_pass_min = max(
        2,
        int(ceil(usable_y_span / max_row_spacing - 1e-12)) + 1,
    )
    n_pass_max = int(
        floor(usable_y_span / (2.0 * required_radius) + 1e-12)
    ) + 1
    if n_pass_max < n_pass_min:
        raise ValueError(
            f"No feasible solution: coverage requires >= {n_pass_min} passes, "
            f"but turn geometry allows <= {n_pass_max}."
        )

    candidates = []
    for n_passes in range(n_pass_min, n_pass_max + 1):
        row_spacing = usable_y_span / (n_passes - 1)
        radius = row_spacing / 2.0
        if radius < required_radius - 1e-9:
            continue
        margin = radius + max_offset
        if config.Lx <= 2.0 * margin + 1e-9:
            continue

        spine = build_spine(config.Lx, config.Ly, radius, max_offset, n_passes)
        leader_path = offset_path(spine, leader_offset)
        if leader_path is None:
            continue
        leader_length = path_length(leader_path)
        spine_length = path_length(spine)
        gap = max_coverage_gap(
            row_spacing, config.d, max_offset, config.num_quads
        )
        if gap > config.coverage_spacing + 1e-9:
            continue
        if leader_length > config.max_path_length + 1e-9:
            continue
        candidates.append(
            {
                "n_passes": n_passes,
                "row_spacing": row_spacing,
                "turn_radius": radius,
                "leader_length": leader_length,
                "spine_length": spine_length,
                "coverage_gap": gap,
                "spine": spine,
                "leader_path": leader_path,
            }
        )

    if not candidates:
        raise ValueError("No feasible candidate satisfies all constraints.")

    best = min(candidates, key=lambda candidate: candidate["leader_length"])
    leader_waypoints = sample_path_new_frame(
        best["leader_path"], config.ds_waypoint, config.Lx, config.Ly
    )
    if np.linalg.norm(leader_waypoints[0, :2]) > 1e-8:
        raise RuntimeError("Frame error: leader does not start at (0,0).")

    quad_paths = [
        offset_path(best["spine"], offset) for offset in offsets
    ]
    waypoints = build_waypoint_dictionary(quad_paths, config)
    for index in range(config.num_quads):
        first = np.asarray(waypoints[index + 1][0], dtype=float)
        expected = np.array([0.0, index * config.d])
        if np.linalg.norm(first[:2] - expected) > 1e-8:
            raise RuntimeError(
                f"Frame/formation error for quad {index + 1}: "
                f"start={first[:2]}, expected={expected}"
            )

    return {
        "config": config,
        "offsets": offsets,
        "leader_offset": leader_offset,
        "best": best,
        "candidates": candidates,
        "leader_waypoints": leader_waypoints,
        "quad_paths": quad_paths,
        "waypoints": waypoints,
    }


class OnlineWaypointGenerator:
    """Generate a coverage plan on demand for one controller node.

    ``generate`` is intentionally side-effect free.  ``start_mission`` is the
    command-facing helper: it generates all paths and starts the existing
    leader mission using only the generated leader path.  Followers continue
    to use the existing shared-leader formation behavior.
    """

    PARAMETER_NAMES = {
        "Lx",
        "Ly",
        "num_quads",
        "d",
        "min_turn_radius",
        "coverage_spacing",
        "max_path_length",
        "ds_waypoint",
        "altitude",
        "turn_waypoints",
    }

    def __init__(self, parent_node):
        self.parent_node = parent_node
        self.last_config = None
        self.last_result = None

    @staticmethod
    def _float(value, name):
        if isinstance(value, str) and value.strip().lower() in {
            "inf",
            "+inf",
            "infinity",
            "+infinity",
        }:
            return inf
        try:
            return float(value)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(f"{name} must be numeric.") from error

    @classmethod
    def _config_from_parameters(cls, parameters):
        if parameters is None:
            parameters = {}
        if not isinstance(parameters, dict):
            raise ValueError("online_mission parameters must be an object.")

        aliases = {name.lower(): name for name in cls.PARAMETER_NAMES}
        values = {}
        for key, value in parameters.items():
            canonical = aliases.get(str(key).lower())
            if canonical is None:
                # These are command options, not planner inputs.
                if str(key).lower() in {"relative_to_start", "yaw_relative"}:
                    continue
                raise ValueError(f"unknown online planner input '{key}'.")
            values[canonical] = value

        defaults = Config()
        float_names = {
            "Lx",
            "Ly",
            "d",
            "min_turn_radius",
            "coverage_spacing",
            "max_path_length",
            "ds_waypoint",
            "altitude",
        }
        for name in float_names:
            if name in values:
                values[name] = cls._float(values[name], name)
        for name in ("num_quads", "turn_waypoints"):
            if name in values:
                numeric = cls._float(values[name], name)
                if not np.isfinite(numeric) or int(numeric) != numeric:
                    raise ValueError(f"{name} must be an integer.")
                values[name] = int(numeric)

        return Config(
            **{
                field: values.get(field, getattr(defaults, field))
                for field in Config.__dataclass_fields__
            }
        )

    def generate(self, parameters=None, **overrides):
        """Generate all quad paths from a mapping or keyword inputs."""
        merged = {}
        if parameters is not None:
            if not isinstance(parameters, dict):
                raise ValueError("online_mission parameters must be an object.")
            merged.update(parameters)
        merged.update(overrides)
        config = self._config_from_parameters(merged)
        result = plan(config)
        self.last_config = config
        self.last_result = result
        return result

    def start_mission(self, parameters=None, **overrides):
        """Generate a plan and start its leader waypoints.

        Returns ``(accepted, result)``.  ``result`` is returned even when the
        controller rejects the mission so callers can report useful counts.
        """
        merged = {}
        if parameters is not None:
            if not isinstance(parameters, dict):
                raise ValueError("online_mission parameters must be an object.")
            merged.update(parameters)
        merged.update(overrides)
        relative_to_start = bool(merged.pop("relative_to_start", False))
        yaw_relative = bool(merged.pop("yaw_relative", True))
        result = self.generate(merged)
        leader_waypoints = result["waypoints"].get(1)
        if not leader_waypoints:
            raise ValueError("online planner generated no leader waypoints.")
        accepted = self.parent_node.start_mission(
            leader_waypoints,
            relative_to_start=relative_to_start,
            yaw_relative=yaw_relative,
        )
        return bool(accepted), result


# Keep the package's existing lower-case class naming style available to
# callers while exposing the conventional class name above.
online_waypoint_generator = OnlineWaypointGenerator

