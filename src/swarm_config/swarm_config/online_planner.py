"""Small, deterministic waypoint planner shared by the station and drones.

The normal ``mission`` command intentionally keeps its file-backed behaviour.
This module is for the separate ``online_mission`` command: a compact command
containing a start/goal and a spacing is expanded on the elected leader.  The
expanded points therefore never have to be serialized and retransmitted over
the swarm Wi-Fi link.

Only the Python standard library is used here.  Keeping the planner in
``swarm_config`` makes it available to both the station and the controller
without introducing a station -> controller package dependency.
"""

from __future__ import annotations

import argparse
import json
import math
from typing import Iterable, List, Sequence


class OnlineWaypointPlanner:
    """Generate bounded straight-line waypoint sequences.

    ``start`` and ``goal`` are three-element ENU positions.  The returned
    sequence excludes ``start`` and always includes ``goal``.  A waypoint is
    emitted at most ``spacing`` metres from the previous waypoint.  The
    optional ``max_leg_distance`` is an independent safety ceiling; callers
    can pass the controller's goal limit so generated missions are accepted by
    the normal mission validator.
    """

    def __init__(
        self,
        *,
        max_waypoints: int = 100,
        max_leg_distance: float = 10.0,
        default_spacing: float | None = None,
    ) -> None:
        try:
            max_waypoints = int(max_waypoints)
            max_leg_distance = float(max_leg_distance)
        except (TypeError, ValueError) as error:
            raise ValueError(
                "max_waypoints and max_leg_distance must be numeric"
            ) from error

        if max_waypoints < 1:
            raise ValueError("max_waypoints must be at least one")
        if not math.isfinite(max_leg_distance) or max_leg_distance <= 0.0:
            raise ValueError("max_leg_distance must be finite and positive")

        if default_spacing is None:
            default_spacing = max_leg_distance
        try:
            default_spacing = float(default_spacing)
        except (TypeError, ValueError) as error:
            raise ValueError("default_spacing must be numeric") from error
        if (
            not math.isfinite(default_spacing)
            or default_spacing <= 0.0
            or default_spacing > max_leg_distance
        ):
            raise ValueError(
                "default_spacing must be finite, positive, and no greater "
                "than max_leg_distance"
            )

        self.max_waypoints = max_waypoints
        self.max_leg_distance = max_leg_distance
        self.default_spacing = default_spacing

    @staticmethod
    def _position(value: Sequence[float] | Iterable[float], name: str) -> List[float]:
        if isinstance(value, (str, bytes)):
            raise ValueError(f"{name} must contain exactly three numbers")
        try:
            position = [float(component) for component in value]
        except (TypeError, ValueError) as error:
            raise ValueError(f"{name} must contain exactly three numbers") from error
        if len(position) != 3 or not all(math.isfinite(component) for component in position):
            raise ValueError(f"{name} must contain exactly three finite numbers")
        return position

    def generate(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        spacing: float | None = None,
    ) -> List[List[float]]:
        """Return a bounded straight-line path from ``start`` to ``goal``.

        The method validates all planner inputs before producing a path.  A
        zero-length request still returns one waypoint (the goal), which lets
        the normal mission state machine perform its configured tolerance and
        dwell handling.
        """

        start_position = self._position(start, "start")
        goal_position = self._position(goal, "goal")
        if spacing is None:
            spacing = self.default_spacing
        try:
            spacing = float(spacing)
        except (TypeError, ValueError) as error:
            raise ValueError("spacing must be numeric") from error
        if not math.isfinite(spacing) or spacing <= 0.0:
            raise ValueError("spacing must be finite and positive")
        if spacing > self.max_leg_distance:
            raise ValueError(
                f"spacing {spacing:.3f} m exceeds the "
                f"{self.max_leg_distance:.3f} m goal limit"
            )

        distance = math.dist(start_position, goal_position)
        # ``ceil`` deliberately uses the full 3-D distance.  The controller's
        # safety gate is horizontal, so this can only produce shorter legs.
        segments = max(1, int(math.ceil(distance / spacing)))
        if segments > self.max_waypoints:
            raise ValueError(
                f"online path needs {segments} waypoints, exceeding the "
                f"{self.max_waypoints} waypoint limit"
            )

        path: List[List[float]] = []
        for index in range(1, segments + 1):
            fraction = index / segments
            path.append([
                start_position[axis]
                + (goal_position[axis] - start_position[axis]) * fraction
                for axis in range(3)
            ])
        return path

    # A descriptive alias is convenient for callers and keeps the public API
    # readable in tests and future planner implementations.
    generate_line = generate


def main(argv: Sequence[str] | None = None) -> None:
    """Command-line helper for checking planner inputs without ROS.

    This is intentionally a tiny utility, not a flight command.  It is useful
    on a development machine for inspecting the compact planner result.
    """

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("start", nargs=3, type=float, metavar=("X0", "Y0", "Z0"))
    parser.add_argument("goal", nargs=3, type=float, metavar=("X1", "Y1", "Z1"))
    parser.add_argument("--spacing", type=float, default=10.0)
    parser.add_argument("--max-waypoints", type=int, default=100)
    parser.add_argument("--max-leg-distance", type=float, default=10.0)
    args = parser.parse_args(argv)
    planner = OnlineWaypointPlanner(
        max_waypoints=args.max_waypoints,
        max_leg_distance=args.max_leg_distance,
        default_spacing=min(args.spacing, args.max_leg_distance),
    )
    print(json.dumps(planner.generate(args.start, args.goal, args.spacing)))


__all__ = ["OnlineWaypointPlanner"]

