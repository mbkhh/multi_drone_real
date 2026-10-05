"""Regression tests for one-shot online mission generation."""

from swarm_single_no_tf_yaw.online_waypoint_generator import (
    OnlineWaypointGenerator,
)


class DummyController:
    """Record normal mission starts without creating a ROS node."""

    max_mission_waypoints = 20
    min_goal_altitude = 0.0
    max_goal_altitude = 10.0

    def __init__(self):
        self.start_calls = []

    def start_mission(self, points, relative_to_start, yaw_relative):
        """Record the one handoff into the normal mission state machine."""
        self.start_calls.append((points, relative_to_start, yaw_relative))
        return True


def test_online_plan_is_generated_once_and_only_summary_is_returned(
    monkeypatch,
):
    """Planner runs once, then hands leader points to normal mission code."""
    controller = DummyController()
    generator = OnlineWaypointGenerator(controller)
    planner_calls = []
    generated = {
        "waypoints": {
            1: [[1.0, 2.0, 4.0, 0.0, False]],
            2: [[1.0, 7.0, 4.0, 0.0, False]],
            3: [[1.0, 12.0, 4.0, 0.0, False]],
        },
        "planner_scratch": object(),
    }

    def fake_generate(parameters):
        planner_calls.append(parameters)
        return generated

    monkeypatch.setattr(generator, "generate", fake_generate)

    accepted, summary = generator.start_mission({"altitude": 4.0})

    assert accepted
    assert planner_calls == [{"altitude": 4.0}]
    assert controller.start_calls == [
        (
            generated["waypoints"][1],
            False,
            True,
        ),
    ]
    assert summary["leader_waypoint_count"] == 1
    assert summary["path_count"] == 3
    assert summary["generation_seconds"] >= 0.0
    assert "waypoints" not in summary
    assert not hasattr(generator, "last_result")
