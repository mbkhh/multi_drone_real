import math
from pathlib import Path

import pytest

from swarm_config import config_utils


def install_test_file(monkeypatch, tmp_path, contents):
    config_directory = tmp_path / 'config'
    config_directory.mkdir()
    waypoint_file = config_directory / 'mission.txt'
    waypoint_file.write_text(contents, encoding='utf-8')
    monkeypatch.setattr(
        config_utils,
        'get_package_share_directory',
        lambda _package_name: str(tmp_path),
    )
    return waypoint_file


def test_loads_only_the_requested_leader_waypoints(monkeypatch, tmp_path):
    install_test_file(
        monkeypatch,
        tmp_path,
        '1:\n  - [0.0, 0.0, 2.0, -22.5]\n'
        '2:\n  - [0.0, 5.0, 2.0, 0.0]\n',
    )

    points = config_utils.get_mission_waypoints('mission.txt', 1)

    assert points == [[0.0, 0.0, 2.0, -22.5]]


def test_rejects_file_without_elected_leader(monkeypatch, tmp_path):
    install_test_file(
        monkeypatch,
        tmp_path,
        '1:\n  - [0.0, 0.0, 2.0, 0.0]\n',
    )

    with pytest.raises(ValueError, match='leader 2'):
        config_utils.get_mission_waypoints('mission.txt', 2)


def test_rejects_waypoint_path_outside_installed_config(monkeypatch, tmp_path):
    monkeypatch.setattr(
        config_utils,
        'get_package_share_directory',
        lambda _package_name: str(Path(tmp_path)),
    )

    with pytest.raises(ValueError, match='must not contain a directory'):
        config_utils.get_mission_waypoints('../mission.txt', 1)


def test_configured_three_drone_mission_fits_control_limits():
    filename = config_utils.get_config('swarm_single.mission.waypoint_file')
    points = config_utils.get_mission_waypoints(filename, 1)
    max_waypoints = int(
        config_utils.get_config('swarm_single.mission.max_waypoints')
    )
    max_leg = float(
        config_utils.get_config('swarm_single.control.max_goal_distance')
    )
    min_altitude = float(
        config_utils.get_config('swarm_single.control.min_goal_altitude')
    )
    max_altitude = float(
        config_utils.get_config('swarm_single.control.max_goal_altitude')
    )
    initial = config_utils.get_config(
        'swarm_single.real_world.initial_positions.1'
    )

    assert 0 < len(points) <= max_waypoints
    assert all(len(point) == 4 for point in points)
    assert all(
        all(math.isfinite(float(value)) for value in point)
        for point in points
    )
    assert all(
        min_altitude <= float(point[2]) <= max_altitude
        for point in points
    )

    positions = [initial, *points]
    assert max(
        math.dist(first[:2], second[:2])
        for first, second in zip(positions, positions[1:])
    ) <= max_leg
