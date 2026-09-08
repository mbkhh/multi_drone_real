from importlib import import_module

import pytest


def test_plotter_loads_xyz_from_configured_waypoint_file(monkeypatch):
    plotter_module = import_module('swarm_station.position_plotter')
    calls = []

    def fake_loader(file_name, leader_id):
        calls.append((file_name, leader_id))
        return [
            [0.0, 1.0, 2.0, 0.0],
            [3.0, 4.0, 2.0, -22.5],
        ]

    monkeypatch.setattr(
        plotter_module, 'get_mission_waypoints', fake_loader
    )

    assert plotter_module._load_planned_xyz('mission.txt', 1) == [
        [0.0, 1.0, 2.0],
        [3.0, 4.0, 2.0],
    ]
    assert calls == [('mission.txt', 1)]


@pytest.mark.parametrize(
    'bad_point',
    (
        [0.0, 1.0, 2.0],
        [0.0, 1.0, 2.0, 'bad'],
        [0.0, 1.0, 2.0, float('nan')],
    ),
)
def test_plotter_rejects_invalid_mission_rows(monkeypatch, bad_point):
    plotter_module = import_module('swarm_station.position_plotter')
    monkeypatch.setattr(
        plotter_module,
        'get_mission_waypoints',
        lambda _file_name, _leader_id: [bad_point],
    )

    with pytest.raises(ValueError):
        plotter_module._load_planned_xyz('mission.txt', 1)
