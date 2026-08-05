import math

from swarm_station.takeoff_spacing_node import (
    EARTH_RADIUS_M,
    evaluate_spacing,
    horizontal_distance_m,
    relative_position_enu_m,
)


def longitude_offset_degrees(distance_m):
    return math.degrees(distance_m / EARTH_RADIUS_M)


def test_horizontal_distance_for_short_enu_scale_separation():
    first = {'lat': 0.0, 'lon': 0.0}
    second = {'lat': 0.0, 'lon': longitude_offset_degrees(4.0)}

    assert math.isclose(
        horizontal_distance_m(first, second), 4.0, rel_tol=1e-6
    )


def test_relative_position_reports_east_north_and_up():
    first = {'lat': 0.0, 'lon': 0.0, 'alt': 100.0}
    second = {
        'lat': math.degrees(3.0 / EARTH_RADIUS_M),
        'lon': longitude_offset_degrees(4.0),
        'alt': 101.5,
    }

    east, north, up = relative_position_enu_m(first, second)

    assert math.isclose(east, 4.0, rel_tol=1e-6)
    assert math.isclose(north, 3.0, rel_tol=1e-6)
    assert math.isclose(up, 1.5, rel_tol=1e-6)


def test_configured_four_meter_spacing_is_okay_within_tolerance():
    positions = {
        1: {'lat': 0.0, 'lon': 0.0},
        2: {'lat': 0.0, 'lon': longitude_offset_degrees(4.25)},
    }
    origins = {1: [0.0, 0.0, 0.0], 2: [4.0, 0.0, 0.0]}

    okay, pairs = evaluate_spacing(positions, [1, 2], origins, 0.5)

    assert okay
    assert pairs[0]['okay']
    assert math.isclose(pairs[0]['expected_m'], 4.0)
    assert math.isclose(pairs[0]['relative_enu_m']['east'], 4.25)
    assert math.isclose(pairs[0]['relative_enu_m']['north'], 0.0)


def test_spacing_outside_tolerance_is_not_okay():
    positions = {
        1: {'lat': 0.0, 'lon': 0.0},
        2: {'lat': 0.0, 'lon': longitude_offset_degrees(3.0)},
    }
    origins = {1: [0.0, 0.0, 0.0], 2: [4.0, 0.0, 0.0]}

    okay, pairs = evaluate_spacing(positions, [1, 2], origins, 0.5)

    assert not okay
    assert not pairs[0]['okay']
