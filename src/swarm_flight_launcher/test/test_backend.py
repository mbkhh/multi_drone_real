import json
import os

from swarm_flight_launcher.backend import (
    build_component_specs,
    build_remote_script,
    build_station_script,
    load_settings,
    PID_MARKER,
    safe_settings_payload,
    save_settings,
)


def test_component_commands_match_real_three_drone_stack():
    for drone_id in (1, 2, 3):
        specs = build_component_specs(drone_id)

        assert specs['agent'].command == (
            'MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600'
        )
        assert (
            'ros2 run swarm_single_no_tf_yaw control_node'
            in specs['control'].command
        )
        assert f'frame_id:={drone_id}' in specs['control'].command
        assert (
            f'__node:=control_node_no_tf_yaw_d{drone_id}'
            in specs['control'].command
        )
        assert './run_swarm_vision.sh flight' in specs['vision'].command
        assert f'uav_id:=UAV_{drone_id}' in specs['vision'].command
        assert 'camera_index:=0' in specs['vision'].command
        assert 'input_size:=320' in specs['vision'].command
        assert 'inference_threads:=1' in specs['vision'].command


def test_remote_script_sources_workspace_and_forces_real_flight_dds():
    script = build_remote_script(
        '~/multi_drone_real',
        'MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600',
    )

    assert 'setsid --wait bash --noprofile --norc -lc' in script
    assert 'unset ROS_DOMAIN_ID' in script
    assert 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp' in script
    assert '/opt/ros/rolling/setup.sh' in script
    assert 'install/setup.sh' in script
    assert PID_MARKER in script
    assert 'MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600' in script


def test_agent_script_does_not_mix_ros_and_local_fastdds_libraries():
    script = build_remote_script(
        '~/multi_drone_real',
        'MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600',
        source_ros_environment=False,
    )

    assert 'source /opt/ros/rolling/setup.sh' not in script
    assert 'source install/setup.sh' not in script
    assert 'unset PYTHONPATH LD_LIBRARY_PATH' in script
    assert 'standalone environment; ROS setup skipped' in script


def test_station_script_is_interactive_original_station(tmp_path):
    script = build_station_script(str(tmp_path))

    assert f'cd -- {tmp_path}' in script
    assert 'unset ROS_DOMAIN_ID' in script
    assert 'exec ros2 run swarm_station station' in script


def test_passwords_are_removed_from_saved_settings(tmp_path):
    source = {
        'station_workspace': '/workspace',
        'password': 'top-secret',
        'drones': {
            '1': {
                'host': '10.0.0.1',
                'username': 'pilot',
                'password': 'drone-secret',
                'nested': {'admin_password': 'also-secret'},
            }
        },
    }
    path = tmp_path / 'launcher.json'

    save_settings(path, source)
    saved_text = path.read_text(encoding='utf-8')
    loaded = load_settings(path)

    assert 'secret' not in saved_text
    assert 'password' not in saved_text.lower()
    assert loaded['drones']['1']['host'] == '10.0.0.1'
    assert os.stat(path).st_mode & 0o777 == 0o600


def test_safe_settings_payload_handles_lists_and_non_mappings():
    source = {
        'items': [
            {'host': 'drone1', 'password': 'hidden'},
            'plain',
        ]
    }

    assert safe_settings_payload(source) == {
        'items': [{'host': 'drone1'}, 'plain']
    }


def test_invalid_settings_root_is_rejected(tmp_path):
    path = tmp_path / 'launcher.json'
    path.write_text(json.dumps(['not', 'an', 'object']), encoding='utf-8')

    try:
        load_settings(path)
    except ValueError as error:
        assert 'JSON object' in str(error)
    else:
        raise AssertionError('invalid settings root was accepted')
