import pytest

from robotics_description.bridge_configurations import create_battery_bridges


def test_create_battery_bridges() -> None:
    assert create_battery_bridges(
        model_name='flart1',
        battery_name='main_battery',
        battery_state_ros_topic='main_battery/state',
        battery_recharge_start_ros_topic='main_battery/recharge/start',
        battery_recharge_stop_ros_topic='main_battery/recharge/stop',
    ) == {
        'bridge_names': ['battery_state', 'battery_recharge_start', 'battery_recharge_stop'],
        'bridges': {
            'battery_state': {
                'ros_topic_name': 'main_battery/state',
                'ros_type_name': 'sensor_msgs/msg/BatteryState',
                'gz_topic_name': '/model/flart1/battery/main_battery/state',
                'gz_type_name': 'gz.msgs.BatteryState',
                'direction': 'GZ_TO_ROS',
                'qos_profile': 'SENSOR_DATA',
                'lazy': True,
            },
            'battery_recharge_start': {
                'ros_topic_name': 'main_battery/recharge/start',
                'ros_type_name': 'std_msgs/msg/Bool',
                'gz_topic_name': '/model/flart1/battery/main_battery/recharge/start',
                'gz_type_name': 'gz.msgs.Boolean',
                'direction': 'ROS_TO_GZ',
                'qos_profile': 'SYSTEM_DEFAULT',
                'lazy': False,
            },
            'battery_recharge_stop': {
                'ros_topic_name': 'main_battery/recharge/stop',
                'ros_type_name': 'std_msgs/msg/Bool',
                'gz_topic_name': '/model/flart1/battery/main_battery/recharge/stop',
                'gz_type_name': 'gz.msgs.Boolean',
                'direction': 'ROS_TO_GZ',
                'qos_profile': 'SYSTEM_DEFAULT',
                'lazy': False,
            },
        },
    }


@pytest.mark.parametrize(
    ('field_name', 'kwargs'),
    [
        ('model_name', {'model_name': '  '}),
        ('battery_name', {'battery_name': ''}),
        ('battery_state_ros_topic', {'battery_state_ros_topic': ''}),
        ('battery_recharge_start_ros_topic', {'battery_recharge_start_ros_topic': ''}),
        ('battery_recharge_stop_ros_topic', {'battery_recharge_stop_ros_topic': ''}),
    ],
)
def test_create_battery_bridges_rejects_empty_strings(field_name: str, kwargs: dict[str, str]) -> None:
    args = {
        'model_name': 'flart1',
        'battery_name': 'main_battery',
        'battery_state_ros_topic': 'main_battery/state',
        'battery_recharge_start_ros_topic': 'main_battery/recharge/start',
        'battery_recharge_stop_ros_topic': 'main_battery/recharge/stop',
    }
    args.update(kwargs)

    with pytest.raises(ValueError, match=field_name):
        create_battery_bridges(**args)
