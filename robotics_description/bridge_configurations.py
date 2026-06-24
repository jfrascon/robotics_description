from typing import Any


def create_battery_bridges(
    model_name: str,
    battery_name: str,
    battery_state_ros_topic: str,
    battery_recharge_start_ros_topic: str,
    battery_recharge_stop_ros_topic: str,
) -> dict[str, Any]:
    """
    Build ros_gz_bridge parameters for the Gazebo linear battery plugin.

    The Gazebo linear battery plugin publishes and subscribes on model-scoped Gazebo Transport
    topics. Those Gazebo topic names are fixed by the plugin and cannot be configured from xacro
    in the same way as most sensor plugin topics.

    This function builds the bridge entries for that fixed Gazebo topic layout and lets each robot
    choose the ROS topic names that should appear in its ROS namespace.
    """
    _validate_non_empty_string(model_name, 'model_name')
    _validate_non_empty_string(battery_name, 'battery_name')
    _validate_non_empty_string(battery_state_ros_topic, 'battery_state_ros_topic')
    _validate_non_empty_string(battery_recharge_start_ros_topic, 'battery_recharge_start_ros_topic')
    _validate_non_empty_string(battery_recharge_stop_ros_topic, 'battery_recharge_stop_ros_topic')

    battery_state_bridge_name = 'battery_state'
    battery_recharge_start_bridge_name = 'battery_recharge_start'
    battery_recharge_stop_bridge_name = 'battery_recharge_stop'

    return {
        'bridge_names': [
            battery_state_bridge_name,
            battery_recharge_start_bridge_name,
            battery_recharge_stop_bridge_name,
        ],
        'bridges': {
            battery_state_bridge_name: {
                'ros_topic_name': battery_state_ros_topic,
                'ros_type_name': 'sensor_msgs/msg/BatteryState',
                'gz_topic_name': f'/model/{model_name}/battery/{battery_name}/state',
                'gz_type_name': 'gz.msgs.BatteryState',
                'direction': 'GZ_TO_ROS',
                'qos_profile': 'SENSOR_DATA',
                'lazy': True,
            },
            battery_recharge_start_bridge_name: {
                'ros_topic_name': battery_recharge_start_ros_topic,
                'ros_type_name': 'std_msgs/msg/Bool',
                'gz_topic_name': f'/model/{model_name}/battery/{battery_name}/recharge/start',
                'gz_type_name': 'gz.msgs.Boolean',
                'direction': 'ROS_TO_GZ',
                'qos_profile': 'SYSTEM_DEFAULT',
                'lazy': False,
            },
            battery_recharge_stop_bridge_name: {
                'ros_topic_name': battery_recharge_stop_ros_topic,
                'ros_type_name': 'std_msgs/msg/Bool',
                'gz_topic_name': f'/model/{model_name}/battery/{battery_name}/recharge/stop',
                'gz_type_name': 'gz.msgs.Boolean',
                'direction': 'ROS_TO_GZ',
                'qos_profile': 'SYSTEM_DEFAULT',
                'lazy': False,
            },
        },
    }


def _validate_non_empty_string(value: str, field_name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"'{field_name}' must be a non-empty string.")
