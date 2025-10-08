import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from eut_robotics_description.tools import make_robot_namespace
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    # Build fully-qualified robot namespace using helper function.
    robot_namespace = make_robot_namespace(namespace, robot_name)

    # ldes -> (l)aunch (d)escription (e)ntitie(s)
    ldes = [
        DeclareLaunchArgument('robot_name', default_value='flart', description='The unique name for the robot'),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for all resources'),
        DeclareLaunchArgument(
            'sim_cfg_file',
            default_value=os.path.join(
                get_package_share_directory('eut_robotics_description'),
                'config',
                'robots',
                'flart',
                'simulation_default.yaml',
            ),
            description='Path to the simulation configuration file (default: flart/simulation_default.yaml)',
        ),
        DeclareLaunchArgument(
            'respawn_rosgz_bridge',
            default_value='False',
            choices=['True', 'true', 'False', 'false'],
            description='Whether to respawn the rosgz_bridge_node if it dies (default: False)',
        ),
        DeclareLaunchArgument(
            'log_level_rosgz_bridge',
            default_value='info',
            choices=['debug', 'info', 'warn', 'error'],
            description='Log level for the rosgz_bridge_node (default: info)',
        ),
        SetLaunchConfiguration('robot_namespace', robot_namespace),
        OpaqueFunction(function=launch_rosgz_bridge),
    ]

    return LaunchDescription(ldes)


# ------------------------------------------------------------------------------
# Opaque functions.
# ------------------------------------------------------------------------------


def launch_rosgz_bridge(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # ldes = (l)aunch (d)escription (e)ntitie(s) to return.
    ldes: list[LaunchDescriptionEntity] = []

    # If we are in simulation mode, get the simulation configuration file provided by the user (be aware, the user
    # could pass an empty string), or the default one.
    sim_cfg_file = LaunchConfiguration('sim_cfg_file').perform(ctx)

    if not isinstance(sim_cfg_file, str):
        raise TypeError(f"Expected 'sim_cfg_file' to be of type 'str', but got '{type(sim_cfg_file)}'")

    if not sim_cfg_file:
        raise ValueError('The provided simulation configuration file is an empty string')

    sim_cfg_path = Path(sim_cfg_file)

    # If the user provided a simulation configuration file, check it exists.
    # If the user provided and empty string, it means no simulation configuration file provided, so no file
    # existence check is needed, but no sensor will be simulated.
    if not sim_cfg_path.is_file():
        raise FileNotFoundError(f"The provided simulation configuration file does not exist: '{sim_cfg_file}'")

    try:
        with sim_cfg_path.open('r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except Exception as e:
        raise RuntimeError(f"Invalid YAML at '{sim_cfg_file}': {e}") from e

    # Handle empty/None content explicitly
    if data is None:
        raise RuntimeError(f"Simulation configuration file '{sim_cfg_file}' is empty")

    if not isinstance(data, dict):
        raise RuntimeError(
            f"Simulation configuration file '{sim_cfg_file}' must be a mapping (YAML dict). Got: {type(data).__name__}"
        )

    # Validate non-empty mapping.
    if not data:
        raise RuntimeError(f"Parameters file '{sim_cfg_file}' has no nodes (empty mapping).")

    # When working in simulation, the topics used by GZ plugins installed in the 'flart' robot description (xacro
    # file) are fixed, they do not admit to be passed as parameters to the plugins.
    # Consequently, the topics used in this rosgz bridge to transfer messages to/from the GZ and ROS must match
    # those used in the robot description.
    # This is a convention adopted that has several advanteges:
    # 1. Reduce the mental burden of deciding where a topic must be defined, the rule is simple: if the topic is
    #    robot-related it must be defined in the robot description (xacro file).
    # 2. Other launch files needing to us those topics should use the same topics, so consistency is enforced.
    #    This is specially important when working with multiple robots, since the topics are automatically
    #    namespaced under the robot namespace.
    # 3. Avoids the need to pass many parameters around to configure topics, since they are fixed.
    # 4. For some reason the topics must be changed, do the changes in the robot description (xacro file)
    #    and in those launch files needing to use those topics.
    # These topics do really make sense since they have the form:
    # <robot_namespace>/<sensor_or_controller_or_plugin>/<topic_base_name>

    robot_namespace = LaunchConfiguration('robot_namespace').perform(ctx)

    # Obtain the configuration for each sensor/plugin from the simulation configuration file.

    base_vel_ctrl_plugin_sim_cfg = data.get('base_velocity_controller', {})
    use_base_vel_ctrl_plugin = base_vel_ctrl_plugin_sim_cfg.get('enabled', False)

    base_odom_pub_plugin_sim_cfg = data.get('base_odometry_publisher', {})
    use_base_odom_pub_plugin = base_odom_pub_plugin_sim_cfg.get('enabled', False)

    fork_pos_ctrl_plugin_sim_cfg = data.get('fork_position_controller', {})
    use_fork_pos_ctrl_plugin = fork_pos_ctrl_plugin_sim_cfg.get('enabled', False)

    fork_pos_pub_plugin_sim_cfg = data.get('fork_position_publisher', {})
    use_fork_pos_pub_plugin = fork_pos_pub_plugin_sim_cfg.get('enabled', False)

    front_lidar_sim_cfg = data.get('front_lidar', {})
    use_front_lidar = front_lidar_sim_cfg.get('enabled', False)

    front_imu_sim_cfg = data.get('front_imu', {})
    use_front_imu = front_imu_sim_cfg.get('enabled', False)

    rosgz_bridge_channels: list[dict] = []

    if use_base_vel_ctrl_plugin:
        # One channel in the rosgz_bridge for the velocity commands.
        vel_ctrl_plugin_topic = f'{robot_namespace}/cmd_vel'

        # Lazy subscription policy
        # Many bridges default to 'lazy: true' to avoid spinning up internal publishers/subscribers unless a
        # real client appears on the opposite side.
        # lazy = true  -> the bridge activates only when at least one ROS-side or GZ-side subscriber/publisher
        #                 exists, saving CPU/bandwidth.
        # lazy = false -> the bridge stays permanently connected, forwarding every message even if no node is
        #                 currently listening.
        # For /clock in simulation we normally force 'lazy: false' so the time source is always available as
        # soon as any ROS node starts.
        rosgz_bridge_channels.append(
            {
                'ros_topic_name': vel_ctrl_plugin_topic,
                'gz_topic_name': vel_ctrl_plugin_topic,
                'ros_type_name': 'geometry_msgs/msg/TwistStamped',
                'gz_type_name': 'gz.msgs.Twist',
                'direction': 'ROS_TO_GZ',
                'qos_profile': 'SENSOR_DATA',
                'lazy': True,
            }
        )

        ldes.append(LogInfo(msg=['[', robot_namespace, '] Bridging topic ', vel_ctrl_plugin_topic, ' (ROS -> GZ)']))

    if use_base_odom_pub_plugin:
        # One channel in the rosgz_bridge for the odometry.
        odom_pub_plugin_topic = f'{robot_namespace}/odom'

        rosgz_bridge_channels.append(
            {
                'ros_topic_name': odom_pub_plugin_topic,
                'gz_topic_name': odom_pub_plugin_topic,
                'ros_type_name': 'nav_msgs/msg/Odometry',
                'gz_type_name': 'gz.msgs.OdometryWithCovariance',
                'direction': 'GZ_TO_ROS',
                'qos_profile': 'SENSOR_DATA',
                'lazy': True,
            }
        )

        ldes.append(LogInfo(msg=['[', robot_namespace, '] Bridging topic ', odom_pub_plugin_topic, ' (GZ -> ROS)']))

        # Bridge the tf message from GZ to ROS, joining the odometry frame and the robot's root frame.
        rosgz_bridge_channels.append(
            {
                'ros_topic_name': '/tf',
                'gz_topic_name': f'{robot_namespace}/tf_odom_fr_robot_root_fr',
                'ros_type_name': 'tf2_msgs/msg/TFMessage',
                'gz_type_name': 'gz.msgs.Pose_V',
                'direction': 'GZ_TO_ROS',
                'qos_profile': 'SENSOR_DATA',
                'lazy': False,  # /tf topic should not be lazy.
            }
        )

        ldes.append(LogInfo(msg=['[', robot_namespace, "] Bridging tf odom fr -> robot's root fr (GZ -> ROS)"]))

    if use_fork_pos_ctrl_plugin:
        # One channel in the rosgz_bridge for the fork position commands.
        fork_pos_ctrl_plugin_topic = f'{robot_namespace}/robot_description/fork_root_joint/cmd_pos'

        rosgz_bridge_channels.append(
            {
                'ros_topic_name': fork_pos_ctrl_plugin_topic,
                'gz_topic_name': fork_pos_ctrl_plugin_topic,
                'ros_type_name': 'std_msgs/msg/Float64',
                'gz_type_name': 'gz.msgs.Double',
                'direction': 'ROS_TO_GZ',
                'qos_profile': 'SENSOR_DATA',
                'lazy': True,
            }
        )

        ldes.append(
            LogInfo(msg=['[', robot_namespace, '] Bridging topic ', fork_pos_ctrl_plugin_topic, ' (ROS -> GZ)'])
        )

    if use_fork_pos_pub_plugin:
        # One channel in the rosgz_bridge for the fork position.
        fork_pos_pub_plugin_topic = f'{robot_namespace}/robot_description/fork_root_joint/pos'

        rosgz_bridge_channels.append(
            {
                'ros_topic_name': fork_pos_pub_plugin_topic,
                'gz_topic_name': fork_pos_pub_plugin_topic,
                'ros_type_name': 'sensor_msgs/msg/JointState',
                'gz_type_name': 'gz.msgs.Model',
                'direction': 'GZ_TO_ROS',
                'qos_profile': 'SENSOR_DATA',
                'lazy': True,
            }
        )

        ldes.append(LogInfo(msg=['[', robot_namespace, '] Bridging topic ', fork_pos_pub_plugin_topic, ' (GZ -> ROS)']))

    if use_front_lidar:
        # One channel in the rosgz_bridge for the front lidar.
        front_lidar_topic = f'{robot_namespace}/front_lidar/scan/points'

        rosgz_bridge_channels.append(
            {
                'ros_topic_name': front_lidar_topic,
                'gz_topic_name': front_lidar_topic,
                'ros_type_name': 'sensor_msgs/msg/PointCloud2',
                'gz_type_name': 'gz.msgs.PointCloudPacked',
                'direction': 'GZ_TO_ROS',
                'qos_profile': 'SENSOR_DATA',
                'lazy': True,
            }
        )

        ldes.append(LogInfo(msg=['[', robot_namespace, '] Bridging topic ', front_lidar_topic, ' (GZ -> ROS)']))

    if use_front_imu:
        # One channel in the bridge for the front imu.
        front_imu_topic = f'{robot_namespace}/front_imu/data'

        rosgz_bridge_channels.append(
            {
                'ros_topic_name': front_imu_topic,
                'gz_topic_name': front_imu_topic,
                'ros_type_name': 'sensor_msgs/msg/Imu',
                'gz_type_name': 'gz.msgs.IMU',
                'direction': 'GZ_TO_ROS',
                'qos_profile': 'SENSOR_DATA',
                'lazy': True,
            }
        )

        ldes.append(LogInfo(msg=['[', robot_namespace, '] Bridging topic ', front_imu_topic, ' (GZ -> ROS)']))

    # If no sensors are enabled, do nothing, i.e., return an empty list, so no bridge node is launched.
    if not rosgz_bridge_channels:
        return []

    # Write the rosgz_bridge configuration file for this robot into a file, so we can later pass it to the
    # rosgz_bridge node.
    # In ROS2-Humble, the node only accepts a file, not a set of parameters.
    ros_home = Path(os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')))
    rosgz_bridge_file = robot_namespace.strip('/').replace('/', '_') + '_rosgz_bridge.yaml'
    abs_rosgz_bridge_file = os.path.join(ros_home, rosgz_bridge_file)
    abs_rosgz_bridge_path = Path(abs_rosgz_bridge_file)

    # Make sure the parent directory exists.
    abs_rosgz_bridge_path.parent.mkdir(parents=True, exist_ok=True)

    # Write the file to disk.
    with abs_rosgz_bridge_path.open('w', encoding='utf-8') as f:
        yaml.safe_dump(
            rosgz_bridge_channels, stream=f, sort_keys=False, default_flow_style=False, allow_unicode=True, width=120
        )

    ldes.append(
        Node(
            package='ros_gz_bridge',
            executable='bridge_node',
            name='rosgz_bridge',
            namespace=robot_namespace,
            output='screen',
            respawn=LaunchConfiguration('respawn_rosgz_bridge'),
            respawn_delay=2.0,
            parameters=[
                {
                    'subscription_heartbeat': 1000,  # default value in 'ros_gz_bridge.cpp'
                    'config_file': abs_rosgz_bridge_file,
                    'expand_gz_topic_names': False,
                    'override_timestamps_with_wall_time': False,
                }
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level_rosgz_bridge')],
        )
    )

    return ldes
