import os
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

from eut_robotics_description.tools import make_robot_namespace
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    # Build fully-qualified robot namespace using helper function.
    robot_namespace = make_robot_namespace(namespace, robot_name)

    # ldes -> (l)aunch (d)escription (e)ntitie(s)
    ldes = [
        DeclareLaunchArgument('robot_name', default_value='flart', description='The unique name for the robot'),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for all resources'),
        SetLaunchConfiguration('robot_namespace', robot_namespace),
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
            'use_composition',
            default_value='False',
            choices=['True', 'true', 'False', 'false'],
            description='Use composition for the bridge node if True (default: False)',
        ),
        DeclareLaunchArgument(
            'create_own_container',
            default_value='False',
            choices=['True', 'true', 'False', 'false'],
            description='Whether we should start our own ROS container when using composition.',
        ),
        DeclareLaunchArgument('container_name', default_value='ros_gz_container', description='Name of the container'),
        DeclareLaunchArgument(
            'respawn_bridge',
            default_value='False',
            choices=['True', 'true', 'False', 'false'],
            description='Whether to respawn the bridge node if it dies (only effective when use_composition:=False)',
        ),
        DeclareLaunchArgument(
            'bridge_log_level',
            default_value='info',
            choices=['debug', 'info', 'warn', 'error'],
            description='Log level for the bridge node (default: info)',
        ),
        LogInfo(msg=['[', robot_namespace, '] Launching rosgz bridge']),
        LogInfo(msg=['[', robot_namespace, '] use_composition: ', LaunchConfiguration('use_composition')]),
        # Launch the bridge; the function will no-op if no sensors are enabled.
        OpaqueFunction(function=launch_rosgz_bridge),
    ]

    return LaunchDescription(ldes)


# ------------------------------------------------------------------------------
# Opaque functions.
# ------------------------------------------------------------------------------


def create_composable_bridge(
    ctx: LaunchContext, bridge_name: str, bridge_cfg: dict[str, Any]
) -> list[LaunchDescriptionEntity]:
    # ldes = (l)aunch (d)escription (e)ntitie(s) to return.
    ldes: list[LaunchDescriptionEntity] = []

    namespace = LaunchConfiguration('namespace').perform(ctx)
    container_name = LaunchConfiguration('container_name').perform(ctx)

    # If 'create_own_container' is False, a container named 'container_name' MUST already exist in the given
    # 'namespace' (e.g., the one created by the simulation launch). Otherwise, LoadComposableNodes will fail because
    # the target container cannot be found. When True, we create the container here explicitly.
    create_own_container = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('create_own_container'), bool), bool
    )

    # Check if a new container is required or not.
    if create_own_container:
        # Append the new created container.
        ldes.append(
            Node(
                package='rclcpp_components',
                executable='component_container',
                name=container_name,
                namespace=namespace,
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('bridge_log_level')],
            )
        )

    if namespace in ('', '/'):
        target_container = namespace + container_name
    else:
        # Make sure the namespace does not end with a slash, to avoid '//' in the target_container name.
        # Once we are sure no trailing '/' is present, we can concatenate the namespace, a '/' and the container name.
        target_container = namespace + '/' + container_name

    robot_namespace = LaunchConfiguration('robot_namespace').perform(ctx)

    # Load composable nodes into the target container. Note: when 'create_own_container' is False, the container must
    # already exist in 'target_container'; otherwise this action will fail to find it.
    ldes.extend(
        [
            LogInfo(msg=['[', robot_namespace, '] create_own_container: ', str(create_own_container)]),
            LogInfo(msg=['[', robot_namespace, '] target_container: ', target_container]),
            LoadComposableNodes(
                target_container=target_container,
                composable_node_descriptions=[
                    ComposableNode(
                        package='ros_gz_bridge',
                        plugin='ros_gz_bridge::RosGzBridge',
                        name=bridge_name,
                        namespace=namespace,
                        parameters=[bridge_cfg],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    )
                ],
            ),
        ]
    )

    return ldes


def create_standard_bridge(
    ctx: LaunchContext, bridge_name: str, bridge_cfg: dict[str, Any]
) -> list[LaunchDescriptionEntity]:
    # Standard node configuration
    return [
        Node(
            package='ros_gz_bridge',
            executable='bridge_node',
            name=bridge_name,
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            respawn=LaunchConfiguration('respawn_bridge'),
            respawn_delay=2.0,
            parameters=[bridge_cfg],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('bridge_log_level')],
        )
    ]


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

    # Obtain the plugins configuration for the simulation.
    vel_ctrl_plugin_sim_cfg = data.get('gz_sim_systems_VelocityControl', {})
    use_vel_ctrl_plugin = vel_ctrl_plugin_sim_cfg.get('enabled', False)
    vel_ctrl_plugin_topic = f'{robot_namespace}/cmd_vel'

    ldes.append(LogInfo(msg=['[', robot_namespace, '] Simulate velocity control: ', str(use_vel_ctrl_plugin)]))

    odom_pub_plugin_sim_cfg = data.get('gz_sim_systems_OdometryPublisher', {})
    use_odom_pub_plugin = odom_pub_plugin_sim_cfg.get('enabled', False)
    odom_pub_plugin_topic = f'{robot_namespace}/odom'

    ldes.append(LogInfo(msg=['[', robot_namespace, '] Simulate odometry publisher: ', str(use_odom_pub_plugin)]))

    front_lidar_sim_cfg = data.get('front_lidar', {})
    use_front_lidar = front_lidar_sim_cfg.get('enabled', False)
    front_lidar_topic = f'{robot_namespace}/front_lidar/scan/points'

    ldes.append(LogInfo(msg=['[', robot_namespace, '] Simulate front LiDAR: ', str(use_front_lidar)]))

    front_imu_sim_cfg = data.get('front_imu', {})
    use_front_imu = front_imu_sim_cfg.get('enabled', False)
    front_imu_topic = f'{robot_namespace}/front_imu/data'

    ldes.append(LogInfo(msg=['[', robot_namespace, '] Simulate front IMU: ', str(use_front_imu)]))

    robot_name = LaunchConfiguration('robot_name').perform(ctx)

    bridge_name = f'rosgz_bridge_{robot_name}'

    bridge_cfg: dict[str, Any] = {
        'subscription_heartbeat': 1000,  # default value in 'ros_gz_bridge.cpp'
        'expand_gz_topic_names': True,
        'bridge_names': [],  # Add the names of the individual channels here.
    }

    if use_vel_ctrl_plugin:
        # One channel in the bridge for the velocity control commands.
        vel_ctrl_channel = f'{bridge_name}_vel_ctrl'

        bridge_cfg['bridge_names'].append(vel_ctrl_channel)
        bridge_cfg[f'bridges.{vel_ctrl_channel}.ros_topic_name'] = vel_ctrl_plugin_topic
        bridge_cfg[f'bridges.{vel_ctrl_channel}.gz_topic_name'] = vel_ctrl_plugin_topic
        bridge_cfg[f'bridges.{vel_ctrl_channel}.ros_type_name'] = 'geometry_msgs/msg/TwistStamped'
        bridge_cfg[f'bridges.{vel_ctrl_channel}.gz_type_name'] = 'gz.msgs.Twist'
        bridge_cfg[f'bridges.{vel_ctrl_channel}.direction'] = 'ROS_TO_GZ'
        bridge_cfg[f'bridges.{vel_ctrl_channel}.qos_profile'] = 'SENSOR_DATA'
        # Lazy subscription policy
        # Many bridges default to 'lazy: true' to avoid spinning up internal publishers/subscribers unless a
        # real client appears on the opposite side.
        # lazy = true  -> the bridge activates only when at least one ROS-side or GZ-side subscriber/publisher
        #                 exists, saving CPU/bandwidth.
        # lazy = false -> the bridge stays permanently connected, forwarding every message even if no node is
        #                 currently listening.
        # For /clock in simulation we normally force 'lazy: false' so the time source is always available as
        # soon as any ROS node starts.
        bridge_cfg[f'bridges.{vel_ctrl_channel}.lazy'] = True

    ldes.append(LogInfo(msg=['[', robot_namespace, '] Velocity control plugin topic: ', vel_ctrl_plugin_topic]))

    if use_odom_pub_plugin:
        # One channel in the bridge for the odometry publisher.
        odom_pub_channel = f'{bridge_name}_odom_pub'

        bridge_cfg['bridge_names'].append(odom_pub_channel)
        bridge_cfg[f'bridges.{odom_pub_channel}.ros_topic_name'] = odom_pub_plugin_topic
        bridge_cfg[f'bridges.{odom_pub_channel}.gz_topic_name'] = odom_pub_plugin_topic
        bridge_cfg[f'bridges.{odom_pub_channel}.ros_type_name'] = 'nav_msgs/msg/Odometry'
        bridge_cfg[f'bridges.{odom_pub_channel}.gz_type_name'] = 'gz.msgs.OdometryWithCovariance'
        bridge_cfg[f'bridges.{odom_pub_channel}.direction'] = 'GZ_TO_ROS'
        bridge_cfg[f'bridges.{odom_pub_channel}.qos_profile'] = 'SENSOR_DATA'
        # Lazy subscription policy
        # Many bridges default to 'lazy: true' to avoid spinning up internal publishers/subscribers unless a
        # real client appears on the opposite side.
        # lazy = true  -> the bridge activates only when at least one ROS-side or GZ-side subscriber/publisher
        #                 exists, saving CPU/bandwidth.
        # lazy = false -> the bridge stays permanently connected, forwarding every message even if no node is
        #                 currently listening.
        # For /clock in simulation we normally force 'lazy: false' so the time source is always available as
        # soon as any ROS node starts.
        bridge_cfg[f'bridges.{odom_pub_channel}.lazy'] = True

        ldes.append(LogInfo(msg=['[', robot_namespace, '] Odometry publisher plugin topic: ', odom_pub_plugin_topic]))

        # Bridge the tf message from GZ to ROS, joining the odometry frame and the robot root frame.
        odom_tf_broadcast_channel = f'{bridge_name}_odom_tf_broadcast'
        bridge_cfg['bridge_names'].append(odom_tf_broadcast_channel)
        bridge_cfg[f'bridges.{odom_tf_broadcast_channel}.ros_topic_name'] = '/tf'
        bridge_cfg[f'bridges.{odom_tf_broadcast_channel}.gz_topic_name'] = (
            f'{LaunchConfiguration("robot_namespace").perform(ctx)}/tf_odom_fr_robot_root_fr'
        )
        bridge_cfg[f'bridges.{odom_tf_broadcast_channel}.ros_type_name'] = 'tf2_msgs/msg/TFMessage'
        bridge_cfg[f'bridges.{odom_tf_broadcast_channel}.gz_type_name'] = 'gz.msgs.Pose_V'
        bridge_cfg[f'bridges.{odom_tf_broadcast_channel}.direction'] = 'GZ_TO_ROS'
        bridge_cfg[f'bridges.{odom_tf_broadcast_channel}.qos_profile'] = 'SENSOR_DATA'
        # Lazy subscription policy
        # Many bridges default to 'lazy: true' to avoid spinning up internal publishers/subscribers unless a
        # real client appears on the opposite side.
        # lazy = true  -> the bridge activates only when at least one ROS-side or GZ-side subscriber/publisher
        #                 exists, saving CPU/bandwidth.
        # lazy = false -> the bridge stays permanently connected, forwarding every message even if no node is
        #                 currently listening.
        # For /tf in simulation we normally force 'lazy: false' so the tf source is always broadcasted as soon as any
        # ROS node starts.
        bridge_cfg[f'bridges.{odom_tf_broadcast_channel}.lazy'] = False

        ldes.append(LogInfo(msg=['[', robot_namespace, "] tf odom fr -> robot's root fr published in topic /tf"]))

    if use_front_lidar:
        front_lidar_channel = f'{bridge_name}_front_lidar'  # One channel in the bridge for the front lidar.

        bridge_cfg['bridge_names'].append(front_lidar_channel)
        bridge_cfg[f'bridges.{front_lidar_channel}.ros_topic_name'] = front_lidar_topic
        bridge_cfg[f'bridges.{front_lidar_channel}.gz_topic_name'] = front_lidar_topic
        bridge_cfg[f'bridges.{front_lidar_channel}.ros_type_name'] = 'sensor_msgs/msg/PointCloud2'
        bridge_cfg[f'bridges.{front_lidar_channel}.gz_type_name'] = 'gz.msgs.PointCloudPacked'
        bridge_cfg[f'bridges.{front_lidar_channel}.direction'] = 'GZ_TO_ROS'
        bridge_cfg[f'bridges.{front_lidar_channel}.qos_profile'] = 'SENSOR_DATA'
        # Lazy subscription policy
        # Many bridges default to 'lazy: true' to avoid spinning up internal publishers/subscribers unless a
        # real client appears on the opposite side.
        # lazy = true  -> the bridge activates only when at least one ROS-side or GZ-side subscriber/publisher
        #                 exists, saving CPU/bandwidth.
        # lazy = false -> the bridge stays permanently connected, forwarding every message even if no node is
        #                 currently listening.
        # For /clock in simulation we normally force 'lazy: false' so the time source is always available as
        # soon as any ROS node starts.
        bridge_cfg[f'bridges.{front_lidar_channel}.lazy'] = True

        ldes.append(LogInfo(msg=['[', robot_namespace, '] Front LiDAR topic: ', front_lidar_topic]))

    if use_front_imu:
        # One channel in the bridge for the front imu.
        front_imu_channel = f'{bridge_name}_front_imu'

        bridge_cfg['bridge_names'].append(front_imu_channel)
        bridge_cfg[f'bridges.{front_imu_channel}.ros_topic_name'] = front_imu_topic
        bridge_cfg[f'bridges.{front_imu_channel}.gz_topic_name'] = front_imu_topic
        bridge_cfg[f'bridges.{front_imu_channel}.ros_type_name'] = 'sensor_msgs/msg/Imu'
        bridge_cfg[f'bridges.{front_imu_channel}.gz_type_name'] = 'gz.msgs.IMU'
        bridge_cfg[f'bridges.{front_imu_channel}.direction'] = 'GZ_TO_ROS'
        bridge_cfg[f'bridges.{front_imu_channel}.qos_profile'] = 'SENSOR_DATA'
        # Lazy subscription policy
        # Many bridges default to 'lazy: true' to avoid spinning up internal publishers/subscribers unless a
        # real client appears on the opposite side.
        # lazy = true  -> the bridge activates only when at least one ROS-side or GZ-side subscriber/publisher
        #                 exists, saving CPU/bandwidth.
        # lazy = false -> the bridge stays permanently connected, forwarding every message even if no node is
        #                 currently listening.
        # For /clock in simulation we normally force 'lazy: false' so the time source is always available as
        # soon as any ROS node starts.
        bridge_cfg[f'bridges.{front_imu_channel}.lazy'] = True

        ldes.append(LogInfo(msg=['[', robot_namespace, '] Front IMU topic: ', front_imu_topic]))

    # If no sensors are enabled, do nothing, i.e., return an empty list, so no bridge node is launched.
    if not bridge_cfg['bridge_names']:
        return []

    use_composition = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('use_composition'), bool), bool
    )

    ldes.append(LogInfo(msg=['[', robot_namespace, '] use_composition: ', str(use_composition)]))

    if use_composition:
        ldes.append(OpaqueFunction(function=create_composable_bridge, args=[bridge_name, bridge_cfg]))
    else:
        ldes.append(OpaqueFunction(function=create_standard_bridge, args=[bridge_name, bridge_cfg]))

    return ldes
