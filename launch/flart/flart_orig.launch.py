import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from jinja2 import Template
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterValue

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EqualsSubstitution,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    OrSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution

# The name of the robot model, 'flart', is obtained from the combination of the words 'forklift' and 'artisteril',
# which is the company that manufactures the forklift robot.

# ======================================================================================================================
# NOTE: If you include this python launch file in a parent launch, DO NOT use the action PushRosNamespace preceding this
# launch file, pass the namespace as a parameter to this launch file instead.
# Reasons:
# 1. We need the namespace here to create the robot_namespace and robot_prefix to be passed to the xacro file that
#    defines the robot.
# 2. The namespace is also used to create the topics published by the gazebo plugins used in the xacro file.
# 3. If you inclue this python launch file in parent launch, using the action PushRosNamespace along with this
# launch file, when 'use_composition' is set to True, the composable bridge node to load into a container do no get the
# namespace automatically, when the namespace is different from pure '/', so they do not find the container
# (namespaced), and they are not loaded.
# There is PR to solve that issue, in which composable nodes get the namespace automatically, but this PR has not been
# merged yet (2025-08-06) since it is no clear the approach the ROS community wants to follow, regarding inhering the
# namespace for composable nodes.
# Issue: https://github.com/ros2/launch_ros/issues/428
# PR: https://github.com/ros2/launch_ros/pull/429 -> There is kind of a discussion here about the approach to follow.
# ======================================================================================================================

# For more information about composition you can read an introduction in:
# https://gazebosim.org/docs/harmonic/ros2_overview/#composition

# For a tutorial in composition you can read:
# https://docs.ros.org/en/jazzy/How-To-Guides/Launching-composable-nodes.html#launch-file-examples

# When running a bridge you might find different ways:
# 1. Using a RosGzBridge action.
# 2. Using the node 'parameter_bridge' from the package 'ros_gz_bridge', like shown in the example 7 of the README
# file found in the package 'ros_gz_bridge':
# https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge#example-7-configuring-the-bridge-via-python-launch-file
# Node(
#     package="ros_gz_bridge",
#     executable="parameter_bridge",
#     parameters=[
#         {"bridge_names": ["clock_bridge"]},
#         {"bridges.clock_bridge.ros_topic_name": "/clock"},
#         {"bridges.clock_bridge.gz_topic_name": "/clock"},
#         {"bridges.clock_bridge.ros_type_name": "rosgraph_msgs/msg/Clock"},
#         {"bridges.clock_bridge.gz_type_name": "gz.msgs.Clock"},
#         {"bridges.clock_bridge.direction": "GZ_TO_ROS"},
#         {"bridges.clock_bridge.lazy": "False"},
#         {"bridges.clock_bridge.qos_profile": "CLOCK"},
#     ],
# )
# 3. Using the same node 'parameter_bridge' from the package 'ros_gz_bridge', but using the arguments instead of
# parameters.
# Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         name='clock_gz_bridge',
#         output='screen',
#         # Descriptions at:
#         # Reference:
#         https://github.com/gazebosim/ros_gz/tree/77522600db37d49a23e349c6e109b08caa621188/ros_gz_bridge#readme
#         # Referente:
#         https://github.com/gazebosim/ros_gz/blob/77522600db37d49a23e349c6e109b08caa621188/ros_gz_bridge/src/parameter_bridge.cpp#L30
#         arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
#     )
# 4. Using the executable directly, like we do in this launch file.

# Method 3 is less preferred, since it can't specify all the options, like the other methods, just by using the format
# '<topic_name>@<ros_type_name><direction><gz_type_name>'.


def generate_launch_description():
    # (L)aunch (d)escription (e)ntitie(s)
    ldes = [OpaqueFunction(function=declare_launch_arguments)]

    # The 'prefix' is similar to the 'namespace', it is a 'flatenized' version of the namespace, i.e., it uses the
    # character '_' as a separator instead of the character '/'.
    # The 'namespace' and the 'robot_name' are passed to the xacro file to make clear to the user that a 'robot_name' is
    # required to distinguish between multiple robots, and the 'namespace' is optional.
    # The 'robot_namespace' is the concatenation of the 'namespace' and the 'robot_name', using the character '/' a
    # separtor. The 'robot_namespace' is prepended to nodes and topics when required.
    # The 'robot_namespace' is computed in this launch file and also inside the xacro file, using the same rules.
    # The 'robot_prefix' is the concatenation of the 'prefix' and the 'robot_name', # using the character '_' as a
    # separator. The 'robot_prefix' is prepended to the links and joints defined in the xacro file.
    # The 'robot_prefix' is computed inside the xacro file.

    # namespace=''          -> robot_namespace = robot_name
    #                          prefix = ''
    #                          robot_prefix = robot_name + '_'
    # namespace='/'         -> robot_namespace = '/' + robot_name
    #                          prefix = ''
    #                          robot_prefix = robot_name + '_'
    # namespace='ns'        -> robot_namespace = 'ns' + '/' + robot_name
    #                          prefix = 'ns' + '_'
    #                          robot_prefix = 'ns' + '_' + robot_name + '_'
    # namespace='ns/'       -> robot_namespace = 'ns' + '/' + robot_name
    #                          prefix = 'ns' + '_'
    #                          robot_prefix = 'ns' + '_' + robot_name + '_'
    # namespace='/ns/'      -> robot_namespace = '/ns' + '/' + robot_name
    #                          prefix = 'ns' + '_'
    #                          robot_prefix = 'ns' + '_' + robot_name + '_'
    # namespace='/ns1/ns2'  -> robot_namespace = '/ns1/ns2' + '/' + robot_name
    #                          prefix = 'ns1_ns2' + '_'
    #                          robot_prefix = 'ns1_ns2' + '_' + robot_name + '_'
    # namespace='/ns1/ns2/' -> robot_namespace = '/ns1/ns2' + '/' + robot_name
    #                          prefix = 'ns1_ns2' + '_'
    #                          robot_prefix = 'ns1_ns2' + '_' + robot_name + '_'
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')

    ns_is_empty_or_slash = OrSubstitution(EqualsSubstitution(namespace, ''), EqualsSubstitution(namespace, '/'))

    robot_namespace = IfElseSubstitution(
        condition=ns_is_empty_or_slash,
        # PythonExpression first subsitutes the variables and then perform the evaluation.
        if_value=[namespace, robot_name],
        else_value=PythonExpression(["'", namespace, "'.rstrip('/') + '/", robot_name, "'"]),
    )

    robot_prefix = IfElseSubstitution(
        condition=ns_is_empty_or_slash,
        if_value=[robot_name, '_'],
        else_value=PythonExpression(["'", namespace, "'.strip('/').replace('/', '_') + '_' + '", robot_name, "_'"]),
    )

    ldes += [
        SetLaunchConfiguration('robot_namespace', robot_namespace),
        SetLaunchConfiguration('robot_prefix', robot_prefix),
        LogInfo(msg=['use_sim_mode: ', LaunchConfiguration('use_sim_time')]),
        LogInfo(msg=['namespace: ', namespace]),
        LogInfo(msg=['robot_name: ', robot_name]),
        LogInfo(msg=['robot_namespace: ', robot_namespace]),
        LogInfo(msg=['robot_prefix: ', robot_prefix]),
        OpaqueFunction(function=launch_robot_state_publisher),
        OpaqueFunction(function=launch_sensors),
        OpaqueFunction(function=launch_ros2_control),
    ]

    return LaunchDescription(ldes)


# ----------------------------------------------------------------------------------------------------------------------

# Opaque functions.


def create_composable_bridge(
    ctx: LaunchContext, bridge_name: str, bridge_params: list[dict[str, Any]], bridge_remappings: list[tuple[str, str]]
) -> list[LaunchDescriptionEntity]:
    # (L)aunch (d)escription (e)ntitie(s) to return.
    ldes: list[LaunchDescriptionEntity] = []

    namespace = LaunchConfiguration('namespace').perform(ctx).strip()
    container_name = LaunchConfiguration('container_name').perform(ctx)

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
            )
        )

    if namespace in ('', '/'):
        target_container = namespace + container_name
    else:
        # No need to use rstrip('/') here, since the namespace has already been normalized, and any leading slash has
        # been removed.
        target_container = namespace + '/' + container_name

    # Composable nodes that are launched in a just created container, or in an existing container.
    ldes.append(
        LoadComposableNodes(
            target_container=target_container,
            composable_node_descriptions=[
                ComposableNode(
                    package='ros_gz_bridge',
                    plugin='ros_gz_bridge::RosGzBridge',
                    name=bridge_name,
                    namespace=namespace,
                    parameters=bridge_params,
                    remappings=bridge_remappings,
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
        )
    )

    return ldes


def create_standard_bridge(
    ctx: LaunchContext, bridge_name: str, bridge_params: list[dict[str, Any]], bridge_remappings: list[tuple[str, str]]
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
            parameters=bridge_params,
            remappings=bridge_remappings,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('bridge_log_level')],
        )
    ]


def declare_launch_arguments(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # (L)aunch (d)escription (e)ntitie(s) to return.
    return [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            choices=['True', 'true', 'False', 'false'],
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument('robot_name', default_value='flart', description='The unique name for the robot'),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for all resources'),
        DeclareLaunchArgument(
            'use_visual_meshes',
            default_value='True',
            choices=['True', 'true', 'False', 'false'],
            description='Whether to use visual meshes if True, or simple shapes if False (default: True)',
        ),
        DeclareLaunchArgument(
            'use_collision_meshes',
            default_value='False',
            choices=['True', 'true', 'False', 'false'],
            description='Whether to use collision meshes if True, or simple shapes if False (default: False)',
        ),
        DeclareLaunchArgument(
            'sim_cfg_file',
            default_value=os.path.join(
                get_package_share_directory('eut_robotics_description'),
                'config',
                'robots',
                'flart',
                'simulation_default.yaml',
            ),
            description='Path to the simulation configuration file',
        ),
        DeclareLaunchArgument(
            'rsp_publish_frequency',
            default_value='20.0',
            description='Frequency of publication for robot_state_publisher',
        ),
        DeclareLaunchArgument(
            'odometry_frame', default_value='odom', description='The odometry frame used in the project'
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
            description='Whether to respawn the bridge node if it dies (default: False)',
        ),
        DeclareLaunchArgument(
            'bridge_log_level',
            default_value='info',
            choices=['debug', 'info', 'warn', 'error'],
            description='Log level for the bridge node (default: info)',
        ),
    ]


def launch_control_manager_and_controllers_real(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    return []


def launch_controllers_sim(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    robot_prefix = LaunchConfiguration('robot_prefix').perform(ctx)

    # Read the configuration file for the flart robot, so we can read the valus we need to render the controllers.
    flart_cfg_file = os.path.join(
        get_package_share_directory('eut_robotics_description'), 'config', 'robots', 'flart', 'description.yaml'
    )

    with open(flart_cfg_file) as f:
        flart_cfg = yaml.safe_load(f)

    # Render the YAML file with the controller configuration for simulation.
    controllers_sim_template = os.path.join(
        get_package_share_directory('eut_robotics_description'), 'config', 'robots', 'flart', 'controllers_sim.j2'
    )
    controllers_sim_file = os.path.join(
        get_package_share_directory('eut_robotics_description'), 'config', 'robots', 'flartcontrollers_sim.yaml'
    )

    with open(controllers_sim_template) as f:
        template = Template(f.read())

    # A wheel is a cilinder with axis along Y, so size_x is the diameter.
    lx = 0.5 * flart_cfg['mecanum_rectangular_base']['body']['wheelbase']
    ly = 0.5 * flart_cfg['mecanum_rectangular_base']['body']['track_width']

    render_context = {
        'front_left_wheel_joint_name': f'{robot_prefix}front_left_wheel_joint',
        'front_right_wheel_joint_name': f'{robot_prefix}front_right_wheel_joint',
        'rear_left_wheel_joint_name': f'{robot_prefix}rear_left_wheel_joint',
        'rear_right_wheel_joint_name': f'{robot_prefix}rear_right_wheel_joint',
        'wheels_radius': 0.5 * flart_cfg['mecanum_rectangular_base']['mecanum_wheel']['size']['x'],
        'lx_plus_ly': lx + ly,
        'base_frame_name': f'{robot_prefix}base_footprint_link',
        'odom_frame_name': robot_prefix + LaunchConfiguration('odometry_frame').perform(ctx),
        'fork_joint_name': f'{robot_prefix}fork_root_joint',
    }

    rendered = template.render(render_context)

    with open(controllers_sim_file, 'w') as f:
        f.write(rendered)

    robot_namespace = LaunchConfiguration('robot_namespace').perform(ctx).strip()
    fq_controller_manager = robot_namespace + '/controller_manager'

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        namespace=robot_namespace,
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            fq_controller_manager,
            '--controller-manager-timeout',
            '60',
            '--param-file',
            controllers_sim_file,
        ],
        output='screen',
    )

    mecanum_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='mecanum_drive_controller_spawner',
        namespace=robot_namespace,
        arguments=[
            'mecanum_drive_controller',
            '--controller-manager',
            fq_controller_manager,
            '--controller-manager-timeout',
            '60',
            '--param-file',
            controllers_sim_file,
        ],
        output='screen',
    )

    # Start the mecanum drive controller (mdc) after the joint state broadcaster (jsb).
    start_mdc_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[mecanum_drive_controller])
    )

    fork_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='fork_controller_spawner',
        namespace=robot_namespace,
        arguments=[
            'fork_controller',
            '--controller-manager',
            fq_controller_manager,
            '--controller-manager-timeout',
            '60',
            '--param-file',
            controllers_sim_file,
        ],
        output='screen',
    )

    # Start the fork controller after the mecanum drive controller (mdc).
    start_fork_controller_after_mdc = RegisterEventHandler(
        OnProcessExit(target_action=mecanum_drive_controller, on_exit=[fork_controller])
    )

    return [joint_state_broadcaster, start_mdc_after_jsb, start_fork_controller_after_mdc]


def launch_robot_state_publisher(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    use_sim_time = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('use_sim_time'), bool), bool
    )

    robot_description_param = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'),
                ' ',
                os.path.join(get_package_share_directory('eut_robotics_description'), 'urdf', 'robots', 'flart.xacro'),
                ' robot_name:=',
                LaunchConfiguration('robot_name'),
                ' namespace:=',
                LaunchConfiguration('namespace'),
                ' use_visual_meshes:=',
                LaunchConfiguration('use_visual_meshes'),
                ' use_collision_meshes:=',
                LaunchConfiguration('use_collision_meshes'),
                ' use_sim:=',
                LaunchConfiguration('use_sim_time'),
                ' sim_cfg_file:=',
                LaunchConfiguration('sim_cfg_file') if use_sim_time else TextSubstitution(text=''),
            ]
        ),
        value_type=str,
    )

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=LaunchConfiguration('robot_namespace'),
            # Both methods of passing parameters are allowed:
            # Method 1: Using a list of dictionaries, where each dictionary contains a key-value pair.
            # parameters = [
            #     {'key1': value1},
            #     {'key2': value2},
            #     ...
            # ]
            # Method 2: Using a single dictionary with all key-value pairs.
            # parameters = [
            #     {
            #         'key1': value1,
            #         'key2': value2,
            #         ...
            #     }
            # ]
            parameters=[
                {
                    'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                    'robot_description': robot_description_param,
                    #'frame_prefix': "DO NOT USE IT"
                    'publish_frequency': ParameterValue(LaunchConfiguration('rsp_publish_frequency'), value_type=float),
                }
            ],
            output='screen',
        )
    ]


def launch_ros2_control(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    use_sim_time = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('use_sim_time'), bool), bool
    )

    if use_sim_time:
        return [OpaqueFunction(function=launch_controllers_sim)]
    else:
        return [OpaqueFunction(function=launch_control_manager_and_controllers_real)]


def launch_sensors(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    """
    Launch the sensors based on the 'use_sensor_sim' argument.
    If 'use_sensor_sim' is True, launch the sensor bridge to transfer data from Gazebo to ROS.
    If 'use_sensor_sim' is False, launch the real sensors.
    """
    use_sim_time = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('use_sim_time'), bool), bool
    )

    return (
        [OpaqueFunction(function=launch_sensor_bridge)]
        if use_sim_time
        else [OpaqueFunction(function=launch_real_sensors)]
    )


def launch_sensor_bridge(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # The parameter 'robot_name' has already been checked in the OpaqueFunction 'check_robot_name', so we can use it
    # directly.
    bridge_name = f'rosgz_bridge_{LaunchConfiguration("robot_name").perform(ctx)}'

    use_composition = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('use_composition'), bool), bool
    )

    # One bridge node, with multiple channels, one for the 'front_lidar' and one for 'front_imu'.
    front_lidar_channel = f'{bridge_name}_front_lidar'
    front_imu_channel = f'{bridge_name}_front_imu'
    # The topics for the lidar_3d_ring sensor and the imu sensor are defined inside the xacro macros
    # 'lidar_3d_ring_macro.xacro' and 'imu_macro.xacro', respectively, with structure
    # lidar_sim_topic = '<robot_namespace>/<lidar_name>/points'
    # imu_sim_topic = '<robot_namespace>/<imu_name>/data'
    front_lidar_sim_topic = f'{LaunchConfiguration("robot_namespace").perform(ctx)}/front_lidar/points'
    front_imu_sim_topic = f'{LaunchConfiguration("robot_namespace").perform(ctx)}/front_imu/data'

    bridge_params = [
        {
            'subscription_heartbeat': 1000,  # default value in 'ros_gz_bridge.cpp'
            'expand_gz_topic_names': True,
            'bridge_names': [front_lidar_channel, front_imu_channel],
            # The behaviour of the Gazebo Sim plugin for a 3D ring LiDAR is to prepend to the given topic the string
            # '/points', i.e, <lidar_sim_topic> + '/points'.
            # There is no way to change this behaviour of the Gazebo Sim plugin for a 3D ring LiDAR.
            # Yes, in this scenario the Gazebo Sim plugin for the 3D LiDAR will publish the topics:
            # - '<robot_namespace>/front_lidar/points' (2D laser scan), this is the 'front_lidar_sim_topic'
            # - '<robot_namespace>/front_lidar/points/points' (3D point cloud), this is the
            #   'front_lidar_sim_topic' + '/points'
            # We instruct the bridge to transfer the topic '<robot_namespace>/front_lidar/points/points' to ROS, and
            # then we remap it to '<robot_namespace>/front_lidar/points', so the user gets the topic he expects.
            f'bridges.{front_lidar_channel}.ros_topic_name': front_lidar_sim_topic + '/points',
            f'bridges.{front_lidar_channel}.gz_topic_name': front_lidar_sim_topic + '/points',
            f'bridges.{front_lidar_channel}.ros_type_name': 'sensor_msgs/msg/PointCloud2',
            f'bridges.{front_lidar_channel}.gz_type_name': 'gz.msgs.PointCloudPacked',
            f'bridges.{front_lidar_channel}.direction': 'GZ_TO_ROS',
            f'bridges.{front_lidar_channel}.qos_profile': 'SENSOR_DATA',
            # Lazy subscription policy
            # Many bridges default to 'lazy: true' to avoid spinning up internal publishers/subscribers unless a
            # real client appears on the opposite side.
            # lazy = true  -> the bridge activates only when at least one ROS-side or GZ-side subscriber/publisher
            #                 exists, saving CPU/bandwidth.
            # lazy = false -> the bridge stays permanently connected, forwarding every message even if no node is
            #                 currently listening.
            # For /clock in simulation we normally force 'lazy: false' so the time source is always available as
            # soon as any ROS node starts.  For secondary topics,
            f'bridges.{front_lidar_channel}.lazy': True,
            # The behaviour of the gazebo plugin for 3d lidar is to prepend to the given topic the string '/points'.
            f'bridges.{front_imu_channel}.ros_topic_name': front_imu_sim_topic,
            f'bridges.{front_imu_channel}.gz_topic_name': front_imu_sim_topic,
            f'bridges.{front_imu_channel}.ros_type_name': 'sensor_msgs/msg/Imu',
            f'bridges.{front_imu_channel}.gz_type_name': 'gz.msgs.IMU',
            f'bridges.{front_imu_channel}.direction': 'GZ_TO_ROS',
            f'bridges.{front_imu_channel}.qos_profile': 'SENSOR_DATA',
            # Lazy subscription policy
            # Many bridges default to 'lazy: true' to avoid spinning up internal publishers/subscribers unless a
            # real client appears on the opposite side.
            # lazy = true  -> the bridge activates only when at least one ROS-side or GZ-side subscriber/publisher
            #                 exists, saving CPU/bandwidth.
            # lazy = false -> the bridge stays permanently connected, forwarding every message even if no node is
            #                 currently listening.
            # For /clock in simulation we normally force 'lazy: false' so the time source is always available as
            # soon as any ROS node starts.  For secondary topics,
            f'bridges.{front_imu_channel}.lazy': True,
        }
    ]

    # By using remappings, we can remap the topic with the pointcloud published by the Gazebo Sim 3D LiDAR plugin and
    # transfered by the bridge into ROS, using the extra suffix '/points', to the topic required by the user, without
    # the extra '/points' suffix.

    bridge_remappings = [(front_lidar_sim_topic + '/points', front_lidar_sim_topic)]

    return (
        [OpaqueFunction(function=create_composable_bridge, args=[bridge_name, bridge_params, bridge_remappings])]
        if use_composition
        else [OpaqueFunction(function=create_standard_bridge, args=[bridge_name, bridge_params, bridge_remappings])]
    )


def launch_real_sensors(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    return []


# ----------------------------------------------------------------------------------------------------------------------
# Non opaque functions.
# ----------------------------------------------------------------------------------------------------------------------

# NOTE: ParameterFile vs. Jinja in this launch file
#
# You might be familiar with the ParameterFile action in ROS 2. It allows you to substitute variables in a YAML file at
# launch time using standard substitution syntax like `$(var some_value)`. This is very convenient when parameters
# change between simulation and real deployments.
#
# Jinja, on the other hand, is a general-purpose templating system. It is not ROS-specific, but allows us to define
# placeholders directly in the YAML that can be rendered into final values before the file is even passed to any ROS or
# Gazebo process.
#
# Why use Jinja here instead of ParameterFile? Simplicity and clarity.
#
# Example: the `use_sim_time` flag. If we used ParameterFile and left '$(var use_sim_time)' in the YAML, ROS 2 nodes
# launched via ROS launch would resolve it correctly. However, Gazebo does NOT process ROS 2 substitutions,  it will try
# to read the file as plain YAML. That means we must pass Gazebo a YAML where 'use_sim_time' is already resolved to
# 'true' or 'false'. If not, Gazebo will fail to parse it.
#
# Technically, we could duplicate the file and have one version for simulation ('true') and one for real ('false'), but
# this would create redundant configuration and maintenance overhead.
#
# Another case is the joint name. This often appears hardcoded in controller configs, but if the URDF changes, the YAML
# must also change. Ideally, this would be substituted at launch time from a LaunchConfiguration argument.
# But again, because Gazebo can't resolve ROS 2 substitutions, this becomes cumbersome unless we add extra logic to
# read from the URDF and propagate it into the YAML.
#
# With Jinja, we can place clear placeholders in the YAML (e.g., '{{ robot_joint_name }}') that are rendered to their
# final values before Gazebo or any node sees the file. The workflow is straightforward:
# 1. YAML with placeholders.
# 2. Render with Jinja in the launch file.
# 3. Save the resolved YAML to disk.
# 4. Pass it to Gazebo or ROS nodes.
#
# This ensures:
#  - Single source of truth for values like joint limits or 'use_sim_time'.
#  - No duplicated files for sim/real.
#  - Easy visibility of what is dynamic (placeholders are explicit).
#  - A simpler and more predictable launch process.
#
# ┌───────────────────────────────────────────────────────────────────────┐
# │                     CONFIGURATION FLOW (AT A GLANCE)                  │
# └───────────────────────────────────────────────────────────────────────┘
#        ParameterFile path                      Jinja path
#       (works for ROS nodes)               (works for ROS & Gazebo)
#
#   YAML with $(var ...)                YAML with {{ ... }} placeholders
#             │                                        │
#   ROS Launch resolves subs.           Launch renders with Jinja (Python)
#             │                                        │
#   Node receives final params          Write resolved YAML to disk
#             │                                        │
#   OK:  Works for ROS nodes               Pass file to Gazebo or ROS nodes
#   NOK: Gazebo does NOT resolve subs.                │
#                                              OK: Works for both
