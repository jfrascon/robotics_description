import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    # ldes => (l)aunch (d)escription (e)ntitie(s)
    ldes = [
        # Common launch arguments.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            choices=['True', 'true', 'False', 'false'],
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument('robot_name', default_value='flart', description='The unique name for the robot'),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for all resources'),
        # Launch arguments for the robot description.
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
            description='Path to the simulation configuration file (default: flart/simulation_default.yaml)',
        ),
        DeclareLaunchArgument(
            'rsp_publish_frequency',
            default_value='20.0',
            description='Frequency of publication for robot_state_publisher',
        ),
        # 'use_<sensor> are share between the robot description and the sensors launch file.
        DeclareLaunchArgument(
            'use_front_lidar',
            default_value='True',
            choices=['True', 'true', 'False', 'false'],
            description='Whether to use the front lidar sensor',
        ),
        DeclareLaunchArgument(
            'use_front_imu',
            default_value='True',
            choices=['True', 'true', 'False', 'false'],
            description='Whether to use the front imu sensor',
        ),
        #
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
        DeclareLaunchArgument('odom_frame', default_value='odom', description='The odometry frame used in the project'),
        # Launch de robot description, both in simulation and real mode.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('eut_robotics_description'), 'launch', 'flart', 'description.launch.py']
                )
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_name': LaunchConfiguration('robot_name'),
                'namespace': LaunchConfiguration('namespace'),
                'use_visual_meshes': LaunchConfiguration('use_visual_meshes'),
                'use_collision_meshes': LaunchConfiguration('use_collision_meshes'),
                # If the robot is running in real mode, do not pass any simulation configuration file, since no sensor
                # can be simulated.
                # If the robot is running in simulation mode, pass the simulation configuration file available, which
                # can be the default if no other is provided by the user, or the one provided by the user.
                # Be aware that if the user provides an empty string as simulation configuration file, no sensor will be
                # simulated.
                'sim_cfg_file': IfElseSubstitution(
                    condition=LaunchConfiguration('use_sim_time'),
                    if_value=LaunchConfiguration('sim_cfg_file'),
                    else_value=TextSubstitution(text=''),
                ),
                # If the robot is running in real mode, no sensor can be simulated.
                # If the robot is running in simulation mode, the simulation of each sensor depends on whether a
                # simulation configuration file is available or not, and on whether the user wants to use that
                # sensor or not.
                'use_front_lidar_sim': IfElseSubstitution(
                    condition=LaunchConfiguration('use_sim_time'),
                    if_value=IfElseSubstitution(
                        condition=EqualsSubstitution(LaunchConfiguration('sim_cfg_file'), ''),
                        if_value=TextSubstitution(text='False'),
                        else_value=LaunchConfiguration('use_front_lidar'),
                    ),
                    else_value=TextSubstitution(text='False'),
                ),
                'use_front_imu_sim': IfElseSubstitution(
                    condition=LaunchConfiguration('use_sim_time'),
                    if_value=IfElseSubstitution(
                        condition=EqualsSubstitution(LaunchConfiguration('sim_cfg_file'), ''),
                        if_value=TextSubstitution(text='False'),
                        else_value=LaunchConfiguration('use_front_imu'),
                    ),
                    else_value=TextSubstitution(text='False'),
                ),
                'rsp_publish_frequency': LaunchConfiguration('rsp_publish_frequency'),
            }.items(),
        ),
        # Launch sensors: simulation vs real, based on 'use_sim_time'.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('eut_robotics_description'), 'launch', 'flart', 'sensors_sim.launch.py']
                )
            ),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'namespace': LaunchConfiguration('namespace'),
                'use_front_lidar': LaunchConfiguration('use_front_lidar'),
                'use_front_imu': LaunchConfiguration('use_front_imu'),
                'use_composition': LaunchConfiguration('use_composition'),
                'create_own_container': LaunchConfiguration('create_own_container'),
                'container_name': LaunchConfiguration('container_name'),
                'respawn_bridge': LaunchConfiguration('respawn_bridge'),
                'bridge_log_level': LaunchConfiguration('bridge_log_level'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_sim_time')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('eut_robotics_description'), 'launch', 'flart', 'sensors_real.launch.py']
                )
            ),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'namespace': LaunchConfiguration('namespace'),
                'use_front_lidar': LaunchConfiguration('use_front_lidar'),
                'use_front_imu': LaunchConfiguration('use_front_imu'),
            }.items(),
            condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
        ),
        # Launch ros2_control nodes, i.e., controllers and controller manager, either in simulation or real mode.
        # ros2 control in simulation mode.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('eut_robotics_description'), 'launch', 'flart', 'ros2_control_sim.launch.py']
                )
            ),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'namespace': LaunchConfiguration('namespace'),
                'odometry_frame': LaunchConfiguration('odometry_frame'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_sim_time')),
        ),
        # ros2 control in real mode.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('eut_robotics_description'), 'launch', 'flart', 'ros2_control_real.launch.py']
                )
            ),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'namespace': LaunchConfiguration('namespace'),
                'odometry_frame': LaunchConfiguration('odometry_frame'),
            }.items(),
            condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
        ),
    ]

    return LaunchDescription(ldes)
