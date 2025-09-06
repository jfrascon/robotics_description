from eut_robotics_description.tools import make_robot_namespace, make_robot_prefix
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Compute namespace-derived values (robot_namespace and robot_prefix)
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    robot_namespace = make_robot_namespace(namespace, robot_name)
    robot_prefix = make_robot_prefix(namespace, robot_name)

    ldes = [
        DeclareLaunchArgument('robot_name', default_value='flart', description='The unique name for the robot'),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for all resources'),
        DeclareLaunchArgument(
            'odometry_frame', default_value='odom', description='The odometry frame used in the project'
        ),
        SetLaunchConfiguration('robot_namespace', robot_namespace),
        SetLaunchConfiguration('robot_prefix', robot_prefix),
        LogInfo(msg=['[REAL r2c] robot_name: ', robot_name]),
        LogInfo(msg=['[REAL r2c] namespace: ', namespace]),
        LogInfo(msg=['[REAL r2c] robot_namespace: ', robot_namespace]),
        LogInfo(msg=['[REAL r2c] robot_prefix: ', robot_prefix]),
        OpaqueFunction(function=launch_ros2_control_real),
    ]

    return LaunchDescription(ldes)


def launch_ros2_control_real(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # Skeleton: add controller_manager spawners for real hardware when available.
    return []
