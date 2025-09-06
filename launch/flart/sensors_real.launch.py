from eut_robotics_description.tools import make_robot_namespace
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution


def generate_launch_description():
    # The 'prefix' is similar to the 'namespace', it is a 'flatenized' version of the namespace, i.e., it uses the
    # character '_' as a separator instead of the character '/'.
    # The 'namespace' and the 'robot_name' are passed to the xacro file to make clear to the user that a 'robot_name' is
    # required to distinguish between multiple robots, and the 'namespace' is optional.
    # The 'robot_namespace' is the concatenation of the 'namespace' and the 'robot_name', using the character '/' a
    # separtor. The 'robot_namespace' is prepended to nodes and topics when required.
    # The 'robot_namespace' is computed in this launch file and also inside the xacro file, using the same rules.

    # namespace=''          -> robot_namespace = robot_name
    #                          prefix = ''
    # namespace='/'         -> robot_namespace = '/' + robot_name
    #                          prefix = ''
    # namespace='ns'        -> robot_namespace = 'ns' + '/' + robot_name
    #                          prefix = 'ns' + '_'
    # namespace='ns/'       -> robot_namespace = 'ns' + '/' + robot_name
    #                          prefix = 'ns' + '_'
    # namespace='/ns/'      -> robot_namespace = '/ns' + '/' + robot_name
    #                          prefix = 'ns' + '_'
    # namespace='/ns1/ns2'  -> robot_namespace = '/ns1/ns2' + '/' + robot_name
    #                          prefix = 'ns1_ns2' + '_'
    # namespace='/ns1/ns2/' -> robot_namespace = '/ns1/ns2' + '/' + robot_name
    #                          prefix = 'ns1_ns2' + '_'
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    robot_namespace = make_robot_namespace(namespace, robot_name)

    # ldes -> (l)aunch (d)escription (e)ntitie(s)

    ldes = [
        DeclareLaunchArgument('robot_name', default_value='flart', description='The unique name for the robot'),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for all resources'),
        DeclareLaunchArgument(
            'use_front_lidar', default_value='True', description='Whether to launch the front lidar in real mode'
        ),
        DeclareLaunchArgument(
            'use_front_imu', default_value='True', description='Whether to launch the front imu in real mode'
        ),
        SetLaunchConfiguration('robot_namespace', robot_namespace),
        # When working in simulation, the topics used by the sensors installed in the 'flart' robot are defined in the
        # xacro macro files for these sensors.
        # Consequently, the topics used in simulation are fixed and cannot be changed from the launch file.
        # This is a convention we have adopted to standardize the topic names for the sensors used in our robots.
        # These topics do really make sense since they have the form:
        # <robot_namespace>/<sensor_name>/<base_topic_for_sensor>
        # Therefore, the topics in real conditions, do must match the topics used in simulation.
        SetLaunchConfiguration('front_lidar_topic', [robot_namespace, '/front_lidar/scan/points']),
        SetLaunchConfiguration('front_imu_topic', [robot_namespace, '/front_imu/data']),
        LogInfo(msg=['robot_name: ', robot_name]),
        LogInfo(msg=['namespace: ', namespace]),
        LogInfo(msg=['use_front_lidar: ', LaunchConfiguration('use_front_lidar')]),
        LogInfo(msg=['use_front_imu: ', LaunchConfiguration('use_front_imu')]),
        LogInfo(msg=['front_lidar_topic: ', LaunchConfiguration('front_lidar_topic')]),
        LogInfo(msg=['front_imu_topic: ', LaunchConfiguration('front_imu_topic')]),
        OpaqueFunction(function=launch_sensors),
    ]

    return LaunchDescription(ldes)


# ----------------------------------------------------------------------------------------------------------------------


# Opaque functions.


def launch_sensors(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    ldes: list[LaunchDescriptionEntity] = []

    use_front_lidar = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('use_front_lidar'), bool), bool
    )
    use_front_imu = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('use_front_imu'), bool), bool
    )

    # If none enabled, do nothing
    if not any([use_front_lidar, use_front_imu]):
        return []

    # Placeholders for real drivers. Replace logs with actual Node(...) definitions when wiring hardware drivers.
    if use_front_lidar:
        pass

    if use_front_imu:
        pass

    return ldes
