import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from eut_robotics_description.tools import make_robot_namespace, make_robot_prefix
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution


def generate_launch_description():
    # The 'namespace' and the 'robot_name' are passed to the xacro file to make clear to the user that a 'robot_name' is
    # required to distinguish between multiple robots, and the 'namespace' is optional.
    # The 'robot_namespace' is prepended to nodes and topics when required.
    # The 'robot_namespace' is computed in this launch file and also inside the xacro file, using the same rules.

    # The 'robot_prefix' is prepended to the links and joints defined in the xacro file.
    # The 'robot_prefix' is also computed inside the xacro file.

    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    robot_namespace = make_robot_namespace(namespace, robot_name)
    robot_prefix = make_robot_prefix(namespace, robot_name)

    # ldes => (l)aunch (d)escription (e)ntitie(s)

    ldes = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            choices=['True', 'true', 'False', 'false'],
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument('robot_name', default_value='flart', description='The unique name for the robot'),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for all resources'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Odometry frame name of the robot'),
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
        SetLaunchConfiguration('robot_namespace', robot_namespace),
        SetLaunchConfiguration('robot_prefix', robot_prefix),
        LogInfo(
            msg=[
                "Launching description for the robot '",
                robot_namespace,
                "' (namespace: ",
                namespace,
                ', robot_name: ',
                robot_name,
                ')',
            ]
        ),
        LogInfo(msg=['robot_prefix: ', robot_prefix]),
        LogInfo(msg=['Simulation config file: ', LaunchConfiguration('sim_cfg_file')]),
        LogInfo(msg=['RSP publish frequency: ', LaunchConfiguration('rsp_publish_frequency')]),
        OpaqueFunction(function=launch_robot_state_publisher),
    ]

    return LaunchDescription(ldes)


# ----------------------------------------------------------------------------------------------------------------------

# Opaque functions.


def launch_robot_state_publisher(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    use_sim_time = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('use_sim_time'), bool), bool
    )

    if use_sim_time:
        # If we are in simulation mode, get the simulation configuration file provided by the user (be aware, the user
        # could pass an empty string), or the default one.
        sim_cfg_file = LaunchConfiguration('sim_cfg_file').perform(ctx)

        if not isinstance(sim_cfg_file, str):
            raise TypeError(f"Expected 'sim_cfg_file' to be of type 'str', but got '{type(sim_cfg_file)}'")

        if not sim_cfg_file:
            raise ValueError('The provided simulation configuration file is an empty string')

        # If the user provided a simulation configuration file, check it exists.
        # If the user provided and empty string, it means no simulation configuration file provided, so no file
        # existence check is needed, but no sensor will be simulated.
        if not Path(sim_cfg_file).is_file():
            raise FileNotFoundError(f"The provided simulation configuration file does not exist: '{sim_cfg_file}'")
    else:
        # If we are not in simulation mode, do not use any simulation configuration file; it is no needed.
        sim_cfg_file = ''

    robot_description_param = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'),
                ' ',
                os.path.join(
                    get_package_share_directory('eut_robotics_description'),
                    'urdf',
                    'robots',
                    'flart',
                    'description.xacro',
                ),
                ' robot_name:=',
                LaunchConfiguration('robot_name'),
                ' namespace:=',
                LaunchConfiguration('namespace'),
                ' odom_frame:=',
                LaunchConfiguration('odom_frame'),
                # User can disable the use of visual or collision meshes, for example to improve performance when
                # simulating the robot in Gazebo.
                ' use_visual_meshes:=',
                LaunchConfiguration('use_visual_meshes'),
                ' use_collision_meshes:=',
                LaunchConfiguration('use_collision_meshes'),
                # Depending on whether the robot is running in simulation mode or in real mode, some xacro sections
                # will be included or excluded.
                ' use_sim_mode:=',
                LaunchConfiguration('use_sim_time'),
                ' sim_cfg_file:=',
                sim_cfg_file,
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
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description_param,
                    #'frame_prefix': "DO NOT USE IT, ROBOT_PREFIX IS COMPUTED AND USED IN THE XACRO FILE"
                    'publish_frequency': ParameterValue(LaunchConfiguration('rsp_publish_frequency'), value_type=float),
                }
            ],
            output='screen',
        )
    ]
