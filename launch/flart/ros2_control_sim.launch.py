import os
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from jinja2 import Template
from launch_ros.actions import Node

from eut_robotics_description.tools import make_robot_namespace, make_robot_prefix
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
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
        LogInfo(
            msg=[
                "Launching ros2_control in simulation mode for the robot '",
                robot_namespace,
                "' (namespace: ",
                namespace,
                ', robot_name: ',
                robot_name,
                ')',
            ]
        ),
        LogInfo(msg=['robot_prefix: ', robot_prefix]),
        LogInfo(msg=['odometry_frame: ', LaunchConfiguration('odometry_frame')]),
        OpaqueFunction(function=launch_controllers_sim),
    ]

    return LaunchDescription(ldes)


def launch_controllers_sim(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    ldes: list[LaunchDescriptionEntity] = []

    robot_prefix = LaunchConfiguration('robot_prefix').perform(ctx)

    # Read the configuration file for the flart robot, so we can read the values we need to render the controllers.
    pkg_dir = get_package_share_directory('eut_robotics_description')

    # Load Jinja template for simulated controllers
    controllers_sim_template = os.path.join(pkg_dir, 'config', 'robots', 'flart', 'controllers_sim.j2')

    if not Path(controllers_sim_template).is_file():
        raise FileNotFoundError(f"Controllers template not found: '{controllers_sim_template}'")

    with open(controllers_sim_template) as f:
        template = Template(f.read())

    flart_cfg_file = os.path.join(pkg_dir, 'config', 'robots', 'flart', 'description.yaml')

    if not Path(flart_cfg_file).is_file():
        raise FileNotFoundError(f"Robot config file not found: '{flart_cfg_file}'")

    with open(flart_cfg_file) as f:
        flart_cfg: dict[str, Any] = yaml.safe_load(f)

    if not flart_cfg:
        raise ValueError(f"Robot config file '{flart_cfg_file}' is empty or could not be parsed into a dictionary.")

    # The mecanum drive controller needs a parameter called 'sum_of_robot_center_projection_on_X_Y_axis', which
    # depends on the geometry of the robot base.
    # In this launch file we refer to that parameter, 'sum_of_robot_center_projection_on_X_Y_axis', as 'lx_plus_ly'.
    try:
        base_cfg = flart_cfg['base']
        base_body_cfg = base_cfg['body']
        lx = 0.5 * base_body_cfg['wheelbase']
        ly = 0.5 * base_body_cfg['track_width']
        wheel_radius = 0.5 * base_cfg['wheel']['shape']['size']['x']
    except KeyError as e:
        raise KeyError(f"Missing required key '{e.args[0]}' in robot config file: '{flart_cfg_file}'") from e

    render_context = {
        'front_left_wheel_joint_name': f'{robot_prefix}front_left_wheel_joint',
        'front_right_wheel_joint_name': f'{robot_prefix}front_right_wheel_joint',
        'rear_left_wheel_joint_name': f'{robot_prefix}rear_left_wheel_joint',
        'rear_right_wheel_joint_name': f'{robot_prefix}rear_right_wheel_joint',
        # A wheel is a cylinder with rotation axis along the local Y axis, which means that the radius is half of the
        # size along the x-axis or the z-axis, since size_x and size_z are equal for a cylinder, a denote the diameter
        # of the circular face of the cylinder.
        'wheels_radius': wheel_radius,
        'lx_plus_ly': lx + ly,  # Substitution for 'sum_of_robot_center_projection_on_X_Y_axis'
        'base_frame_name': f'{robot_prefix}base_footprint_link',
        'odom_frame_name': robot_prefix + LaunchConfiguration('odometry_frame').perform(ctx),
        'fork_joint_name': f'{robot_prefix}fork_root_joint',
    }

    rendered = template.render(render_context)

    # Write rendered YAML into ROS_HOME with a per-robot unique name to avoid permission issues and collisions.
    ros_home = os.environ.get('ROS_HOME', os.path.expanduser('~/.ros'))
    os.makedirs(ros_home, exist_ok=True)
    controllers_sim_file = os.path.join(ros_home, f'{robot_prefix}controllers_sim.yaml')

    with open(controllers_sim_file, 'w') as f:
        f.write(rendered)

    robot_namespace = LaunchConfiguration('robot_namespace').perform(ctx).strip()
    fq_controller_manager = robot_namespace + '/controller_manager'

    # Spawners (jsb -> mdc -> fork)
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

    ldes.extend([joint_state_broadcaster, start_mdc_after_jsb, start_fork_controller_after_mdc])
    return ldes
