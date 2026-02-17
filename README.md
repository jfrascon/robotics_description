# robotics_description

Reusable URDF/Xacro assets for ROS 2.

This package is a shared component library to build robot descriptions across projects. The generic part provides reusable macro blocks for consistent naming (`namespace`/`prefix`), visual and collision definitions, inertial definitions, and common mechanical elements such as wheels and forklift forks. Sensor URDF/Xacro files are grouped by type under `urdf/sensors` (`lidars`, `imus`, `cameras`), and that is the place where specific sensor models are incorporated over time.

Current specific sensor models are: `robosense_airy`, `robosense_helios_16p`, `um7`, and `realsense_d435`. Associated meshes are stored under `meshes/`, and example simulation parameters are provided under `config/sensors/`.

Example integration of `robosense_helios_16p` (some properties are scalar, others are space-separated sequences like `"min max"`):

```xml
<xacro:property name="robot_prefix"                   value="my_robot_"/>
<xacro:property name="robot_namespace"                value="/test/my_robot"/>
<xacro:property name="use_sim_mode"                   value="True"/>
<xacro:property name="joint_parent_fr_root_fr"        value="0.0 0.0 0.12 0.0 0.0 0.0"/>

<xacro:property name="front_lidar_use_visual"         value="True"/>
<xacro:property name="front_lidar_use_collision"      value="True"/>
<xacro:property name="front_lidar_use_inertial"       value="True"/>
<xacro:property name="front_lidar_use_v_mesh"         value="False"/>
<xacro:property name="front_lidar_use_low_res_v_mesh" value="True"/>
<xacro:property name="front_lidar_color"              value="0.1 0.1 0.1 1.0"/>
<xacro:property name="front_lidar_use_c_mesh"         value="False"/>
<xacro:property name="front_lidar_use_low_res_c_mesh" value="True"/>

<xacro:property name="front_lidar_sim_enabled"        value="True"/>
<xacro:property name="front_lidar_sim_always_on"      value="True"/>
<xacro:property name="front_lidar_sim_visualize"      value="False"/>
<xacro:property name="front_lidar_sim_update_rate"    value="10"/>
<xacro:property name="front_lidar_sim_hor_fov_deg"    value="-180 180"/>
<xacro:property name="front_lidar_sim_hor_res_deg"    value="0.2"/>
<xacro:property name="front_lidar_sim_ver_fov_deg"    value="-15 15"/>
<xacro:property name="front_lidar_sim_ver_res_deg"    value="2"/>
<xacro:property name="front_lidar_sim_dist_span"      value="0.2 150"/>
<xacro:property name="front_lidar_sim_gaussian_noise" value="0 0.01"/>

<xacro:robosense_helios_16p name="front_lidar"
                            prefix="${robot_prefix}"
                            namespace="${robot_namespace}"
                            parent_frame="${robot_prefix}front_platform_link"
                            use_visual="${front_lidar_use_visual}"
                            use_collision="${front_lidar_use_collision}"
                            use_inertial="${front_lidar_use_inertial}"
                            use_v_mesh="${front_lidar_use_v_mesh}"
                            use_low_res_v_mesh="${front_lidar_use_low_res_v_mesh}"
                            color="${front_lidar_color}"
                            use_c_mesh="${front_lidar_use_c_mesh}"
                            use_low_res_c_mesh="${front_lidar_use_low_res_c_mesh}"
                            joint_parent_fr_root_fr="${joint_parent_fr_root_fr}"
                            use_sim="${use_sim_mode}"
                            sim_always_on="${front_lidar_sim_always_on}"
                            sim_visualize="${front_lidar_sim_visualize}"
                            sim_update_rate="${front_lidar_sim_update_rate}"
                            sim_hor_fov_deg="${front_lidar_sim_hor_fov_deg}"
                            sim_hor_res_deg="${front_lidar_sim_hor_res_deg}"
                            sim_ver_fov_deg="${front_lidar_sim_ver_fov_deg}"
                            sim_ver_res_deg="${front_lidar_sim_ver_res_deg}"
                            sim_dist_span="${front_lidar_sim_dist_span}"
                            sim_gaussian_noise="${front_lidar_sim_gaussian_noise}"/>
```

Validation is covered by `test/test_xacro.py`, which checks Xacro-to-URDF generation and mesh references.
