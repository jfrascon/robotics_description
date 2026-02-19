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

## Xacro guard pattern

Use this pattern in macros to enforce input constraints at generation time:

```xml
<xacro:if value="${condition_here}">
  ${xacro.fatal("error_message_here")}
</xacro:if>
```

Important XML note:
- Inside XML attributes (such as `value="..."`), the `<` symbol must be escaped as `&lt;`.
- For example, write `&lt;=` instead of `<=`, otherwise XML parsing may fail before `xacro.fatal` is evaluated.
- The `>` symbol is usually accepted as-is in XML attributes, but you may still use `&gt;` (or `&gt;=`) for consistency and readability.

This form is incorrect and may fail during XML parsing:

```xml
<xacro:if value="${mass <= 0.0}">
  ${xacro.fatal("mass must be > 0")}
</xacro:if>
```

Examples:

```xml
<xacro:if value="${mass &lt;= 0.0}">
  ${xacro.fatal("mass must be > 0")}
</xacro:if>

<xacro:if value="${len(shape_seq) &lt; 3}">
  ${xacro.fatal("shape must contain at least 3 elements")}
</xacro:if>

<xacro:if value="${effort &gt; 1000.0}">
  ${xacro.fatal("effort must be <= 1000")}
</xacro:if>
```

Keep this section as the canonical reference when adding validation checks to new Xacro files.

## Mesh generation scripts

Some objects include Python generators under `meshes/` (for example in `meshes/extras/fork_simple`) that export STL files with a simplified body representation. These simplified meshes are intended to be lightweight and sufficient for simulation workflows (visual/collision/inertial approximation), while keeping geometry generation reproducible from script parameters.

For these generators, installing `cadquery` in the local user environment is recommended:

```bash
python3 -m pip install --user cadquery
```
