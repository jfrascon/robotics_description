# robotics_description

This package provides a collection of xacro macros to build robot descriptions across projects.
The xacro macros are located under the `urdf/` directory, and are organized into categories, according to their purpose:

- `common/`: very generic macros. They do not define any specific robot component, but rather common operations such as **compute inertia for basic shapes**, **expand a topic with a namespace**, **create visual/collision/inertial elements for a <link>**, etc.
- `extras/`: to be honest, components that are no sensors, like a `fork`, for example. Other components will be added here over time.
- `gz_system_plugins/`: macros to set up Gazebo system plugins for simulation.
- `sensors/`: macros for specific sensors. There are subdirectories for each sensor type (`lidars/`, `imus/`, `cameras/`), and specific sensor models are added here over time. Thera are also generic macros for each sensor type, reusable across specific sensor models of the same type.
- `wheels/`: macros for different types of wheels, such as `regular wheels`, `steerable wheels`, `caster wheels`, etc.

If a specific sensor model needs a mesh, it is stored under the `meshes/` directory, inside a subdirectory for the sensor type (for example, `meshes/lidars/robosense_helios_16/`).

There is a `doc/` directory with documentation that could be useful for users, like `how_to_compute_inertia_w_meshlab.md`. It is expected that this documentation will grow over time as more resources are added to the package.

The [`plugin_referenced_frame_pose_publisher`](doc/plugin_referenced_frame_pose_publisher.md) documentation explains the frame convention, offset semantics, and output interpretation for the Gazebo system plugin macro that wraps `gz::sim::systems::OdometryPublisher`. Read it before using that macro, because the generated messages use Gazebo odometry message types even when the published pose is not odometry in the semantic sense.

When a xacro is too specific, like the [`robosense_helios_16_macro.xacro`](urdf/sensors/lidars/robosense_helios_16_macro.xacro) for example, it already contains the Gazebo plugin within it. Just check out the xacro macro parameters to see how to configure the macro in general and its Gazebo plugin in particular. Great care has been taken into make the interface of the xacro macros as consistent as possible across different sensor types and models, so the mental burden of using different macros for different sensors is minimized.

However, some general macros, like the [`fork_simple_macro.xacro`](urdf/extras/fork_simple/fork_simple_macro.xacro) or the [`steerable_wheel_macro.xacro`](urdf/wheels/steerable_wheel_macro.xacro) for example, do not have any Gazebo plugin within them, because there is not a specific Gazebo plugin in the [SDF specification](https://sdformat.org/spec/) for these items per se. Instead, this kind of components can be associated to different Gazebo system plugins, depending on the specific simulation setup and requirements. For example, the `fork_simple_macro.xacro` defines a simple fork component that depending on the robot description where it is used, can be associated to a `JointPositionController` Gazebo plugin to control the position of the fork, either by using messages of type `actuator_msgs/msg/Actuators` or messages of type `std_msgs/msg/Float64`, either controlled with a `PID` controller or with `velocity commands`, etc.,

You can identify the components that do not have a Gazebo plugin within them, in other words, that are linked to Gazebo system plugins, if you go to their folder an inside a subdirectory called `generic_macros` you seer xacro macro wrappers around Gazebo system plugin.
For example, in the case of the `fork_simple_macro.xacro`, you can find:
- [`plugin_fork_joint_am_pos_pid_macro.xacro`](urdf/extras/fork_simple/generic_macros/plugin_fork_joint_am_pos_pid_macro.xacro)
- [`plugin_fork_joint_am_pos_velcmd_macro.xacro`](urdf/extras/fork_simple/generic_macros/plugin_fork_joint_am_pos_velcmd_macro.xacro)
- [`plugin_fork_joint_f64_pos_pid_macro.xacro`](urdf/extras/fork_simple/generic_macros/plugin_fork_joint_f64_pos_pid_macro.xacro)
- [`plugin_fork_joint_f64_pos_velcmd_macro.xacro`](urdf/extras/fork_simple/generic_macros/plugin_fork_joint_f64_pos_velcmd_macro.xacro)

Many specific sensor macros already embed their Gazebo plugin. For that reason, their interfaces usually expose several simulation parameters, such as topics, update rates, noise models, field-of-view limits, and similar plugin-specific settings. You can always pass those parameters one by one when invoking the macro, which keeps the call site explicit and gives you full control.

In practice, however, these simulation parameters are often maintained in a YAML file. To make that workflow easier, this package also provides helper macros whose names start with `set_props_`. These helpers take a simulation configuration dictionary, usually loaded from one of those YAML files, and expand it into individual Xacro properties with the exact names expected by the corresponding macro. This means you can choose the style that better fits your project: pass simulation arguments explicitly, or load them from YAML and bridge both representations through a `set_props_...` macro.

Under the folder `config` you can find example simulation configuration files for the Gazebo plugins associated to the different components. These files are not consumed automatically by the Xacro macros, but they show the kind of simulation dictionaries that can later be loaded and adapted to your own robot configuration before passing them, directly or through a `set_props_...` helper, to the corresponding macro.

Example integration of `robosense_helios_16` following the pattern used in `robot_forklift_simple_3sw/urdf/v1.xacro`:

```xml
# Load the simulation YAML once and extract the section for this lidar.
<xacro:property name="sim_cfg"
                value="${xacro.load_yaml(sim_file)}"/>
# Key `top_lidar` is expected to be present in the YAML, but if not, an empty
# dictionary is used as default value to avoid errors.
<xacro:property name="top_lidar_sim_cfg"
                value="${sim_cfg.get('top_lidar', dict())}"/>

# Expand the YAML dictionary into individual Xacro properties such as:
# - top_lidar_sim_enabled
# - top_lidar_sim_update_rate
# - top_lidar_sim_hor_fov_deg
# ...
# - top_lidar_sim_base_topic
<xacro:set_props_plugin_lidar_3d_ring prop_prefix="top_lidar_sim"
                                      sim_cfg="${top_lidar_sim_cfg}"/>

# Instantiate the sensor macro. Visual/collision/inertial arguments can still
# be set explicitly at the call site, while simulation arguments come from the
# properties produced by set_props_plugin_lidar_3d_ring.
<xacro:robosense_helios_16
    name="top_lidar"
    prefix="${robot_prefix}"
    parent_frame="${robot_prefix}top_platform_link"
    use_visual="${top_lidar_use_visual}"
    use_collision="${top_lidar_use_collision}"
    use_inertial="${top_lidar_use_inertial}"
    use_v_mesh="${top_lidar_use_v_mesh}"
    v_mesh_use_low_res="${top_lidar_v_mesh_use_low_res}"
    color="${top_lidar_color}"
    use_c_mesh="${top_lidar_use_c_mesh}"
    c_mesh_use_low_res="${top_lidar_c_mesh_use_low_res}"
    joint_parent_fr_root_fr="0.0 0.0 ${top_platform_size_z} 0.0 0.0 0.0"
    sim_enabled="${top_lidar_sim_enabled and use_sim_mode}"
    sim_always_on="${top_lidar_sim_always_on}"
    sim_visualize="${top_lidar_sim_visualize}"
    sim_update_rate="${top_lidar_sim_update_rate}"
    sim_hor_fov_deg="${top_lidar_sim_hor_fov_deg}"
    sim_hor_res_deg="${top_lidar_sim_hor_res_deg}"
    sim_ver_fov_deg="${top_lidar_sim_ver_fov_deg}"
    sim_ver_res_deg="${top_lidar_sim_ver_res_deg}"
    sim_dist_span="${top_lidar_sim_dist_span}"
    sim_gaussian_noise="${top_lidar_sim_gaussian_noise}"
    namespace="${robot_namespace}"
    sim_base_topic="${top_lidar_sim_base_topic}"/>
```

If you prefer, you can also define the simulation properties one by one instead of loading them from YAML and using a `set_props_...` helper:

```xml
# Define only the simulation properties you want to control explicitly.
<xacro:property name="top_lidar_sim_enabled"     value="True"/>
<xacro:property name="top_lidar_sim_always_on"   value="True"/>
<xacro:property name="top_lidar_sim_visualize"   value="False"/>
<xacro:property name="top_lidar_sim_update_rate" value="10.0"/>
# ... other simulation properties ...
<xacro:property name="top_lidar_sim_base_topic"  value="sensors/top_lidar"/>

# Instantiate the same macro and inject those properties directly.
# The remaining simulation arguments follow the same pattern and can be added
# one by one if needed.
<xacro:robosense_helios_16
    name="top_lidar"
    prefix="${robot_prefix}"
    parent_frame="${robot_prefix}top_platform_link"
    use_visual="${top_lidar_use_visual}"
    use_collision="${top_lidar_use_collision}"
    use_inertial="${top_lidar_use_inertial}"
    use_v_mesh="${top_lidar_use_v_mesh}"
    v_mesh_use_low_res="${top_lidar_v_mesh_use_low_res}"
    color="${top_lidar_color}"
    use_c_mesh="${top_lidar_use_c_mesh}"
    c_mesh_use_low_res="${top_lidar_c_mesh_use_low_res}"
    joint_parent_fr_root_fr="0.0 0.0 ${top_platform_size_z} 0.0 0.0 0.0"
    sim_enabled="${top_lidar_sim_enabled and use_sim_mode}"
    sim_always_on="${top_lidar_sim_always_on}"
    sim_visualize="${top_lidar_sim_visualize}"
    sim_update_rate="${top_lidar_sim_update_rate}"
    ...
    sim_base_topic="${top_lidar_sim_base_topic}"/>
```

## Tests

The main validation entry point for this package is:
- `test/test_xacro.py`

This test suite exists because most files in `robotics_description` are reusable macros, not complete robot descriptions. To validate them, the package includes small test cases under `test/test_xacros/` that instantiate those macros with valid arguments and example simulation settings.

For each collected test Xacro, the test suite performs:
- Xacro expansion to make sure the file can be rendered into URDF.
- `check_urdf` validation on the generated URDF.
- mesh validation to make sure every `package://` mesh reference points to an
  existing file.

To run the tests from the workspace root:

```bash
colcon test --merge-install --packages-select robotics_description
colcon test-result --all --verbose
```

For direct debugging with full output:

```bash
python3 -m pytest -s -v src/0_deps/robotics_description/test/test_xacro.py
```

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
