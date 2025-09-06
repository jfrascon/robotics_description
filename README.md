# eut_robotics_description

The goal of this package is to provide a common place where we can store the URDF/Xacro files of the different robots we use in EUT Robotics.
This way we can reuse those models in different projects in an simple way.

The organization of the package is simple, there is a **urdf** folder where you can find folders for different components of the robots, like **bases**, **imus**, **lidars**, **wheels** and **robots**.
There is another folder called **meshes** that contains folders matching the components in the **urdf** folder, where you can find the 3D meshes of the components.
It is very likely that over time we will add more folders to the **urdf** and **meshes** folders, so you can find more components in the future.

Regarding the xacro files under the **urdf** folder, the approach here is to reuse as much a possible these description files. For example, you will find the macro `lidar_3d_ring` defined in the file `lidar_3d_ring_macro.xacro`, that represents the 3D LIDAR with a ring of beams, and can be used to create different types of LIDARs by changing the number of beams and their angles. Internally, the macro `lidar_3d_ring` uses the macro `lidar_3d_ring_plugin`, defined in the file `lidar_3d_ring_plugin_macro.xacro`, which is a plugin for Gazebo. The macro `lidar_2d`, defined in the file `lidar_2d_macro.xacro`, is just an instatation of the macro `lidar_3d_ring`, restricting the number of beams to 1, at 0 degrees, so it can be used as a 2D LIDAR.
The macro `robosense_helios_16p` substitutes some of the parameters of the macro `lidar_3d_ring`, like `mass`, `dimensions`, etc., to create a specific LIDAR model, the **Robosense Helios 16P**.
Other parameters in the macro `robosense_helios_16p` are left to the user to set, like the position of the `root_link`, used to attach the LIDAR to the robot, the number of horizontal beams, etc.

Finally, specific robots are defined in the **robots** folder, where you can find the URDF/Xacro files for the complete robots, including the mobile base, arms, sensors, etc.
For example, the macro `mecanum_rectangular_forklift`, defined in the file `mecanum_rectangular_forklift_macro.xacro` represents a generic forklift that uses a rectangular base, with four mecanum wheels, two at front and two at back, and a fork that can be moved up and down. No sensors are included in this macro, so it can be used as a base for different types of forklifts. Then, the robot `forlift_artisteril`, which is **NOT A MACRO** but a complete robot, defined in the file `forlift_artisteril.xacro`, is an instantiation of the macro `mecanum_rectangular_forklift`, using the proper values for the parameters, like the position of the wheels, the dimensions of the fork, etc. This particular model of a robot can define its own sensors, like a 2D LIDAR, a 3D LIDAR, a camera, etc., and it can be used in different projects, if needed.

At *Eurecat*, we do not manufacture our own robots, but we customize existing ones, or retrofit regular vehicles to create robots, and we can treat all these robots like *our products*, the same way a manufacture would do with its own products. We give them a name, a version, and we can use them in different projects, if needed, like a product catalog. This is the reason why we have the **robots** folder, where you can find the URDF/Xacro files for the complete robots.
For example, we could have files called `husky_agro_trials.xacro` or `vogui_agro_trials.xacro`, defining especifics configurations for the Husky and Vogui robots, respectively, to be used in the trials for Agro projects, so we could reuse those configurations for specific purposes.

Each robot defined in the **robots** folder must have a python launch file called the same name as the robot, with the suffix `.launch.py`, that will be used to launch the robot description. In the robot python launch file, you tipically define one `DeclareArgument` for each argument the associated xacro file for the robot accepts. Sice you are using a python launch file, you have the possibility to code whatever actions you need to do before launching the `robot_state_publlisher` node that will take the expanded robot description as an input and publish it in a topic.

Since the robot `forklift_artisteril` is the first robot added to this package, you could use both the xacro file and the associated launch file as a guide to create new robots in the future. The xacro file is a good example of how to use the macros defined in the **urdf** folder, and the launch file is a good example of what arguments to declare and how to declare them and launch the `robot_state_publisher` node.

**This package has been developed with the idea in mind that multiple robots can be used in the same project, so in order to achive that the source code in the xacro files and the python launch files has been thought and structured in a way that permit this goal.**
For example, the concept of `namespace` is used across the xacro files when needed. Derivated from the concept of the `namespace` we have the concept of the `prefix`, which is basically the namespace where the characters `/` are replaced by `_`, so it can be used as a prefix for the names of the links, joints, sensors, etc., in the URDF/Xacro files. Another concept that has been introduced is the `id` for each component used in a robot, so each macro uses a parameter `id` which will be use to name the component.

For example, in the file `lidar_3d_ring_macro.xacro`, you can find the macro `lidar_3d_ring` with the parameter `id`, used to name the links, joints, sensors, etc., in the URDF/Xacro file. This way, you can have multiple LIDARs in the same robot, and each one will have its own unique name based on the `id` parameter.

```xml
<robot xmlns:xacro="http://ros.org/wiki/xacro" xmlns:gz="http://gazebosim.org/schema">
    <xacro:macro name="lidar_3d_ring"
                 params="id
                         prefix:=''
                         parent_frame
                         mass
                         size_x
                         size_y
                         size_z
                         mesh:=''
                         scale_x:=1.0
                         scale_y:=1.0
                         scale_z:=1.0
                         color:=''
                         ...
```

Then, in the file `robosense_helios_16p_macro.xacro`, you can find the macro `robosense_helios_16p` that uses the macro `lidar_3d_ring`:

```xml
<robot xmlns:xacro="http://ros.org/wiki/xacro" xmlns:gz="http://gazebosim.org/schema">
  <xacro:macro
    name="robosense_helios_16p"
    params="id
            prefix:=''
            parent_frame
            color:='0 0 0 1'
            ...
            *joint_parent_frame_root_frame">

    <xacro:include filename="$(find eut_robotics_description)/urdf/lidars/lidar_3d_ring_macro.xacro"/>
    <xacro:lidar_3d_ring
        id="${id}"
        prefix="${prefix}"
        parent_frame="${parent_frame}"
        mass="0.99"
        size_x="0.1"
        size_y="0.1"
        size_z="0.1005"
        mesh="eut_robotics_description/meshes/lidars/robosense_helios_16p_low_res.stl"
        scale_x="1.0"
        scale_y="1.0"
        scale_z="1.0"
        color="${color}"
        ...
    </xacro:lidar_3d_ring>
  </xacro:macro>
</robot>
```

where you can observe that some parameters in the macro `robosense_helios_16p` do no appear in the macro definition, like `mass`, `size_x`, `size_y`, `size_z`, `mesh`, `scale_x`, etc., since they have been substituted by the proper values in the macro `lidar_3d_ring`.

Then when you use the macro `robosense_helios_16p` in a robot, you can give the parameter `id` a value, like `front_lidar`, `back_lidar`, etc., so the links, joints, sensors, etc., in the URDF/Xacro file will have unique names based on that value:

```xml
<robot name="botzilla" xmlns:xacro="http://www.ros.org/wiki/xacro" xmlns:gz="http://gazebosim.org/schema">
  <xacro:arg name="id"      default="pink_botzilla"/>
  <!-- arg transformed into property to use ${...} notation -->
  <xacro:property name="id" value="$(arg id)"/>

  <xacro:arg name="namespace" default=""/>
  <!-- arg transformed into property to use ${...} notation -->
  <xacro:property name="namespace" value="$(arg namespace)"/>

  <xacro:include filename="$(find eut_robotics_description)/urdf/lidars/robosense_helios_16p_macro.xacro"/>

  <xacro:property name="prefix"
                  value="${'' if namespace == '' or namespace == '/' else namespace.strip('/').replace('/', '_') + '_'}"/>

  <xacro:robosense_helios_16p
    id="front_lidar"
    prefix="${prefix}${id}_"
    parent_frame="${prefix}${id}_base_link"
    ...
```

It is important to understand how arguments are handled in the Python launch files associated with each robot defined in the **robots** folder.

Each `.launch.py` file declares a list of arguments that are required to instantiate the robot description properly. These include:

* **All the parameters expected by the corresponding Xacro file**, which typically represent the robot configuration (geometry, frame naming, enabled components, etc.).
* **Parameters required by the `robot_state_publisher` node**, such as `use_sim_time` or `publish_frequency`, that are not used in the Xacro but must be passed at runtime.
* An additional argument named `config_file`, which allows passing a YAML file containing any subset of the above parameters.

Each parameter can be provided from multiple sources, and the system uses a well-defined precedence rule to determine which value should be used at runtime.

The argument resolution priority is:

1. **Explicit values passed to the launch file**, either from the command line or from a parent launch file using `launch_arguments=[...]`.
2. **Values loaded from the YAML configuration file**, specified via the `config_file` argument.
3. **Fallback values defined in the launch file itself**, as part of the internal `Arguments` list.

If an argument is defined in more than one source, the one with higher priority takes precedence.
For example, if the same parameter is set both in the YAML file and from the command line, the command line value will be used.
If a value is not provided anywhere, the fallback defined in the `Arguments` list will be used.

This priority system provides flexibility to use the most appropriate source of configuration for each parameter: values can be passed explicitly via the command line or a parent launch file when needed, set globally through a YAML file for reuse across experiments, or simply fall back to predefined defaults when no specific value is required.
