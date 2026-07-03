# `plugin_pose_in_reference_frame_publisher`

The `plugin_pose_in_reference_frame_publisher` macro emits the XML for one Gazebo Sim plugin of type `gz::sim::systems::OdometryPublisher` attached to a robot model.

The Gazebo plugin publishes one message of type `gz::msgs::Odometry`, one message of type `gz::msgs::OdometryWithCovariance`, and it can also publish a transform. The odometry messages and the transform encode the pose of a **robot-attached frame with respect to the root frame selected through `reference_frame`**. Therefore, **the published pose is not necessarily odometry in the semantic sense.** However, depending on the frame names and offset values passed to the macro, the published pose can have different physical meanings.

This macro intentionally avoids using the prefix `odom_` in its public parameter names because the published pose is not necessarily odometry in the semantic sense.

## Transform convention

In this document the symbol ${}^{\mathrm{parent\_frame}}T_{\mathrm{child\_frame}}$ means the position and orientation of `child_frame` expressed in `parent_frame`. It can be used to transform points and vectors expressed in `child_frame` into `parent_frame`.

With that convention, the pose published by this macro is:

$$
{}^{\mathrm{reference\_frame}}T_{\mathrm{robot\_frame}}
=
{}^{\mathrm{reference\_frame}}T_{\mathrm{sim\_world\_frame}}
\cdot
{}^{\mathrm{sim\_world\_frame}}T_{\mathrm{robot\_reference\_frame}}
\cdot
{}^{\mathrm{robot\_reference\_frame}}T_{\mathrm{robot\_frame}}
$$

where:

- `reference_frame` is the root frame used for the published pose. It is the frame in which the pose of `robot_frame` is expressed.
- `sim_world_frame` is the frame of the SDF world used by Gazebo, i.e., the frame with respect to which the objects present in the SDF world file are positioned.
- `robot_reference_frame` is the robot-attached frame from which Gazebo computes the robot's raw pose. In typical robot descriptions this is `base_link`, `base_footprint_link`, or an equivalent frame.
- `robot_frame` is the robot-attached frame whose pose is published by this macro.

## Global Offsets

The `xyz_global_offset` and `rpy_global_offset` values provided by the user in the macro define the position and orientation of the `reference_frame` expressed in the `sim_world_frame` (the frame of the SDF world used by Gazebo):

$$
{}^{\mathrm{sim\_world\_frame}}T_{\mathrm{reference\_frame}}
=
\left[
\begin{array}{c|c}
R_{\mathrm{global\_offset}} &
\begin{array}{c}
x_{\mathrm{global\_offset}} \\
y_{\mathrm{global\_offset}} \\
z_{\mathrm{global\_offset}}
\end{array}
\\ \hline
\begin{array}{ccc}
0 & 0 & 0
\end{array}
& 1
\end{array}
\right]
$$

The Gazebo plugin internally inverts the transform ${}^{\mathrm{sim\_world\_frame}}T_{\mathrm{reference\_frame}}$ to compute the transform ${}^{\mathrm{reference\_frame}}T_{\mathrm{sim\_world\_frame}}$:

$$
{}^{\mathrm{reference\_frame}}T_{\mathrm{sim\_world\_frame}}
=
{}^{\mathrm{sim\_world\_frame}}T_{\mathrm{reference\_frame}}^{-1}
$$

When the global offsets are zero:

```xml
<xyz_global_offset>${xyz_global_offset}</xyz_global_offset>
<rpy_global_offset>${rpy_global_offset}</rpy_global_offset>
```

the transform ${}^{\mathrm{reference\_frame}}T_{\mathrm{sim\_world\_frame}}$ is the 4x4 identity matrix $I_4$:

$$
I_4
=
\begin{bmatrix}
I_3 & 0 \\
\boldsymbol{0} & 1
\end{bmatrix}
=
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

where $I_3$ is the 3x3 identity rotation matrix, $t = [0, 0, 0]^T$, and the last row is $[0, 0, 0, 1]$.
In that case, `reference_frame` coincides with `sim_world_frame` (the frame of the SDF world used by Gazebo).

## Local Offsets

The `xyz_offset` and `rpy_offset` values provided by the user in the macro define the position and orientation of `robot_frame`, expressed in `robot_reference_frame`:

$$
{}^{\mathrm{robot\_reference\_frame}}T_{\mathrm{robot\_frame}}
=
\left[
\begin{array}{c|c}
R_{\mathrm{local\_offset}} &
\begin{array}{c}
x_{\mathrm{local\_offset}} \\
y_{\mathrm{local\_offset}} \\
z_{\mathrm{local\_offset}}
\end{array}
\\ \hline
\begin{array}{ccc}
0 & 0 & 0
\end{array}
& 1
\end{array}
\right]
$$

The robot reference frame is typically `base_link`, `base_footprint_link`, or an equivalent frame.

When the local offsets are zero:

```xml
<xyz_offset>${xyz_offset}</xyz_offset>
<rpy_offset>${rpy_offset}</rpy_offset>
```

the transform ${}^{\mathrm{robot\_reference\_frame}}T_{\mathrm{robot\_frame}}$ is the 4x4 identity matrix $I_4$.
In that case, `robot_frame` coincides with `robot_reference_frame`.

## How to interpret the published pose

- If the global offsets are zero and the local offsets are zero, `reference_frame` coincides with `sim_world_frame`, and `robot_frame` coincides with `robot_reference_frame`.
The plugin publishes the pose of `robot_reference_frame` with respect to `sim_world_frame`.
This is simulator ground-truth localization, not odometry relative to the initial pose of `robot_reference_frame`.

- If the global offsets are zero and the local offsets are non-zero, `reference_frame` still coincides with `sim_world_frame`.
The plugin publishes the pose of `robot_frame` with respect to `sim_world_frame`. This is also simulator ground-truth localization, but for `robot_frame` instead of `robot_reference_frame`.

- If the global offsets are non-zero and the local offsets are zero, `robot_frame` coincides with `robot_reference_frame`.
The plugin publishes the pose of `robot_reference_frame` with respect to `reference_frame`.
**If the global offsets place `reference_frame` at the initial pose of `robot_reference_frame`, the output is interpreted as odometry for `robot_reference_frame`.**

- If the global offsets are non-zero and the local offsets are non-zero, the plugin publishes the pose of `robot_frame` with respect to `reference_frame`.
**If the global offsets place `reference_frame` at the initial pose of `robot_reference_frame`, the output is interpreted as odometry for `robot_frame`.**

Now that the physical meaning of the published pose is defined, the reader can understand better why we use at the beginning of the macro file a comment like this:

```text
The Gazebo plugin publishes one message of type `gz::msgs::Odometry`, one message of type `gz::msgs::OdometryWithCovariance`, and it can also publish a transform. The odometry messages and the transform encode the pose of a **robot-attached frame with respect to the root frame selected through `reference_frame`**. Therefore, **the published pose is not necessarily odometry in the semantic sense.** However, depending on the frame names and offset values passed to the macro, the published pose can have different physical meanings.
```

The message type alone does not define the physical meaning of the data. A `gz::msgs::Odometry` message can contain simulator ground-truth localization, odometry relative to the initial pose of `robot_reference_frame`, or another referenced pose. **The meaning is defined by the frame names and by the transforms introduced through the global and local offsets.**

## Macro parameters

The caller has to decide:

- The root frame used for the published pose, through `reference_frame`.
- The robot-attached frame whose pose is published, through `robot_frame`.
- The publication frequency through `msg_publication_freq`.
- The message topics through `topic` and `topic_with_covariance`.
- The TF topic through `tf_topic`.
- Whether the published pose is reported in 2D or 3D through `dimensions`.
- The local and global pose offsets through the `*_offset` parameters.
- The Gaussian noise value through `gaussian_noise`. Gazebo uses this value to populate the covariance information of the `gz::msgs::OdometryWithCovariance` message.

## Gazebo documentation

For the ROS 2 Jazzy and Gazebo Sim Harmonic combination used by this branch (July 2026), the applicable Gazebo Sim API reference is the Sim 8 API reference. The system plugin namespace and the wrapped `OdometryPublisher` plugin are documented here:

- <https://gazebosim.org/api/sim/8/namespacegz_1_1sim_1_1systems.html>
- <https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html>
