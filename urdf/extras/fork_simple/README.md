# fork_simple (URDF/Xacro)

This folder contains the current `fork_simple` macro and a temporarily retained legacy implementation.

## Current macro

`fork_simple_macro.xacro` defines the public `fork_simple` macro used by maintained robot descriptions and tests.
It creates separate root, carriage, tine-stop and tine links.
The carriage moves on a prismatic joint, while the remaining fork joints are fixed.

The current model uses fixed tine dimensions and separation rather than a general scale parameter.
`carriage_size_x` and `carriage_size_z` remain configurable within the validation rules implemented by the macro.
`tine_open` controls whether the front closing plates are omitted from the two hollow tines.

Visual and collision geometry are controlled independently.
`use_v_mesh` selects the fixed tine mesh or a box primitive for visual geometry.
`use_c_mesh` independently selects the same mesh or a box primitive for collision geometry.
The carriage and tine-stop geometry always use box primitives.

Example:

```xml
<xacro:include filename="$(find robotics_description)/urdf/extras/fork_simple/fork_simple_macro.xacro"/>

<xacro:fork_simple name="fork"
                   parent_frame="base_link"
                   limits="-0.10 0.10 1.0 100.0"
                   joint_parent_fr_root_fr="0 0 0 0 0 0"/>
```

`limits` is ordered as `lower upper velocity effort`.
`joint_parent_fr_root_fr` is ordered as `x y z roll pitch yaw`.

## Legacy macro

`fork_simple_legacy_macro.xacro` defines the temporary compatibility macro `fork_simple_legacy`.
It retains the former whole-fork mesh model, anisotropic `scale`, closed and open mesh variants, and MeshLab-based inertia scaling.
New robot descriptions should use `fork_simple` unless they explicitly need to compare against the legacy model.

Example:

```xml
<xacro:include filename="$(find robotics_description)/urdf/extras/fork_simple/fork_simple_legacy_macro.xacro"/>

<xacro:fork_simple_legacy name="fork_legacy"
                          parent_frame="base_link"
                          scale="1.0 1.0 1.0"
                          use_inertial="True"
                          open_tines="False"
                          limits="-0.10 0.10 1.0 100.0"
                          joint_parent_fr_root_fr="0 0 0 0 0 0"/>
```

The legacy wrapper internally uses `generic_macros/fork_simple_links_joints_legacy_macro.xacro`.
That helper defines `fork_simple_links_joints_legacy` and is not intended as a new public entry point.
Both legacy files may be removed after confirming that no robot description still depends on them.

## Mesh and inertia references

The current tine mesh is stored under `meshes/extras/fork_tine/`.
The legacy whole-fork meshes and their generator are stored under `meshes/extras/fork_simple/`.
See `meshes/extras/fork_simple/README.md` for the legacy mesh frame convention.
See `doc/how_to_compute_inertia_w_meshlab.md` for the MeshLab inertia procedure used by the legacy wrapper.
