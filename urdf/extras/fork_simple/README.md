# fork_simple (URDF/Xacro)

This folder contains the URDF/Xacro side of the `fork_simple` model family.

The generic entry point is `fork_simple_macro.xacro`. It defines the simplified fork structure and expects all model parameters from the caller, including mesh path, geometry-related values, joint limits, and inertial terms. On top of that generic macro, `fork_simple_0_macro.xacro` acts as a concrete model wrapper that provides fixed values for one specific fork (`fork_simple_0`) and forwards integration parameters such as frame placement and naming.
If additional models are added, they should follow the same naming pattern, for example `fork_simple_1_macro.xacro`, `fork_simple_2_macro.xacro`, and so on.

In theory, a fork can be recreated in Xacro with primitive shapes, but it quickly becomes harder to read and maintain once offsets, multiple bodies, and validation checks are added.

The key design decision is that the generic macro is mesh-based, so it expects a `fork_simple` STL (generated from `meshes/extras/fork_simple/fork_simple_stl_generator.py`) and does not use a primitive-box fallback in this macro family.

Using the STL files produced by `meshes/extras/fork_simple/fork_simple_stl_generator.py` keeps the `fork_simple` models lightweight, reproducible, and easier to evolve without increasing URDF complexity.

With this approach, the STL mesh is generated in `meshes/extras/fork_simple/`, and the macros in this folder define how URDF uses that mesh as links, joints, and inertial data.

The usual workflow is: generate or update the STL in `meshes/extras/fork_simple/`, measure geometric/inertial properties in MeshLab, scale inertia to the target mass, and then place the resulting values (`mass`, `inertial_elements`, `center_of_mass`) in a model-specific macro such as `fork_simple_0_macro.xacro`.

In `fork_simple_0_macro.xacro`, `use_inertial` is enabled by default (`True`). If needed for simulation experiments, you can set `use_inertial` to `False` when calling the macro so the root link falls back to `null_inertia`. Also, mesh paths passed to `fork_simple` must be package-relative paths (without `package://`), because the generic macro already prepends that prefix internally.

For companion information, read `meshes/extras/fork_simple/README.md` for the mesh-side conventions and `doc/how_to_compute_inertia_w_meshlab.md` for the inertia computation/scaling procedure.
