# Testing URDF/Xacro in `robotics_description`

This directory contains the validation tests for the URDF/Xacro content of the `robotics_description` package.

The main test entry point is:
- `test_xacro.py`

## What is tested

`test_xacro.py` validates the explicit test cases under `test/test_xacros/`.

These are small Xacro files that instantiate reusable macros with valid
arguments and, when needed, example simulation configurations. They exist
because most files in `robotics_description` are reusable macro building
blocks, not standalone robot descriptions that can be rendered and validated
directly on their own.

Current explicit test cases in `test/test_xacros/`:
- `test_box_imu.xacro`
- `test_fork_simple.xacro`
- `test_gz_system_plugins.xacro`
- `test_orbbec_gemini335le_rgbd.xacro`
- `test_orbbec_gemini335le_split.xacro`
- `test_realsense_d435.xacro`
- `test_realsense_d435_rgbd.xacro`
- `test_robosense_airy.xacro`
- `test_robosense_helios_16.xacro`
- `test_robosense_m1_plus.xacro`
- `test_steerable_wheel.xacro`
- `test_um7.xacro`
- `test_wheel.xacro`

For every collected Xacro file, the test performs:

1. Xacro expansion
   - Runs `xacro` and checks that the file can be rendered into URDF.

2. URDF structural validation
   - Runs `check_urdf` on the generated URDF.

3. Mesh validation
   - Verifies that every `<mesh>` element:
     - has a `filename`
     - uses the `package://` scheme
     - points to an existing file on disk

## Why `test_xacros/` exists

Many files in `urdf/sensors/...` are macros such as:
- `box_imu_macro.xacro`
- `orbbec_gemini335le_split_macro.xacro`
- `orbbec_gemini335le_rgbd_macro.xacro`
- `realsense_d435_split_macro.xacro`
- `realsense_d435_rgbd_macro.xacro`
- `robosense_airy_macro.xacro`
- `robosense_helios_16_macro.xacro`
- `robosense_m1_plus_macro.xacro`
- `steerable_wheel_macro.xacro`
- `um7_macro.xacro`
- `wheel_macro.xacro`

These macros are reusable building blocks, not complete robot models. To test them properly, each one must be:
- included from a small test Xacro
- instantiated with valid arguments
- connected to example simulation / bridge YAML files when needed

That is exactly what the files under `test/test_xacros/` do.

## Package configuration required for testing

The package is currently wired for testing with:

In `../CMakeLists.txt`:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(xacro_test test/test_xacro.py)
endif()
```

Relevant test dependencies in `../package.xml`:

```xml
<test_depend>ament_cmake_pytest</test_depend>
<test_depend>launch_testing_ament_cmake</test_depend>
<test_depend>launch_testing_ros</test_depend>
<test_depend>liburdfdom-tools</test_depend>
<test_depend>xacro</test_depend>
```

The important executables for this test are:
- `xacro`
- `check_urdf`

## How to run the tests

From the workspace root:

```bash
colcon test --merge-install --packages-select robotics_description
colcon test-result --all --verbose
```

For direct debugging with full output:

```bash
pytest -s -v src/0_deps/robotics_description/test/test_xacro.py
```

You can also run the test file directly:

```bash
python3 src/0_deps/robotics_description/test/test_xacro.py
```

## Notes

- `colcon test-result --all --verbose` may show warnings caused by unrelated stale files in the workspace `build/` tree.
- Those warnings do not necessarily mean that `robotics_description` itself is failing.
- The authoritative result for this package is the `xacro_test` result under:
  - `build/robotics_description/test_results/robotics_description/xacro_test.xunit.xml`
