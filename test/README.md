# Testing the URDF/XACRO descriptions

This document explains how to run the validation tests for the robot description files (URDF/XACRO) in the `robotics_description` package.

## 1. Purpose of the tests

The primary test script, `test_xacro.py`, is a crucial quality assurance tool. It is not a unit test for an algorithm, but rather a **validation and sanity check** for the robot models. For every "robot" xacro file found in the `urdf/robots/` directory, this test automatically performs the following checks:

1. **XACRO processing:** Verifies that the `.xacro` file can be successfully processed and converted into a final URDF file without syntax errors.
2. **URDF validation:** Runs `check_urdf` on the generated file to ensure it is a structurally valid URDF model (e.g., all joints connect existing links).
3. **Mesh path validation:** Parses the final URDF and verifies that every `<mesh>` tag:
    * Has the mandatory `filename` attribute.
    * Uses the portable `package://` URI scheme.
    * Points to a mesh file that actually exists on disk.

These tests are designed to prevent broken robot models from being committed and to catch errors early in the development process.

## 2. Required package configuration for testing

To ensure that these tests can be discovered and executed by `colcon`, two files in the package root must be correctly configured: `CMakeLists.txt` and `package.xml`.

### 2.1. CMakeLists.txt configuration

The `CMakeLists.txt` file must instruct the build system to find and register the tests. This is done inside a conditional block to make testing optional.

**Required block in `CMakeLists.txt`:**

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(urdf_xacro test/test_xacro.py)
endif()
```

**Explanation of each line:**

* `if(BUILD_TESTING)`: This is a standard CMake directive. It ensures that the test-related commands are only executed when `colcon` is building in a mode that enables testing (e.g., development or debug builds). This is the "on/off switch" for the tests.
* `find_package(ament_cmake_pytest REQUIRED)`: This command finds the necessary ROS 2 tools that provide the "glue" between the CMake build system and the `pytest` testing framework.
* `ament_add_pytest_test(urdf_xacro test/test_xacro.py)`: This is the most important command. It "registers" a new test with the system.
  * `urdf_xacro`: This is a user-defined **name** for the test. It's an identifier used by the build system.
  * `test/test_xacro.py`: This is the path to the actual Python script that contains the test logic.

### 2.2. package.xml dependencies

The `package.xml` file must declare all the external packages that are needed to *run* the tests. This allows tools like `rosdep` to automatically install everything required.

**Required `<test_depend>` tags in `package.xml`:**

```xml
  <!-- Tools for the testing framework itself -->
  <test_depend>ament_cmake_pytest</test_depend>

  <!-- Tools for running tests that involve ROS 2 launch files -->
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>launch_testing_ros</test_depend>

  <!-- Tool that provides the 'check_urdf' command -->
  <test_depend>liburdfdom-tools</test_depend>

  <!-- Tool that provides the 'xacro' command -->
  <test_depend>xacro</test_depend>
```

**Explanation of each dependency:**

* `ament_cmake_pytest`: Provides the core integration between `ament_cmake` and `pytest`.
* `launch_testing_*`: While the current test doesn't launch a ROS system, these are standard dependencies for any package that might include launch tests. It's good practice to keep them.
* `liburdfdom-tools`: This package provides the `check_urdf` executable, which the test script calls to validate the final URDF structure.
* `xacro`: This package provides the `xacro` executable, which the test script calls to process the `.xacro` files.

With these configurations in place, the package is self-contained and explicitly declares everything it needs to be tested robustly.

## 3. Building the workspace with tests enabled

Before running any tests, the workspace must be built in a way that registers them. Certain `colcon` build types, such as the `release` mixin, may disable tests by default. A "debug" or "development" build type will ensure tests are available.

### Recommended build command

From the workspace root (`~/workspace`), run the following commands to ensure a clean build that includes tests:

```bash
# First, clean the previous build artifacts to avoid any conflicts
rm -rf build/ install/ log/

# Now, build the workspace. Using a debug-ready mixin like 'rel-with-deb-info'
# typically enables testing by default.
colcon build --merge-install --symlink-install --parallel-workers 6 --mixin rel-with-deb-info --mixin compile-commands --cmake-args -DCMAKE_CXX_FLAGS=-Wall\ -Wextra\ -Wpedantic\ -Wnon-virtual-dtor\ -Woverloaded-virtual\ -Wnull-dereference\ -Wunused-parameter

# Source the new environment to make packages and executables findable
. install/setup.bash
```

## 4. Running the tests

Once the workspace is built correctly, there are three primary methods to execute the tests, each with a different purpose.

---

### Method A: Direct `pytest` execution (recommended for debugging)

This is the most effective method when actively developing or debugging a test, as it provides immediate, detailed, and unfiltered output directly to the terminal.

**Command:**

```bash
pytest -s -v src/0_deps/robotics_description/test/test_xacro.py
```

*(Adjust the path to the test file if the package is in a different subdirectory of `src/`)*

**Explanation of flags:**

* `-s`: (shorthand for `--show-capture=no`) This is the key flag that tells `pytest` **not to capture** the output. It allows `print()` statements to be displayed in real-time.
* `-v`: (shorthand for `verbose`) This provides more detailed output, including the full name of each test being run, making the results easier to read.

---

### Method B: Using `colcon test` (the official ROS 2 method)

This is the standard way to run tests in a ROS 2 workspace. It is typically used for a final check or in a Continuous Integration (CI) environment. By design, `colcon test` provides a high-level summary and **hides the detailed print output** from the terminal to keep it clean.

**Command:**

```bash
colcon test --merge-install --packages-select robotics_description
```

**How to see the detailed output:**
The required verbose output is not lost; it is simply redirected to log files. To view it, inspect the log file after the test run is complete.

1. Run the `colcon test` command above.
2. After it finishes, view the standard output log with this command:

    ```bash
    cat log/latest_test/robotics_description/stdout_stderr.log
    ```

    This file contains the complete, verbose output from `pytest`, including all `print()` statements.

---

### Method C: Direct script execution (for a quick sanity check)

The test file can also be run directly with Python. This is useful for a quick check to detect syntax errors, but it does not use the `pytest` framework, so it will not generate standard test reports.

**Command:**

```bash
python3 src/0_deps/robotics_description/test/test_xacro.py
```

This executes the code inside the `if __name__ == "__main__":` block of the test script, which is configured to run tests manually on all discovered xacro files.
