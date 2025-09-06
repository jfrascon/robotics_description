import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

# Get all robot xacro files
# test_file_path = os.path.abspath(__file__)
# test_dir = os.path.dirname(test_file_path)
# package_dir = os.path.dirname(test_dir)

xacro_files = []
robots_dir = os.path.join(get_package_share_directory('eut_robotics_description'), 'urdf', 'robots')
# robots_dir = os.path.join(package_dir, "urdf", "robots")

for root, _, files in os.walk(robots_dir):
    for file in files:
        # Only consider xacro files for fully operational robots, which do not end with the expression '_macro.xacro'.
        # If a xacro file ends with '_macro.xacro' inside the 'robots' folder it probably defines a complete robot, but
        # it needs to be instantiated properly with specific parameters to become a fully operational robot, and that
        # particular robot model defined in that xacro file will not use the word 'macro' in the name.
        # Most of the times, these macro xacro files in the 'robot' folder are complete robots that are combined
        # with other xacro files to form unique robot model, in a file that does not end with '_macro.xacro', just
        # 'xacro'.
        # For example, the 'mecanum_rectangular_forklift_macro.xacro' file defines a complete robot, a forklift with
        # rectangular base and mecanum wheels, but it is not a fully operational robot until it is instantiated with
        # specific parameters in a file that does not end with '_macro.xacro'. On the other hand, the file
        # 'forklift_artisteril.xacro' is a fully operational robot that uses the
        # 'mecanum_rectangular_forklift_macro.xacro' file and uses other macro files, like the lidar_3d_ring to create a
        # unique robot model, and therefore it does not end with '_macro.xacro'.
        if file.endswith('.xacro') and not file.endswith('_macro.xacro'):
            xacro_files.append(os.path.join(root, file))

print(f'DEBUG [pytest setup]: Found {len(xacro_files)} testable xacro files: {xacro_files}')


def check_meshes(urdf_file):
    """
    Check that all meshes in the URDF file exist and are valid.
    This function verifies:
    1. That every <mesh> tag has the mandatory 'filename' attribute.
    2. That every 'filename' uses the 'package://' format.
    3. That the file pointed to by the 'package://' path actually exists.

    Args:
        urdf_file (str): Path to the URDF file to check.
    Raises:
        AssertionError: If any of the above checks fail.
    """
    print(f"Checking meshes from file '{os.path.basename(urdf_file)}'")

    # Parse URDF file
    root = ET.parse(urdf_file).getroot()

    # Find all mesh elements
    meshes = root.findall('.//mesh')

    # List of all failed meshes [filename: reason]
    failed_meshes = []

    # Check that all meshes exist
    for mesh in meshes:
        filename = mesh.get('filename')

        if filename is None:
            failed_meshes.append(
                f"A <mesh> tag was found without the mandatory 'filename' attribute in file '{urdf_file}'"
            )

            continue

        if filename.startswith('package://'):
            package_name, _, package_path = filename[10:].partition('/')
            package_share_path = get_package_share_directory(package_name)

            try:
                package_share_path = get_package_share_directory(package_name)

                if not os.path.exists(os.path.join(package_share_path, package_path)):
                    failed_meshes.append(f"Mesh '{filename}' does not exist")
            except Exception:
                failed_meshes.append(f"Package '{package_name}' in mesh '{filename}' not found")
        else:
            failed_meshes.append(f"Mesh path is not in 'package://' format: '{filename}'")

    assert not failed_meshes, 'URDF mesh validation failed:\n  - ' + '\n  - '.join(failed_meshes)


@pytest.mark.parametrize('xacro_file', xacro_files)
def test_xacro_file(xacro_file):
    """Test that a xacro file can be converted to URDF and that the URDF file is valid.

    Args:
        xacro_file (str): Path to the xacro file to test.
    """

    xacro_filename_stem = os.path.splitext(os.path.basename(xacro_file))[0]
    tmp_urdf_output_file = os.path.join('/tmp', f'{xacro_filename_stem}.urdf')

    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    check_urdf_path = shutil.which('check_urdf')
    assert check_urdf_path, 'check_urdf is not installed'

    robot_id = 'tmp_robot'
    xacro_command_list = [xacro_path, xacro_file, f'robot_id:={robot_id}', '-o', tmp_urdf_output_file]
    check_command_list = [check_urdf_path, tmp_urdf_output_file]

    try:
        # Generate URDF file.
        print(f"Testing xacro file '{xacro_file}'")
        print(f'Executing command: {" ".join(xacro_command_list)}')

        xacro_process = subprocess.run(xacro_command_list, capture_output=True, text=True, check=False)
        assert xacro_process.returncode == 0, f'xacro command failed with stderr: {xacro_process.stderr}'
        assert os.path.exists(tmp_urdf_output_file), f"Output urdf file '{tmp_urdf_output_file}' not found"

        # Check URDF file.
        print(f'Executing command: {" ".join(check_command_list)}')
        check_process = subprocess.run(check_command_list, capture_output=True, text=True, check=False)
        assert check_process.returncode == 0, f'> check_urdf command failed with stderr: {check_process.stderr}'

        # Check meshes
        check_meshes(tmp_urdf_output_file)

    finally:
        os.remove(tmp_urdf_output_file)


if __name__ == '__main__':
    # This block is now only for manual debugging, not for pytest.
    # It will not run during 'colcon test'.
    if not xacro_files:
        print('No xacro files found to test.')
        exit(0)

    for xacro_file in xacro_files:
        test_xacro_file(xacro_file)
