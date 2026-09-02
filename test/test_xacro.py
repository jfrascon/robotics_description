import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

xacro_files = []
test_dir = os.path.dirname(os.path.abspath(__file__))
package_root = os.path.dirname(test_dir)
package_name = os.path.basename(package_root)
test_xacros_dir = os.path.join(test_dir, 'test_xacros')

if os.path.isdir(test_xacros_dir):
    for file in sorted(os.listdir(test_xacros_dir)):
        if file.endswith('.xacro'):
            xacro_files.append(os.path.join(test_xacros_dir, file))

all_xacro_files = list(xacro_files)
urdf_dir = os.path.join(package_root, 'urdf')

for root_dir, directories, files in os.walk(urdf_dir):
    directories.sort()

    for file in sorted(files):
        if file.endswith('.xacro'):
            all_xacro_files.append(os.path.join(root_dir, file))


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

            try:
                package_share_path = get_package_share_directory_from_test_env(package_name)

                if not os.path.exists(os.path.join(package_share_path, package_path)):
                    failed_meshes.append(f"Mesh '{filename}' does not exist")
            except Exception:
                failed_meshes.append(f"Package '{package_name}' in mesh '{filename}' not found")
        else:
            failed_meshes.append(f"Mesh path is not in 'package://' format: '{filename}'")

    assert not failed_meshes, 'URDF mesh validation failed:\n  - ' + '\n  - '.join(failed_meshes)


def create_source_package_ament_prefix():
    """
    Create an ament index prefix that points this package name to the source tree.

    The test Xacro files use $(find robotics_description). When pytest is run
    directly from the source tree, $(find ...) may otherwise resolve an older
    installed copy under the workspace install directory. That would test stale
    files and miss newly added macros until the package is installed again.
    """
    ament_prefix = tempfile.TemporaryDirectory()
    resource_dir = os.path.join(ament_prefix.name, 'share', 'ament_index', 'resource_index', 'packages')
    share_dir = os.path.join(ament_prefix.name, 'share')
    os.makedirs(resource_dir, exist_ok=True)
    os.makedirs(share_dir, exist_ok=True)

    with open(os.path.join(resource_dir, package_name), 'w', encoding='utf-8'):
        pass

    os.symlink(package_root, os.path.join(share_dir, package_name))

    return ament_prefix


def get_package_share_directory_from_test_env(name):
    """Resolve this package to the source tree and all other packages normally."""
    if name == package_name:
        return package_root

    return get_package_share_directory(name)


@pytest.mark.parametrize('xacro_file', all_xacro_files)
def test_macro_file_robot_name_uses_macro_suffix(xacro_file):
    """
    Check the naming convention for files that declare reusable Xacro macros.

    A file that declares a <xacro:macro> is a macro container, not a standalone
    robot model. Its top-level <robot name="..."> therefore uses a _macro or
    _macros suffix. The public macro names themselves do not use that suffix.
    """
    root = ET.parse(xacro_file).getroot()
    macro_elements = [element for element in root.iter() if element.tag == '{http://www.ros.org/wiki/xacro}macro']

    if not macro_elements:
        return

    robot_name = root.get('name')
    assert robot_name, f"Macro file '{xacro_file}' must set the top-level <robot name>"
    assert robot_name.endswith(('_macro', '_macros')), (
        f"Macro file '{xacro_file}' has robot name '{robot_name}', "
        'but macro-container robot names must end with _macro or _macros'
    )

    macro_names_with_suffix = [
        macro.get('name') for macro in macro_elements if macro.get('name', '').endswith(('_macro', '_macros'))
    ]
    assert not macro_names_with_suffix, (
        f"Macro file '{xacro_file}' has public macro names with a macro suffix: {macro_names_with_suffix}"
    )


@pytest.mark.parametrize('xacro_file', all_xacro_files)
def test_gz_namespace_is_used_when_declared(xacro_file):
    """Require every declared ``gz`` XML namespace to qualify an element or attribute."""
    namespace_events = ET.iterparse(xacro_file, events=('start-ns',))
    gz_namespace_uris = {uri for _, (prefix, uri) in namespace_events if prefix == 'gz'}

    if not gz_namespace_uris:
        return

    root = ET.parse(xacro_file).getroot()
    expanded_names = tuple(f'{{{uri}}}' for uri in gz_namespace_uris)
    uses_gz_namespace = any(
        element.tag.startswith(expanded_names)
        or any(attribute.startswith(expanded_names) for attribute in element.attrib)
        for element in root.iter()
    )

    assert uses_gz_namespace, f"Xacro file '{xacro_file}' declares xmlns:gz but never uses it"


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

    ament_prefix = create_source_package_ament_prefix()
    env = os.environ.copy()
    current_ament_prefix_path = env.get('AMENT_PREFIX_PATH', '')
    env['AMENT_PREFIX_PATH'] = os.pathsep.join(path for path in [ament_prefix.name, current_ament_prefix_path] if path)

    try:
        # Generate URDF file.
        print(f"Testing xacro file '{xacro_file}'")
        print(f'Executing command: {" ".join(xacro_command_list)}')

        xacro_process = subprocess.run(xacro_command_list, capture_output=True, text=True, check=False, env=env)
        assert xacro_process.returncode == 0, f'xacro command failed with stderr: {xacro_process.stderr}'
        assert os.path.exists(tmp_urdf_output_file), f"Output urdf file '{tmp_urdf_output_file}' not found"

        # Check URDF file.
        print(f'Executing command: {" ".join(check_command_list)}')
        check_process = subprocess.run(check_command_list, capture_output=True, text=True, check=False, env=env)
        assert check_process.returncode == 0, f'> check_urdf command failed with stderr: {check_process.stderr}'

        # Check meshes
        check_meshes(tmp_urdf_output_file)

    finally:
        if os.path.exists(tmp_urdf_output_file):
            os.remove(tmp_urdf_output_file)
        ament_prefix.cleanup()


if __name__ == '__main__':
    # This block is now only for manual debugging, not for pytest.
    # It will not run during 'colcon test'.
    if not xacro_files:
        print('No xacro files found to test.')
        exit(0)

    for xacro_file in xacro_files:
        test_xacro_file(xacro_file)
