"""Test the basket Xacro macro and its installed mesh references."""

from pathlib import Path
import subprocess
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_basket_expands_to_valid_urdf_with_existing_meshes(tmp_path: Path) -> None:
    """Expand one basket instance and require every generated mesh to exist."""
    wrapper = tmp_path / 'basket.xacro'
    urdf = tmp_path / 'basket.urdf'
    macro = PACKAGE_ROOT / 'urdf' / 'extras' / 'basket' / 'basket_macro.xacro'
    wrapper.write_text(
        f"""<?xml version="1.0"?>
<robot name="basket_test" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{macro}"/>
  <link name="base_link"/>
  <xacro:structure_basket prefix="" parent="base_link">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:structure_basket>
</robot>
""",
        encoding='utf-8',
    )

    subprocess.run(['xacro', str(wrapper), '-o', str(urdf)], check=True, timeout=20)
    subprocess.run(['check_urdf', str(urdf)], check=True, timeout=20)

    for mesh in ElementTree.parse(urdf).getroot().iter('mesh'):
        uri = mesh.attrib['filename']
        assert uri.startswith('package://')
        package_name, relative_path = uri.removeprefix('package://').split('/', maxsplit=1)
        package_share = Path(get_package_share_directory(package_name))
        assert package_share.joinpath(relative_path).is_file(), uri
