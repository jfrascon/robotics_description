import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.sax.saxutils import quoteattr

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def source_test_env(tmp_path: Path) -> dict[str, str]:
    """Create an ament prefix that resolves robotics_description to its source tree."""
    ament_prefix = tmp_path / 'ament_prefix'
    resource_dir = ament_prefix / 'share' / 'ament_index' / 'resource_index' / 'packages'
    share_dir = ament_prefix / 'share'
    resource_dir.mkdir(parents=True, exist_ok=True)
    share_dir.mkdir(exist_ok=True)
    (resource_dir / 'robotics_description').touch()

    package_link = share_dir / 'robotics_description'
    if not package_link.exists():
        package_link.symlink_to(PACKAGE_ROOT, target_is_directory=True)

    env = os.environ.copy()
    current_ament_prefix_path = env.get('AMENT_PREFIX_PATH', '')
    env['AMENT_PREFIX_PATH'] = os.pathsep.join(path for path in [str(ament_prefix), current_ament_prefix_path] if path)
    return env


def run_macro(
    tmp_path: Path, *, macro_file: str, macro_name: str, arguments: dict[str, str]
) -> subprocess.CompletedProcess[str]:
    """Expand a minimal robot containing one macro instance."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    attributes = '\n'.join(f'    {name}={quoteattr(str(value))}' for name, value in arguments.items())
    test_xacro = tmp_path / f'{macro_name}_validation.xacro'
    test_xacro.write_text(
        f"""<?xml version="1.0"?>
<robot name="{macro_name}_validation" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find robotics_description)/{macro_file}"/>
  <link name="base_link"/>
  <xacro:{macro_name}
    name="component"
    parent_frame="base_link"
{attributes}/>
</robot>
""",
        encoding='utf-8',
    )

    return subprocess.run(
        [xacro_path, str(test_xacro)], capture_output=True, text=True, check=False, env=source_test_env(tmp_path)
    )


def expanded_root(result: subprocess.CompletedProcess[str]) -> ET.Element:
    """Parse a successful Xacro expansion."""
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout)


def assert_fatal(result: subprocess.CompletedProcess[str], message: str) -> None:
    """Require a failed expansion with the expected contract message."""
    assert result.returncode != 0
    assert message in result.stderr


def geometry(root: ET.Element, link_name: str, element_name: str) -> ET.Element | None:
    """Return the single shape inside one visual or collision element."""
    geometry_element = root.find(f"./link[@name='{link_name}']/{element_name}/geometry")
    if geometry_element is None:
        return None

    children = list(geometry_element)
    assert len(children) == 1
    return children[0]


def float_attribute(element: ET.Element, attribute: str) -> tuple[float, ...]:
    """Convert one whitespace-separated XML attribute to floats."""
    return tuple(float(value) for value in element.get(attribute, '').split())
