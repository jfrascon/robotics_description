import shutil
import subprocess
from pathlib import Path

from test_imu_box import _expanded_root, _source_test_env


def test_all_imus_macros_exports_public_entry_points(tmp_path: Path) -> None:
    """The aggregator exposes its helper and both component macros."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    test_xacro = tmp_path / 'all_imus_validation.xacro'
    test_xacro.write_text(
        """<?xml version="1.0"?>
<robot name="all_imus_validation" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find robotics_description)/urdf/sensors/imus/all_imus_macros.xacro"/>
  <xacro:set_props_plugin_imu prop_prefix="cfg"/>
  <link name="base_link"/>
  <xacro:imu_box name="generic_imu"
                  parent_frame="base_link"
                  mass="0.01"
                  size="0.04 0.03 0.02"
                  joint_parent_fr_root_fr="0 0 0 0 0 0"
                  sim_enabled="${cfg_enabled}"
                  sim_topic="${cfg_topic}"/>
  <xacro:um7 name="um7"
             parent_frame="base_link"
             joint_parent_fr_root_fr="0.1 0 0 0 0 0"/>
</robot>
""",
        encoding='utf-8',
    )

    result = subprocess.run(
        [xacro_path, str(test_xacro)], capture_output=True, text=True, check=False, env=_source_test_env(tmp_path)
    )
    root = _expanded_root(result)

    expected_links = {'base_link', 'generic_imu_root_link', 'generic_imu_link', 'um7_root_link', 'um7_link'}
    assert expected_links.issubset({link.get('name') for link in root.findall('link')})
    assert root.findall('.//sensor') == []
