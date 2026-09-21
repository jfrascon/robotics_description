from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

from test_imu_box import _source_test_env

UBLOX_MACRO = (
    '$(find robotics_description)/urdf/sensors/gnss/'
    'ublox_antenna_ann_mb_00_00_macro.xacro'
)


def test_ublox_antenna_geometry_frames_and_navsat_plugin(tmp_path: Path) -> None:
    """Expand the antenna and verify its fixed geometry and frame contract."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    fixture = tmp_path / 'ublox_antenna_ann_mb_00_00.xacro'
    fixture.write_text(
        f"""<?xml version="1.0"?>
<robot name="ublox_antenna_test" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{UBLOX_MACRO}"/>
  <link name="base_link"/>
  <xacro:ublox_antenna_ann_mb_00_00 name="front_gnss"
                                        prefix="robot_"
                                        parent_frame="base_link"
                                        joint_parent_fr_root_fr="1 2 3 0.1 0.2 0.3"
                                        sim_enabled="True"
                                        namespace="robot"
                                        sim_topic="gnss/fix"/>
</robot>
""",
        encoding='utf-8',
    )
    result = subprocess.run(
        [xacro_path, str(fixture)],
        capture_output=True,
        text=True,
        check=False,
        env=_source_test_env(tmp_path),
    )
    assert result.returncode == 0, result.stderr
    root = ET.fromstring(result.stdout)

    root_link = root.find("link[@name='robot_front_gnss_root_link']")
    assert root_link is not None
    visual_mesh = root_link.find('visual/geometry/mesh')
    collision_mesh = root_link.find('collision/geometry/mesh')
    assert visual_mesh is not None
    assert collision_mesh is not None
    assert visual_mesh.attrib == {
        'filename': (
            'package://robotics_description/meshes/sensors/gnss/'
            'ublox_antenna_ann_mb_00_00/ublox_antenna_ann_mb_00_00.obj'
        ),
        'scale': '1 1 1',
    }
    assert collision_mesh.attrib == {
        'filename': (
            'package://robotics_description/meshes/sensors/gnss/'
            'ublox_antenna_ann_mb_00_00/ublox_antenna_ann_mb_00_00.stl'
        ),
        'scale': '1 1 1',
    }
    assert root_link.find('visual/origin').attrib == {
        'rpy': '0.0 0.0 0.0',
        'xyz': '0.0 0.0 0.01125',
    }
    assert root_link.find('collision/origin').attrib == {
        'rpy': '0.0 0.0 0.0',
        'xyz': '0.0 0.0 0.01125',
    }

    root_joint = root.find("joint[@name='robot_front_gnss_root_joint']")
    assert root_joint is not None
    assert root_joint.find('parent').attrib['link'] == 'base_link'
    assert root_joint.find('child').attrib['link'] == 'robot_front_gnss_root_link'
    assert root_joint.find('origin').attrib == {'rpy': '0.1 0.2 0.3', 'xyz': '1 2 3'}

    data_link = root.find("link[@name='robot_front_gnss_link']")
    assert data_link is not None
    data_joint = root.find("joint[@name='robot_front_gnss_joint']")
    assert data_joint is not None
    assert data_joint.find('origin').attrib == {'rpy': '0 0 0', 'xyz': '0 0 0'}

    sensor = root.find("gazebo[@reference='robot_front_gnss_link']/sensor[@type='navsat']")
    assert sensor is not None
    assert sensor.findtext('topic') == 'robot/gnss/fix'


def test_ublox_antenna_box_fallback_uses_nominal_dimensions(tmp_path: Path) -> None:
    """Verify the primitive fallback uses the specified physical envelope."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    fixture = tmp_path / 'ublox_antenna_box.xacro'
    fixture.write_text(
        f"""<?xml version="1.0"?>
<robot name="ublox_antenna_box_test" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{UBLOX_MACRO}"/>
  <link name="base_link"/>
  <xacro:ublox_antenna_ann_mb_00_00 name="gnss"
                                        parent_frame="base_link"
                                        use_v_mesh="False"
                                        use_c_mesh="False"
                                        joint_parent_fr_root_fr="0 0 0 0 0 0"/>
</robot>
""",
        encoding='utf-8',
    )
    result = subprocess.run(
        [xacro_path, str(fixture)],
        capture_output=True,
        text=True,
        check=False,
        env=_source_test_env(tmp_path),
    )
    assert result.returncode == 0, result.stderr
    root = ET.fromstring(result.stdout)

    root_link = root.find("link[@name='gnss_root_link']")
    assert root_link is not None
    assert root_link.find('visual/geometry/box').attrib['size'] == '0.06 0.082 0.0225'
    assert root_link.find('collision/geometry/box').attrib['size'] == '0.06 0.082 0.0225'
