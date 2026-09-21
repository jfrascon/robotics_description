from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

from test_imu_box import _source_test_env


LIVOX_MID360_MACRO = (
    '$(find robotics_description)/urdf/sensors/lidars/livox_mid360_macro.xacro'
)


def test_livox_mid360_uses_the_specified_imu_frame_and_plugin(tmp_path: Path) -> None:
    """Expand Livox and verify the IMU frame pose and Gazebo sensor contract."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    fixture = tmp_path / 'livox_mid360_imu.xacro'
    fixture.write_text(
        f"""<?xml version="1.0"?>
<robot name="livox_mid360_imu_test" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="{LIVOX_MID360_MACRO}"/>
  <link name="base_link"/>
  <xacro:livox_mid360 name="lidar"
                        prefix="robot_"
                        parent_frame="base_link"
                        joint_parent_fr_root_fr="0 0 0 0 0 0"
                        sim_enabled="False"
                        sim_update_rate="10"
                        sim_hor_fov_deg="-180 180"
                        sim_hor_res_deg="0.4"
                        sim_ver_fov_deg="0 90"
                        sim_ver_res_deg="0.947"
                        sim_dist_span="0.1 60"
                        sim_imu_enabled="True"
                        namespace="robot"
                        sim_imu_topic="lidar/imu"/>
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

    imu_link = root.find("link[@name='robot_lidar_imu_link']")
    assert imu_link is not None

    imu_joint = root.find("joint[@name='robot_lidar_imu_joint']")
    assert imu_joint is not None
    assert imu_joint.find('parent').attrib['link'] == 'robot_lidar_link'
    assert imu_joint.find('child').attrib['link'] == 'robot_lidar_imu_link'
    assert imu_joint.find('origin').attrib == {
        'rpy': '0 0 0',
        'xyz': '0.011 0.02329 -0.04412',
    }

    sensor = root.find("gazebo[@reference='robot_lidar_imu_link']/sensor[@type='imu']")
    assert sensor is not None
    assert sensor.attrib['name'] == 'robot_lidar_imu_gz'
    assert sensor.findtext('topic') == 'robot/lidar/imu'
