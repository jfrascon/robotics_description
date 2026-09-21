from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

from test_imu_box import _source_test_env

NAVSAT_PLUGIN_INCLUDE = (
    '  <xacro:include filename="$(find robotics_description)/urdf/sensors/gnss/generic_macros/'
    'plugin_navsat_macro.xacro"/>'
)
SET_PROPS_NAVSAT_INCLUDE = (
    '  <xacro:include filename="$(find robotics_description)/urdf/sensors/gnss/generic_macros/'
    'set_props_plugin_navsat_macro.xacro"/>'
)
NAVSAT_NOISE = (
    "${dict(type='gaussian', mean=1, stddev=2, bias_mean=3, "
    'bias_stddev=4, dynamic_bias_stddev=5, dynamic_bias_correlation_time=6, precision=7)}'
)
NAVSAT_CONFIG = (
    '${dict(enabled=True, use_gravity=True, always_on=True, update_rate=20, '
    'position_horizontal_noise=noise, position_vertical_noise=noise, '
    "velocity_horizontal_noise=noise, velocity_vertical_noise=noise, topic='gnss/fix')}"
)


def _expand_xacro(tmp_path: Path, name: str, content: str) -> ET.Element:
    """Expand an isolated NavSat macro fixture using the source package resources."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    source_file = tmp_path / name
    source_file.write_text(content, encoding='utf-8')
    result = subprocess.run(
        [xacro_path, str(source_file)],
        capture_output=True,
        text=True,
        check=False,
        env=_source_test_env(tmp_path),
    )
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout)


def test_plugin_navsat_emits_every_sdf_noise_field(tmp_path: Path) -> None:
    root = _expand_xacro(
        tmp_path,
        'plugin_navsat.xacro',
        """<?xml version="1.0"?>
<robot name="plugin_navsat_test" xmlns:xacro="http://www.ros.org/wiki/xacro">
"""
        + NAVSAT_PLUGIN_INCLUDE
        + """
  <link name="base_link"/>
  <xacro:plugin_navsat
    sensor_name="gnss"
    reference_link="base_link"
    namespace="robot"
    topic="gnss/fix"
    position_horizontal_noise="gaussian 1 2 3 4 5 6 7"
    position_vertical_noise="gaussian_quantized 11 12 13 14 15 16 17"
    velocity_horizontal_noise="none 21 22 23 24 25 26 27"
    velocity_vertical_noise="gaussian 31 32 33 34 35 36 37"/>
</robot>
""",
    )

    sensor = root.find("gazebo/sensor[@type='navsat']")
    assert sensor is not None
    assert sensor.findtext('topic') == 'robot/gnss/fix'

    expected = {
        'position_sensing/horizontal': ('gaussian', '1', '2', '3', '4', '5', '6', '7'),
        'position_sensing/vertical': (
            'gaussian_quantized',
            '11',
            '12',
            '13',
            '14',
            '15',
            '16',
            '17',
        ),
        'velocity_sensing/horizontal': ('none', '21', '22', '23', '24', '25', '26', '27'),
        'velocity_sensing/vertical': ('gaussian', '31', '32', '33', '34', '35', '36', '37'),
    }
    fields = (
        'mean',
        'stddev',
        'bias_mean',
        'bias_stddev',
        'dynamic_bias_stddev',
        'dynamic_bias_correlation_time',
        'precision',
    )
    for path, values in expected.items():
        noise = sensor.find(f'navsat/{path}/noise')
        assert noise is not None
        assert noise.attrib['type'] == values[0]
        assert tuple(noise.findtext(field) for field in fields) == values[1:]

    defaults_root = _expand_xacro(
        tmp_path,
        'plugin_navsat_defaults.xacro',
        """<?xml version="1.0"?>
<robot name="plugin_navsat_defaults" xmlns:xacro="http://www.ros.org/wiki/xacro">
"""
        + NAVSAT_PLUGIN_INCLUDE
        + """
  <link name="base_link"/>
  <xacro:plugin_navsat
    sensor_name="gnss"
    reference_link="base_link"
    topic="gnss/fix"/>
</robot>
""",
    )
    default_sensor = defaults_root.find("gazebo/sensor[@type='navsat']")
    assert default_sensor is not None
    for path in expected:
        noise = default_sensor.find(f'navsat/{path}/noise')
        assert noise is not None
        assert noise.attrib['type'] == 'gaussian'


def test_set_props_plugin_navsat_flattens_complete_noise_configuration(tmp_path: Path) -> None:
    root = _expand_xacro(
        tmp_path,
        'set_props_plugin_navsat.xacro',
        """<?xml version="1.0"?>
<robot name="set_props_plugin_navsat_test" xmlns:xacro="http://www.ros.org/wiki/xacro">
"""
        + SET_PROPS_NAVSAT_INCLUDE
        + '  <xacro:property name="noise" value="'
        + NAVSAT_NOISE
        + '"/>\n'
        + '  <xacro:property name="cfg" value="'
        + NAVSAT_CONFIG
        + '"/>\n'
        + """
  <xacro:set_props_plugin_navsat prop_prefix="gnss" navsat_sim_cfg="${cfg}"/>
  <result
    enabled="${gnss_enabled}"
    topic="${gnss_topic}"
    noise="${gnss_position_horizontal_noise}"/>
</robot>
""",
    )

    result = root.find('result')
    assert result is not None
    assert result.attrib == {
        'enabled': 'True',
        'topic': 'gnss/fix',
        'noise': 'gaussian 1 2 3 4 5 6 7',
    }
