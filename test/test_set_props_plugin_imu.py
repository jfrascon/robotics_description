import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from test_imu_box import _source_test_env

COMPLETE_CONFIG = """dict(
    enabled=True,
    use_gravity=False,
    always_on=True,
    update_rate=42.0,
    ang_vel_gaussian_noise=dict(mean_x=1, stddev_x=2, mean_y=3, stddev_y=4, mean_z=5, stddev_z=6),
    ang_vel_gaussian_noise_bias=dict(mean_x=11, stddev_x=12, mean_y=13, stddev_y=14, mean_z=15, stddev_z=16),
    ang_vel_gaussian_noise_dynamic_bias=dict(
        stddev_x=21, corr_time_x=22, stddev_y=23, corr_time_y=24, stddev_z=25, corr_time_z=26
    ),
    lin_acc_gaussian_noise=dict(mean_x=31, stddev_x=32, mean_y=33, stddev_y=34, mean_z=35, stddev_z=36),
    lin_acc_gaussian_noise_bias=dict(
        mean_x=41, stddev_x=42, mean_y=43, stddev_y=44, mean_z=45, stddev_z=46
    ),
    lin_acc_gaussian_noise_dynamic_bias=dict(
        stddev_x=51, corr_time_x=52, stddev_y=53, corr_time_y=54, stddev_z=55, corr_time_z=56
    ),
    use_orientation=False,
    topic='imu/data',
)"""


def _run_set_props(
    tmp_path: Path, *, config_expression: str | None = None, prop_prefix: str = 'imu'
) -> subprocess.CompletedProcess[str]:
    """Expand set_props_plugin_imu and expose its parent-scope properties."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    config_block = ''
    config_argument = ''
    if config_expression is not None:
        config_block = f'  <xacro:property name="cfg" value="${{{config_expression}}}"/>\n'
        config_argument = ' imu_sim_cfg="${cfg}"'

    test_xacro = tmp_path / 'set_props_plugin_imu_validation.xacro'
    test_xacro.write_text(
        f"""<?xml version="1.0"?>
<robot name="set_props_plugin_imu_validation" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include
    filename="$(find robotics_description)/urdf/sensors/imus/generic_macros/set_props_plugin_imu_macro.xacro"/>
{config_block}  <xacro:set_props_plugin_imu prop_prefix="{prop_prefix}"{config_argument}/>
  <result enabled="${{imu_enabled}}"
          use_gravity="${{imu_use_gravity}}"
          always_on="${{imu_always_on}}"
          update_rate="${{imu_update_rate}}"
          ang_noise="${{imu_ang_vel_gaussian_noise}}"
          ang_bias="${{imu_ang_vel_gaussian_noise_bias}}"
          ang_dynamic_bias="${{imu_ang_vel_gaussian_noise_dynamic_bias}}"
          lin_noise="${{imu_lin_acc_gaussian_noise}}"
          lin_bias="${{imu_lin_acc_gaussian_noise_bias}}"
          lin_dynamic_bias="${{imu_lin_acc_gaussian_noise_dynamic_bias}}"
          use_orientation="${{imu_use_orientation}}"
          topic="${{imu_topic}}"/>
</robot>
""",
        encoding='utf-8',
    )

    return subprocess.run(
        [xacro_path, str(test_xacro)], capture_output=True, text=True, check=False, env=_source_test_env(tmp_path)
    )


def _result_element(result: subprocess.CompletedProcess[str]) -> ET.Element:
    assert result.returncode == 0, result.stderr
    element = ET.fromstring(result.stdout).find('result')
    assert element is not None
    return element


def test_set_props_plugin_imu_uses_disabled_defaults_without_configuration(tmp_path: Path) -> None:
    result = _result_element(_run_set_props(tmp_path))

    assert result.attrib == {
        'enabled': 'False',
        'use_gravity': 'False',
        'always_on': 'False',
        'update_rate': '0',
        'ang_noise': '0 0 0 0 0 0',
        'ang_bias': '0 0 0 0 0 0',
        'ang_dynamic_bias': '0 0 0 0 0 0',
        'lin_noise': '0 0 0 0 0 0',
        'lin_bias': '0 0 0 0 0 0',
        'lin_dynamic_bias': '0 0 0 0 0 0',
        'use_orientation': 'False',
        'topic': '',
    }


def test_set_props_plugin_imu_ignores_partial_configuration_when_disabled(tmp_path: Path) -> None:
    result = _result_element(_run_set_props(tmp_path, config_expression="dict(enabled=False, topic='ignored')"))

    assert result.get('enabled') == 'False'
    assert result.get('topic') == ''


def test_set_props_plugin_imu_maps_complete_enabled_configuration(tmp_path: Path) -> None:
    result = _result_element(_run_set_props(tmp_path, config_expression=COMPLETE_CONFIG))

    assert result.attrib == {
        'enabled': 'True',
        'use_gravity': 'False',
        'always_on': 'True',
        'update_rate': '42.0',
        'ang_noise': '1 2 3 4 5 6',
        'ang_bias': '11 12 13 14 15 16',
        'ang_dynamic_bias': '21 22 23 24 25 26',
        'lin_noise': '31 32 33 34 35 36',
        'lin_bias': '41 42 43 44 45 46',
        'lin_dynamic_bias': '51 52 53 54 55 56',
        'use_orientation': 'False',
        'topic': 'imu/data',
    }


def test_set_props_plugin_imu_rejects_incomplete_enabled_configuration(tmp_path: Path) -> None:
    incomplete_config = COMPLETE_CONFIG.replace("    topic='imu/data',\n", '')
    result = _run_set_props(tmp_path, config_expression=incomplete_config)

    assert result.returncode != 0
    assert 'set_props_plugin_imu: enabled configuration is missing fields: topic' in result.stderr


def test_set_props_plugin_imu_rejects_blank_prefix(tmp_path: Path) -> None:
    result = _run_set_props(tmp_path, prop_prefix='   ')

    assert result.returncode != 0
    assert 'set_props_plugin_imu: prop_prefix must not be blank' in result.stderr
