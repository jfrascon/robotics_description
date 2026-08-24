import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from test_imu_box import PACKAGE_ROOT, TEST_MESH, _expanded_root, _float_attribute, _geometry_name, _source_test_env


def _run_um7(
    tmp_path: Path,
    *,
    use_visual: str = 'True',
    use_collision: str = 'True',
    use_inertial: str = 'True',
    use_v_mesh: str = 'True',
    color: str = '',
    use_c_mesh: str = 'False',
    joint_parent_fr_root_fr: str = '0 0 0 0 0 0',
    sim_enabled: str = 'False',
    sim_update_rate: str = '100',
    namespace: str = '',
    sim_topic: str | None = None,
) -> subprocess.CompletedProcess[str]:
    """Expand a minimal robot containing one UM7 instance."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    sim_topic_attribute = '' if sim_topic is None else f'\n             sim_topic="{sim_topic}"'
    test_xacro = tmp_path / 'um7_validation.xacro'
    test_xacro.write_text(
        f"""<?xml version="1.0"?>
<robot name="um7_validation" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find robotics_description)/urdf/sensors/imus/um7_macro.xacro"/>
  <link name="base_link"/>
  <xacro:um7 name="imu"
             parent_frame="base_link"
             use_visual="{use_visual}"
             use_collision="{use_collision}"
             use_inertial="{use_inertial}"
             use_v_mesh="{use_v_mesh}"
             color="{color}"
             use_c_mesh="{use_c_mesh}"
             joint_parent_fr_root_fr="{joint_parent_fr_root_fr}"
             sim_enabled="{sim_enabled}"
             sim_update_rate="{sim_update_rate}"
             namespace="{namespace}"{sim_topic_attribute}/>
</robot>
""",
        encoding='utf-8',
    )

    return subprocess.run(
        [xacro_path, str(test_xacro)], capture_output=True, text=True, check=False, env=_source_test_env(tmp_path)
    )


def _run_xacro_file(tmp_path: Path, xacro_file: Path) -> ET.Element:
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'
    result = subprocess.run(
        [xacro_path, str(xacro_file)], capture_output=True, text=True, check=False, env=_source_test_env(tmp_path)
    )
    return _expanded_root(result)


def test_um7_defines_fixed_physical_model_and_data_frame(tmp_path: Path) -> None:
    root = _expanded_root(_run_um7(tmp_path))
    body_link = root.find("./link[@name='imu_root_link']")
    assert body_link is not None

    visual_mesh = body_link.find('./visual/geometry/mesh')
    collision_box = body_link.find('./collision/geometry/box')
    inertial = body_link.find('inertial')
    assert visual_mesh is not None
    assert collision_box is not None
    assert inertial is not None
    assert visual_mesh.get('filename') == f'package://{TEST_MESH}'
    assert _float_attribute(visual_mesh, 'scale') == pytest.approx((1.0, 1.0, 1.0))
    assert _float_attribute(collision_box, 'size') == pytest.approx((0.0285, 0.0285, 0.0128))

    mass = inertial.find('mass')
    inertial_origin = inertial.find('origin')
    assert mass is not None
    assert inertial_origin is not None
    assert float(mass.get('value', '')) == pytest.approx(0.011)
    assert _float_attribute(inertial_origin, 'xyz') == pytest.approx((0.0, 0.0, 0.0064))

    data_joint = root.find("./joint[@name='imu_joint']")
    assert data_joint is not None
    data_origin = data_joint.find('origin')
    assert data_origin is not None
    assert data_joint.find('parent').get('link') == 'imu_root_link'
    assert data_joint.find('child').get('link') == 'imu_link'
    assert _float_attribute(data_origin, 'xyz') == pytest.approx((-0.00785, 0.00785, 0.01118))
    assert _float_attribute(data_origin, 'rpy') == pytest.approx((3.141592653589793, 0.0, 0.0))


@pytest.mark.parametrize(
    ('use_v_mesh', 'use_c_mesh', 'expected_visual', 'expected_collision'),
    [
        pytest.param('False', 'False', 'box', 'box', id='primitive-primitive'),
        pytest.param('True', 'True', 'mesh', 'mesh', id='mesh-mesh'),
        pytest.param('True', 'False', 'mesh', 'box', id='mesh-primitive'),
        pytest.param('False', 'True', 'box', 'mesh', id='primitive-mesh'),
    ],
)
def test_um7_selects_visual_and_collision_geometry_independently(
    tmp_path: Path, use_v_mesh: str, use_c_mesh: str, expected_visual: str, expected_collision: str
) -> None:
    root = _expanded_root(_run_um7(tmp_path, use_v_mesh=use_v_mesh, use_c_mesh=use_c_mesh))

    assert _geometry_name(root, 'visual') == expected_visual
    assert _geometry_name(root, 'collision') == expected_collision


@pytest.mark.parametrize(
    ('use_visual', 'use_collision', 'expected_visual', 'expected_collision'),
    [
        pytest.param('False', 'True', None, 'box', id='visual-disabled'),
        pytest.param('True', 'False', 'mesh', None, id='collision-disabled'),
        pytest.param('False', 'False', None, None, id='both-disabled'),
    ],
)
def test_um7_forwards_visual_and_collision_flags(
    tmp_path: Path, use_visual: str, use_collision: str, expected_visual: str | None, expected_collision: str | None
) -> None:
    root = _expanded_root(_run_um7(tmp_path, use_visual=use_visual, use_collision=use_collision))

    assert _geometry_name(root, 'visual') == expected_visual
    assert _geometry_name(root, 'collision') == expected_collision


def test_um7_forwards_disabled_inertial_mode(tmp_path: Path) -> None:
    root = _expanded_root(_run_um7(tmp_path, use_inertial='False'))
    inertial = root.find("./link[@name='imu_root_link']/inertial")
    assert inertial is not None

    mass = inertial.find('mass')
    origin = inertial.find('origin')
    assert mass is not None
    assert origin is not None
    assert float(mass.get('value', '')) == pytest.approx(0.01)
    assert _float_attribute(origin, 'xyz') == pytest.approx((0.0, 0.0, 0.0))


def test_um7_uses_embedded_mesh_material_when_color_is_empty(tmp_path: Path) -> None:
    root = _expanded_root(_run_um7(tmp_path, use_v_mesh='True', color=''))

    assert root.find("./link[@name='imu_root_link']/visual/material") is None


def test_um7_applies_explicit_visual_color(tmp_path: Path) -> None:
    root = _expanded_root(_run_um7(tmp_path, color='0.1 0.2 0.3 1.0'))
    material = root.find("./link[@name='imu_root_link']/visual/material")
    assert material is not None
    assert material.get('name') == 'imu_color'

    rgba = material.find('color')
    assert rgba is not None
    assert _float_attribute(rgba, 'rgba') == pytest.approx((0.1, 0.2, 0.3, 1.0))


def test_um7_allows_omitted_topic_when_simulation_is_disabled(tmp_path: Path) -> None:
    root = _expanded_root(_run_um7(tmp_path, sim_enabled='False', sim_topic=None))

    assert root.findall('.//sensor') == []


@pytest.mark.parametrize('sim_topic', ['', '   '], ids=['empty', 'whitespace'])
def test_um7_rejects_blank_topic_when_simulation_is_enabled(tmp_path: Path, sim_topic: str) -> None:
    result = _run_um7(tmp_path, sim_enabled='True', sim_topic=sim_topic)

    assert result.returncode != 0
    assert 'imu_box: sim_topic must not be blank when simulation is enabled' in result.stderr


def test_um7_creates_simulated_sensor_on_data_frame(tmp_path: Path) -> None:
    root = _expanded_root(
        _run_um7(tmp_path, sim_enabled='True', sim_update_rate='75.0', namespace='robot', sim_topic='imu/data')
    )

    sensor = root.find("./gazebo[@reference='imu_link']/sensor[@type='imu']")
    assert sensor is not None
    assert sensor.get('name') == 'imu_gz'
    assert sensor.findtext('topic') == 'robot/imu/data'
    assert float(sensor.findtext('update_rate', '')) == pytest.approx(75.0)
    assert sensor.findtext('gz_frame_id') == 'imu_link'


def test_um7_example_noise_configuration_expands_to_numeric_values(tmp_path: Path) -> None:
    root = _run_xacro_file(tmp_path, PACKAGE_ROOT / 'test' / 'test_xacros' / 'test_um7.xacro')
    noise_values = [field.text for noise in root.findall('.//noise') for field in noise]
    assert noise_values

    for value in noise_values:
        assert value is not None
        float(value)
