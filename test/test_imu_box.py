import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
TEST_MESH = 'robotics_description/meshes/sensors/imus/um7/mesh.dae'


def _source_test_env(tmp_path: Path) -> dict[str, str]:
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


def _run_imu_box(
    tmp_path: Path,
    *,
    mass: str = '0.01',
    size: str = '0.04 0.03 0.02',
    use_visual: str = 'True',
    use_collision: str = 'True',
    use_inertial: str = 'True',
    v_mesh: str = '',
    v_mesh_scale: str = '1.0 1.0 1.0',
    color: str = '0.1 0.2 0.3 1.0',
    c_mesh: str = '',
    c_mesh_scale: str = '1.0 1.0 1.0',
    joint_parent_fr_root_fr: str = '0 0 0 0 0 0',
    joint_root_fr_data_fr: str = '0 0 0 0 0 0',
    sim_enabled: str = 'False',
    sim_topic: str = '',
    namespace: str = '',
) -> subprocess.CompletedProcess[str]:
    """Expand a minimal robot containing one imu_box instance."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    test_xacro = tmp_path / 'imu_box_validation.xacro'
    test_xacro.write_text(
        f"""<?xml version="1.0"?>
<robot name="imu_box_validation" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find robotics_description)/urdf/sensors/imus/imu_box_macro.xacro"/>
  <link name="base_link"/>
  <xacro:imu_box name="imu"
                  parent_frame="base_link"
                  mass="{mass}"
                  size="{size}"
                  use_visual="{use_visual}"
                  use_collision="{use_collision}"
                  use_inertial="{use_inertial}"
                  v_mesh="{v_mesh}"
                  v_mesh_scale="{v_mesh_scale}"
                  color="{color}"
                  c_mesh="{c_mesh}"
                  c_mesh_scale="{c_mesh_scale}"
                  joint_parent_fr_root_fr="{joint_parent_fr_root_fr}"
                  joint_root_fr_data_fr="{joint_root_fr_data_fr}"
                  sim_enabled="{sim_enabled}"
                  namespace="{namespace}"
                  sim_topic="{sim_topic}"/>
</robot>
""",
        encoding='utf-8',
    )

    return subprocess.run(
        [xacro_path, str(test_xacro)], capture_output=True, text=True, check=False, env=_source_test_env(tmp_path)
    )


def _expanded_root(result: subprocess.CompletedProcess[str]) -> ET.Element:
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout)


def _assert_fatal(result: subprocess.CompletedProcess[str], message: str) -> None:
    assert result.returncode != 0
    assert message in result.stderr


def _geometry_name(root: ET.Element, element_name: str) -> str | None:
    geometry = root.find(f"./link[@name='imu_root_link']/{element_name}/geometry")
    if geometry is None:
        return None

    children = list(geometry)
    assert len(children) == 1
    return children[0].tag


def _float_attribute(element: ET.Element, attribute: str) -> tuple[float, ...]:
    return tuple(float(value) for value in element.get(attribute, '').split())


@pytest.mark.parametrize('size', ['0.04 0.03', '0.04 0.03 0.02 0.01'])
def test_imu_box_rejects_invalid_size_arity(tmp_path: Path, size: str) -> None:
    result = _run_imu_box(tmp_path, size=size)

    _assert_fatal(result, "imu_box: size must be 'len_x len_y len_z'")


@pytest.mark.parametrize(
    'size',
    [
        pytest.param('0.0 0.03 0.02', id='zero-x'),
        pytest.param('-0.04 0.03 0.02', id='negative-x'),
        pytest.param('0.04 0.0 0.02', id='zero-y'),
        pytest.param('0.04 -0.03 0.02', id='negative-y'),
        pytest.param('0.04 0.03 0.0', id='zero-z'),
        pytest.param('0.04 0.03 -0.02', id='negative-z'),
    ],
)
def test_imu_box_rejects_non_positive_size(tmp_path: Path, size: str) -> None:
    result = _run_imu_box(tmp_path, size=size)

    _assert_fatal(result, 'imu_box: size values must be greater than zero')


@pytest.mark.parametrize('mass', ['0.0', '-0.01'])
def test_imu_box_rejects_non_positive_mass_with_inertia(tmp_path: Path, mass: str) -> None:
    result = _run_imu_box(tmp_path, mass=mass)

    _assert_fatal(result, 'imu_box: mass must be greater than zero when inertial data is enabled')


@pytest.mark.parametrize('mass', ['0.0', '-0.01'])
def test_imu_box_allows_non_positive_mass_without_inertia(tmp_path: Path, mass: str) -> None:
    result = _run_imu_box(tmp_path, mass=mass, use_inertial='False')

    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('joint_parent_fr_root_fr', ['0 0 0 0 0', '0 0 0 0 0 0 0'])
def test_imu_box_rejects_invalid_parent_to_root_transform_arity(tmp_path: Path, joint_parent_fr_root_fr: str) -> None:
    result = _run_imu_box(tmp_path, joint_parent_fr_root_fr=joint_parent_fr_root_fr)

    _assert_fatal(result, "imu_box: joint_parent_fr_root_fr must be 'x y z roll pitch yaw'")


@pytest.mark.parametrize('joint_root_fr_data_fr', ['0 0 0 0 0', '0 0 0 0 0 0 0'])
def test_imu_box_rejects_invalid_root_to_data_transform_arity(tmp_path: Path, joint_root_fr_data_fr: str) -> None:
    result = _run_imu_box(tmp_path, joint_root_fr_data_fr=joint_root_fr_data_fr)

    _assert_fatal(result, "imu_box: joint_root_fr_data_fr must be 'x y z roll pitch yaw'")


@pytest.mark.parametrize(
    ('v_mesh', 'c_mesh', 'expected_visual', 'expected_collision'),
    [
        pytest.param('', '', 'box', 'box', id='primitive-primitive'),
        pytest.param(TEST_MESH, TEST_MESH, 'mesh', 'mesh', id='mesh-mesh'),
        pytest.param(TEST_MESH, '', 'mesh', 'box', id='mesh-primitive'),
        pytest.param('', TEST_MESH, 'box', 'mesh', id='primitive-mesh'),
    ],
)
def test_imu_box_selects_visual_and_collision_geometry_independently(
    tmp_path: Path, v_mesh: str, c_mesh: str, expected_visual: str, expected_collision: str
) -> None:
    root = _expanded_root(_run_imu_box(tmp_path, v_mesh=v_mesh, c_mesh=c_mesh))

    assert _geometry_name(root, 'visual') == expected_visual
    assert _geometry_name(root, 'collision') == expected_collision

    visual_origin = root.find("./link[@name='imu_root_link']/visual/origin")
    collision_origin = root.find("./link[@name='imu_root_link']/collision/origin")
    assert visual_origin is not None
    assert collision_origin is not None
    assert _float_attribute(visual_origin, 'xyz') == pytest.approx((0.0, 0.0, 0.01))
    assert _float_attribute(collision_origin, 'xyz') == pytest.approx((0.0, 0.0, 0.01))

    for mesh in root.findall("./link[@name='imu_root_link']/*/geometry/mesh"):
        assert mesh.get('filename') == f'package://{TEST_MESH}'
        assert _float_attribute(mesh, 'scale') == pytest.approx((1.0, 1.0, 1.0))


@pytest.mark.parametrize(
    ('use_visual', 'use_collision', 'expected_visual', 'expected_collision'),
    [
        pytest.param('False', 'True', None, 'box', id='visual-disabled'),
        pytest.param('True', 'False', 'box', None, id='collision-disabled'),
        pytest.param('False', 'False', None, None, id='both-disabled'),
    ],
)
def test_imu_box_respects_visual_and_collision_flags(
    tmp_path: Path, use_visual: str, use_collision: str, expected_visual: str | None, expected_collision: str | None
) -> None:
    root = _expanded_root(_run_imu_box(tmp_path, use_visual=use_visual, use_collision=use_collision))

    assert _geometry_name(root, 'visual') == expected_visual
    assert _geometry_name(root, 'collision') == expected_collision


def test_imu_box_creates_solid_box_inertia_at_body_origin(tmp_path: Path) -> None:
    root = _expanded_root(_run_imu_box(tmp_path, mass='0.02'))
    inertial = root.find("./link[@name='imu_root_link']/inertial")
    assert inertial is not None

    mass = inertial.find('mass')
    origin = inertial.find('origin')
    assert mass is not None
    assert origin is not None
    assert float(mass.get('value', '')) == pytest.approx(0.02)
    assert _float_attribute(origin, 'xyz') == pytest.approx((0.0, 0.0, 0.01))


def test_imu_box_creates_null_inertia_when_inertial_data_is_disabled(tmp_path: Path) -> None:
    root = _expanded_root(_run_imu_box(tmp_path, mass='-1.0', use_inertial='False'))
    inertial = root.find("./link[@name='imu_root_link']/inertial")
    assert inertial is not None

    mass = inertial.find('mass')
    origin = inertial.find('origin')
    assert mass is not None
    assert origin is not None
    assert float(mass.get('value', '')) == pytest.approx(0.01)
    assert _float_attribute(origin, 'xyz') == pytest.approx((0.0, 0.0, 0.0))


def test_imu_box_creates_body_and_data_frame_tree(tmp_path: Path) -> None:
    root = _expanded_root(
        _run_imu_box(tmp_path, joint_parent_fr_root_fr='1 2 3 0.1 0.2 0.3', joint_root_fr_data_fr='4 5 6 0.4 0.5 0.6')
    )

    assert root.find("./link[@name='imu_root_link']") is not None
    assert root.find("./link[@name='imu_link']") is not None

    root_joint = root.find("./joint[@name='imu_root_joint']")
    data_joint = root.find("./joint[@name='imu_joint']")
    assert root_joint is not None
    assert data_joint is not None
    assert root_joint.find('parent').get('link') == 'base_link'
    assert root_joint.find('child').get('link') == 'imu_root_link'
    assert data_joint.find('parent').get('link') == 'imu_root_link'
    assert data_joint.find('child').get('link') == 'imu_link'

    root_origin = root_joint.find('origin')
    data_origin = data_joint.find('origin')
    assert root_origin is not None
    assert data_origin is not None
    assert _float_attribute(root_origin, 'xyz') == pytest.approx((1.0, 2.0, 3.0))
    assert _float_attribute(root_origin, 'rpy') == pytest.approx((0.1, 0.2, 0.3))
    assert _float_attribute(data_origin, 'xyz') == pytest.approx((4.0, 5.0, 6.0))
    assert _float_attribute(data_origin, 'rpy') == pytest.approx((0.4, 0.5, 0.6))


@pytest.mark.parametrize('sim_topic', ['', '   '], ids=['empty', 'whitespace'])
def test_imu_box_requires_topic_when_simulation_is_enabled(tmp_path: Path, sim_topic: str) -> None:
    result = _run_imu_box(tmp_path, sim_enabled='True', sim_topic=sim_topic)

    _assert_fatal(result, 'imu_box: sim_topic must not be blank when simulation is enabled')


@pytest.mark.parametrize('sim_topic', ['', 'imu/data'], ids=['empty', 'configured'])
def test_imu_box_omits_sensor_when_simulation_is_disabled(tmp_path: Path, sim_topic: str) -> None:
    root = _expanded_root(_run_imu_box(tmp_path, sim_enabled='False', sim_topic=sim_topic))

    assert root.findall('.//sensor') == []


def test_imu_box_creates_simulated_sensor_on_data_frame(tmp_path: Path) -> None:
    root = _expanded_root(_run_imu_box(tmp_path, sim_enabled='True', sim_topic='imu/data'))

    sensor = root.find("./gazebo[@reference='imu_link']/sensor[@type='imu']")
    assert sensor is not None
    assert sensor.get('name') == 'imu_gz'
    assert sensor.findtext('topic') == 'imu/data'


def test_imu_box_applies_gazebo_material_when_visual_simulation_is_enabled(tmp_path: Path) -> None:
    root = _expanded_root(_run_imu_box(tmp_path, sim_enabled='True', sim_topic='imu/data'))

    material = root.find("./gazebo[@reference='imu_root_link']/material")
    assert material is not None
    assert material.findtext('ambient') == '0.1 0.2 0.3 1.0'
    assert material.findtext('diffuse') == '0.1 0.2 0.3 1.0'


def test_imu_box_omits_gazebo_material_when_visual_is_disabled(tmp_path: Path) -> None:
    root = _expanded_root(_run_imu_box(tmp_path, use_visual='False', sim_enabled='True', sim_topic='imu/data'))

    assert root.find("./gazebo[@reference='imu_root_link']/material") is None
    assert root.find("./gazebo[@reference='imu_link']/sensor[@type='imu']") is not None
