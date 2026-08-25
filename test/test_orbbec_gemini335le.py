import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.sax.saxutils import quoteattr

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BODY_LINK = 'camera_bottom_screw_frame'
BODY_SIZE = (0.05, 0.124022, 0.029035)
BODY_ORIGIN = (0.01301, 0.0, 0.013578)
MESH_DIRECTORY = 'package://robotics_description/meshes/sensors/cameras/orbbec_gemini335le'


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


def _run_links_joints(
    tmp_path: Path,
    *,
    use_visual: str = 'True',
    use_collision: str = 'True',
    use_inertial: str = 'True',
    use_v_mesh: str = 'True',
    v_mesh_use_low_res: str = 'True',
    color: str = '0.1 0.2 0.3 1.0',
    use_c_mesh: str = 'False',
    c_mesh_use_low_res: str = 'True',
    joint_parent_fr_root_fr: str = '0 0 0 0 0 0',
) -> subprocess.CompletedProcess[str]:
    """Expand a minimal robot containing the Orbbec links-and-joints macro."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    test_xacro = tmp_path / 'orbbec_gemini335le_links_joints_validation.xacro'
    test_xacro.write_text(
        f"""<?xml version="1.0"?>
<robot name="orbbec_gemini335le_links_joints_validation"
       xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include
    filename="$(find robotics_description)/urdf/sensors/cameras/orbbec_gemini335le_links_joints_macro.xacro"/>
  <link name="base_link"/>
  <xacro:orbbec_gemini335le_links_joints
    name="camera"
    parent_frame="base_link"
    use_visual="{use_visual}"
    use_collision="{use_collision}"
    use_inertial="{use_inertial}"
    use_v_mesh="{use_v_mesh}"
    v_mesh_use_low_res="{v_mesh_use_low_res}"
    color="{color}"
    use_c_mesh="{use_c_mesh}"
    c_mesh_use_low_res="{c_mesh_use_low_res}"
    joint_parent_fr_root_fr="{joint_parent_fr_root_fr}"/>
</robot>
""",
        encoding='utf-8',
    )

    return subprocess.run(
        [xacro_path, str(test_xacro)], capture_output=True, text=True, check=False, env=_source_test_env(tmp_path)
    )


def _run_client(tmp_path: Path, filename: str) -> ET.Element:
    """Expand one of the package's complete Orbbec wrapper test clients."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    result = subprocess.run(
        [xacro_path, str(PACKAGE_ROOT / 'test' / 'test_xacros' / filename)],
        capture_output=True,
        text=True,
        check=False,
        env=_source_test_env(tmp_path),
    )
    return _expanded_root(result)


def _run_wrapper(
    tmp_path: Path, *, macro_filename: str, macro_name: str, arguments: dict[str, str]
) -> subprocess.CompletedProcess[str]:
    """Expand a minimal robot containing one complete Orbbec wrapper."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    attributes = '\n'.join(f'    {name}={quoteattr(value)}' for name, value in arguments.items())
    test_xacro = tmp_path / f'{macro_name}_validation.xacro'
    test_xacro.write_text(
        f"""<?xml version="1.0"?>
<robot name="{macro_name}_validation" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find robotics_description)/urdf/sensors/cameras/{macro_filename}"/>
  <link name="base_link"/>
  <xacro:{macro_name}
    name="camera"
    parent_frame="base_link"
{attributes}/>
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


def _geometry(root: ET.Element, element_name: str) -> ET.Element | None:
    geometry = root.find(f"./link[@name='{BODY_LINK}']/{element_name}/geometry")
    if geometry is None:
        return None

    children = list(geometry)
    assert len(children) == 1
    return children[0]


def _float_attribute(element: ET.Element, attribute: str) -> tuple[float, ...]:
    return tuple(float(value) for value in element.get(attribute, '').split())


@pytest.mark.parametrize('joint_parent_fr_root_fr', ['0 0 0 0 0', '0 0 0 0 0 0 0'])
def test_links_joints_rejects_invalid_parent_transform_arity(tmp_path: Path, joint_parent_fr_root_fr: str) -> None:
    result = _run_links_joints(tmp_path, joint_parent_fr_root_fr=joint_parent_fr_root_fr)

    _assert_fatal(result, "orbbec_gemini335le_links_joints: joint_parent_fr_root_fr must be 'x y z roll pitch yaw'")


@pytest.mark.parametrize(
    ('use_v_mesh', 'use_c_mesh', 'expected_visual', 'expected_collision'),
    [
        pytest.param('False', 'False', 'box', 'box', id='box-box'),
        pytest.param('True', 'True', 'mesh', 'mesh', id='mesh-mesh'),
        pytest.param('True', 'False', 'mesh', 'box', id='mesh-box'),
        pytest.param('False', 'True', 'box', 'mesh', id='box-mesh'),
    ],
)
def test_links_joints_selects_visual_and_collision_geometry_independently(
    tmp_path: Path, use_v_mesh: str, use_c_mesh: str, expected_visual: str, expected_collision: str
) -> None:
    root = _expanded_root(_run_links_joints(tmp_path, use_v_mesh=use_v_mesh, use_c_mesh=use_c_mesh))
    visual = _geometry(root, 'visual')
    collision = _geometry(root, 'collision')

    assert visual is not None
    assert collision is not None
    assert visual.tag == expected_visual
    assert collision.tag == expected_collision

    visual_origin = root.find(f"./link[@name='{BODY_LINK}']/visual/origin")
    collision_origin = root.find(f"./link[@name='{BODY_LINK}']/collision/origin")
    assert visual_origin is not None
    assert collision_origin is not None
    assert _float_attribute(visual_origin, 'xyz') == pytest.approx(BODY_ORIGIN)
    assert _float_attribute(collision_origin, 'xyz') == pytest.approx(BODY_ORIGIN)

    for geometry in [visual, collision]:
        if geometry.tag == 'box':
            assert _float_attribute(geometry, 'size') == pytest.approx(BODY_SIZE)
        else:
            assert _float_attribute(geometry, 'scale') == pytest.approx((1.0, 1.0, 1.0))


@pytest.mark.parametrize(
    ('low_resolution', 'mesh_filename'),
    [pytest.param('True', 'low_res_mesh.STL', id='low-resolution'), pytest.param('False', 'mesh.STL', id='detailed')],
)
def test_links_joints_selects_mesh_resolution(tmp_path: Path, low_resolution: str, mesh_filename: str) -> None:
    root = _expanded_root(
        _run_links_joints(
            tmp_path,
            use_v_mesh='True',
            v_mesh_use_low_res=low_resolution,
            use_c_mesh='True',
            c_mesh_use_low_res=low_resolution,
        )
    )

    expected_filename = f'{MESH_DIRECTORY}/{mesh_filename}'
    assert _geometry(root, 'visual').get('filename') == expected_filename
    assert _geometry(root, 'collision').get('filename') == expected_filename


@pytest.mark.parametrize(
    ('use_visual', 'use_collision', 'expected_visual', 'expected_collision'),
    [
        pytest.param('False', 'True', None, 'box', id='visual-disabled'),
        pytest.param('True', 'False', 'mesh', None, id='collision-disabled'),
        pytest.param('False', 'False', None, None, id='both-disabled'),
    ],
)
def test_links_joints_respects_geometry_flags(
    tmp_path: Path, use_visual: str, use_collision: str, expected_visual: str | None, expected_collision: str | None
) -> None:
    root = _expanded_root(_run_links_joints(tmp_path, use_visual=use_visual, use_collision=use_collision))

    visual = _geometry(root, 'visual')
    collision = _geometry(root, 'collision')
    assert (visual.tag if visual is not None else None) == expected_visual
    assert (collision.tag if collision is not None else None) == expected_collision


def test_links_joints_applies_optional_visual_color(tmp_path: Path) -> None:
    root = _expanded_root(_run_links_joints(tmp_path, color='0.1 0.2 0.3 1.0'))
    material = root.find(f"./link[@name='{BODY_LINK}']/visual/material")
    color = root.find(f"./link[@name='{BODY_LINK}']/visual/material/color")
    assert material is not None
    assert material.get('name') == 'camera_color'
    assert color is not None
    assert _float_attribute(color, 'rgba') == pytest.approx((0.1, 0.2, 0.3, 1.0))

    root_without_color = _expanded_root(_run_links_joints(tmp_path, color=''))
    assert root_without_color.find(f"./link[@name='{BODY_LINK}']/visual/material") is None


def test_links_joints_creates_fixed_body_inertia(tmp_path: Path) -> None:
    root = _expanded_root(_run_links_joints(tmp_path))
    inertial = root.find(f"./link[@name='{BODY_LINK}']/inertial")
    assert inertial is not None

    mass = inertial.find('mass')
    origin = inertial.find('origin')
    assert mass is not None
    assert origin is not None
    assert float(mass.get('value', '')) == pytest.approx(0.220)
    assert _float_attribute(origin, 'xyz') == pytest.approx(BODY_ORIGIN)


def test_links_joints_creates_null_inertia_when_disabled(tmp_path: Path) -> None:
    root = _expanded_root(_run_links_joints(tmp_path, use_inertial='False'))
    inertial = root.find(f"./link[@name='{BODY_LINK}']/inertial")
    assert inertial is not None

    mass = inertial.find('mass')
    origin = inertial.find('origin')
    assert mass is not None
    assert origin is not None
    assert float(mass.get('value', '')) == pytest.approx(0.01)
    assert _float_attribute(origin, 'xyz') == pytest.approx((0.0, 0.0, 0.0))


def test_links_joints_creates_complete_fixed_frame_tree(tmp_path: Path) -> None:
    root = _expanded_root(_run_links_joints(tmp_path, joint_parent_fr_root_fr='1 2 3 0.1 0.2 0.3'))
    suffixes = {
        'link',
        'bottom_screw_frame',
        'depth_frame',
        'depth_optical_frame',
        'right_ir_frame',
        'right_ir_optical_frame',
        'left_ir_frame',
        'left_ir_optical_frame',
        'color_frame',
        'color_optical_frame',
        'imu_frame',
    }
    expected_links = {'base_link'} | {f'camera_{suffix}' for suffix in suffixes}
    assert {link.get('name') for link in root.findall('link')} == expected_links
    assert len(root.findall('joint')) == len(suffixes)

    parent_joint = root.find("./joint[@name='camera_joint']")
    assert parent_joint is not None
    assert parent_joint.find('parent').get('link') == 'base_link'
    assert parent_joint.find('child').get('link') == 'camera_link'
    assert _float_attribute(parent_joint.find('origin'), 'xyz') == pytest.approx((1.0, 2.0, 3.0))
    assert _float_attribute(parent_joint.find('origin'), 'rpy') == pytest.approx((0.1, 0.2, 0.3))

    optical_joint = root.find("./joint[@name='camera_depth_optical_joint']")
    assert optical_joint is not None
    assert _float_attribute(optical_joint.find('origin'), 'rpy') == pytest.approx(
        (-1.5707963267948966, 0.0, -1.5707963267948966)
    )

    assert root.findall('.//sensor') == []


@pytest.mark.parametrize(
    ('filename', 'expected_sensors'),
    [
        pytest.param(
            'test_orbbec_gemini335le_split.xacro',
            {
                ('camera_depth_frame', 'camera_depth_gz_sensor', 'depth_camera'),
                ('camera_color_frame', 'camera_color_gz_sensor', 'camera'),
                ('camera_left_ir_frame', 'camera_left_ir_gz_sensor', 'camera'),
                ('camera_right_ir_frame', 'camera_right_ir_gz_sensor', 'camera'),
                ('camera_imu_frame', 'camera_imu', 'imu'),
            },
            id='split-wrapper',
        ),
        pytest.param(
            'test_orbbec_gemini335le_rgbd.xacro',
            {('camera_depth_frame', 'camera_rgbd_gz_sensor', 'rgbd_camera'), ('camera_imu_frame', 'camera_imu', 'imu')},
            id='rgbd-wrapper',
        ),
    ],
)
def test_wrapper_clients_preserve_body_geometry_and_expected_sensors(
    tmp_path: Path, filename: str, expected_sensors: set[tuple[str, str, str]]
) -> None:
    root = _run_client(tmp_path, filename)
    visual = _geometry(root, 'visual')
    collision = _geometry(root, 'collision')
    assert visual is not None
    assert collision is not None
    assert visual.tag == 'mesh'
    assert visual.get('filename') == f'{MESH_DIRECTORY}/low_res_mesh.STL'
    assert collision.tag == 'box'
    assert _float_attribute(collision, 'size') == pytest.approx(BODY_SIZE)

    actual_sensors = {
        (gazebo.get('reference'), sensor.get('name'), sensor.get('type'))
        for gazebo in root.findall('gazebo')
        for sensor in gazebo.findall('sensor')
    }
    assert actual_sensors == expected_sensors


SPLIT_DISABLED_SENSORS = {
    'sim_depth_enabled': 'False',
    'sim_color_enabled': 'False',
    'sim_left_ir_enabled': 'False',
    'sim_right_ir_enabled': 'False',
    'sim_imu_enabled': 'False',
}
RGBD_DISABLED_SENSORS = {
    'sim_rgbd_enabled': 'False',
    'sim_left_ir_enabled': 'False',
    'sim_right_ir_enabled': 'False',
    'sim_imu_enabled': 'False',
}

WRAPPERS_WITH_DISABLED_SENSORS = [
    pytest.param(
        'orbbec_gemini335le_split_macro.xacro', 'orbbec_gemini335le', SPLIT_DISABLED_SENSORS, id='split-wrapper'
    ),
    pytest.param(
        'orbbec_gemini335le_rgbd_macro.xacro', 'orbbec_gemini335le_rgbd', RGBD_DISABLED_SENSORS, id='rgbd-wrapper'
    ),
]


@pytest.mark.parametrize(('macro_filename', 'macro_name', 'disabled_arguments'), WRAPPERS_WITH_DISABLED_SENSORS)
def test_wrappers_allow_empty_topics_for_disabled_sensors(
    tmp_path: Path, macro_filename: str, macro_name: str, disabled_arguments: dict[str, str]
) -> None:
    root = _expanded_root(
        _run_wrapper(tmp_path, macro_filename=macro_filename, macro_name=macro_name, arguments=disabled_arguments)
    )

    assert root.findall('.//sensor') == []


@pytest.mark.parametrize(
    ('macro_filename', 'macro_name', 'disabled_arguments', 'enabled_argument', 'message'),
    [
        pytest.param(
            'orbbec_gemini335le_split_macro.xacro',
            'orbbec_gemini335le',
            SPLIT_DISABLED_SENSORS,
            'sim_depth_enabled',
            'orbbec_gemini335le: depth image and camera-info topics must not be empty',
            id='split-depth',
        ),
        pytest.param(
            'orbbec_gemini335le_split_macro.xacro',
            'orbbec_gemini335le',
            SPLIT_DISABLED_SENSORS,
            'sim_color_enabled',
            'orbbec_gemini335le: color image and camera-info topics must not be empty',
            id='split-color',
        ),
        pytest.param(
            'orbbec_gemini335le_split_macro.xacro',
            'orbbec_gemini335le',
            SPLIT_DISABLED_SENSORS,
            'sim_left_ir_enabled',
            'orbbec_gemini335le: left infrared image and camera-info topics must not be empty',
            id='split-left-infrared',
        ),
        pytest.param(
            'orbbec_gemini335le_split_macro.xacro',
            'orbbec_gemini335le',
            SPLIT_DISABLED_SENSORS,
            'sim_right_ir_enabled',
            'orbbec_gemini335le: right infrared image and camera-info topics must not be empty',
            id='split-right-infrared',
        ),
        pytest.param(
            'orbbec_gemini335le_split_macro.xacro',
            'orbbec_gemini335le',
            SPLIT_DISABLED_SENSORS,
            'sim_imu_enabled',
            'orbbec_gemini335le: IMU topic must not be empty',
            id='split-imu',
        ),
        pytest.param(
            'orbbec_gemini335le_rgbd_macro.xacro',
            'orbbec_gemini335le_rgbd',
            RGBD_DISABLED_SENSORS,
            'sim_rgbd_enabled',
            'orbbec_gemini335le_rgbd: RGB-D base topic must not be empty',
            id='rgbd',
        ),
        pytest.param(
            'orbbec_gemini335le_rgbd_macro.xacro',
            'orbbec_gemini335le_rgbd',
            RGBD_DISABLED_SENSORS,
            'sim_left_ir_enabled',
            'orbbec_gemini335le_rgbd: left infrared image and camera-info topics must not be empty',
            id='rgbd-left-infrared',
        ),
        pytest.param(
            'orbbec_gemini335le_rgbd_macro.xacro',
            'orbbec_gemini335le_rgbd',
            RGBD_DISABLED_SENSORS,
            'sim_right_ir_enabled',
            'orbbec_gemini335le_rgbd: right infrared image and camera-info topics must not be empty',
            id='rgbd-right-infrared',
        ),
        pytest.param(
            'orbbec_gemini335le_rgbd_macro.xacro',
            'orbbec_gemini335le_rgbd',
            RGBD_DISABLED_SENSORS,
            'sim_imu_enabled',
            'orbbec_gemini335le_rgbd: IMU topic must not be empty',
            id='rgbd-imu',
        ),
    ],
)
def test_wrappers_reject_empty_topics_for_enabled_sensors(
    tmp_path: Path,
    macro_filename: str,
    macro_name: str,
    disabled_arguments: dict[str, str],
    enabled_argument: str,
    message: str,
) -> None:
    arguments = disabled_arguments | {enabled_argument: 'True'}
    result = _run_wrapper(tmp_path, macro_filename=macro_filename, macro_name=macro_name, arguments=arguments)

    _assert_fatal(result, message)


@pytest.mark.parametrize(
    ('macro_filename', 'macro_name', 'disabled_arguments', 'arguments', 'message'),
    [
        pytest.param(
            'orbbec_gemini335le_split_macro.xacro',
            'orbbec_gemini335le',
            SPLIT_DISABLED_SENSORS,
            {'sim_imu_enabled': 'True', 'sim_imu_topic': '   '},
            'orbbec_gemini335le: IMU topic must not be empty',
            id='split-imu',
        ),
        pytest.param(
            'orbbec_gemini335le_rgbd_macro.xacro',
            'orbbec_gemini335le_rgbd',
            RGBD_DISABLED_SENSORS,
            {'sim_rgbd_enabled': 'True', 'sim_rgbd_base_topic': '   '},
            'orbbec_gemini335le_rgbd: RGB-D base topic must not be empty',
            id='rgbd',
        ),
    ],
)
def test_wrappers_reject_whitespace_only_topics(
    tmp_path: Path,
    macro_filename: str,
    macro_name: str,
    disabled_arguments: dict[str, str],
    arguments: dict[str, str],
    message: str,
) -> None:
    result = _run_wrapper(
        tmp_path, macro_filename=macro_filename, macro_name=macro_name, arguments=disabled_arguments | arguments
    )

    _assert_fatal(result, message)


@pytest.mark.parametrize(
    ('macro_filename', 'macro_name', 'disabled_arguments', 'arguments', 'message'),
    [
        pytest.param(
            'orbbec_gemini335le_split_macro.xacro',
            'orbbec_gemini335le',
            SPLIT_DISABLED_SENSORS,
            {
                'sim_depth_enabled': 'True',
                'sim_depth_image_topic': 'depth/image',
                'sim_depth_camera_info_topic': 'depth/camera_info',
                'sim_depth_triggered': 'True',
            },
            'orbbec_gemini335le: depth trigger topic must not be empty',
            id='split-depth',
        ),
        pytest.param(
            'orbbec_gemini335le_split_macro.xacro',
            'orbbec_gemini335le',
            SPLIT_DISABLED_SENSORS,
            {
                'sim_color_enabled': 'True',
                'sim_color_image_topic': 'color/image',
                'sim_color_camera_info_topic': 'color/camera_info',
                'sim_color_triggered': 'True',
            },
            'orbbec_gemini335le: color trigger topic must not be empty',
            id='split-color',
        ),
        pytest.param(
            'orbbec_gemini335le_split_macro.xacro',
            'orbbec_gemini335le',
            SPLIT_DISABLED_SENSORS,
            {
                'sim_left_ir_enabled': 'True',
                'sim_left_ir_image_topic': 'left_ir/image',
                'sim_left_ir_camera_info_topic': 'left_ir/camera_info',
                'sim_left_ir_triggered': 'True',
            },
            'orbbec_gemini335le: left infrared trigger topic must not be empty',
            id='split-left-infrared',
        ),
        pytest.param(
            'orbbec_gemini335le_split_macro.xacro',
            'orbbec_gemini335le',
            SPLIT_DISABLED_SENSORS,
            {
                'sim_right_ir_enabled': 'True',
                'sim_right_ir_image_topic': 'right_ir/image',
                'sim_right_ir_camera_info_topic': 'right_ir/camera_info',
                'sim_right_ir_triggered': 'True',
            },
            'orbbec_gemini335le: right infrared trigger topic must not be empty',
            id='split-right-infrared',
        ),
        pytest.param(
            'orbbec_gemini335le_rgbd_macro.xacro',
            'orbbec_gemini335le_rgbd',
            RGBD_DISABLED_SENSORS,
            {'sim_rgbd_enabled': 'True', 'sim_rgbd_base_topic': 'rgbd', 'sim_rgbd_triggered': 'True'},
            'orbbec_gemini335le_rgbd: RGB-D trigger topic must not be empty',
            id='rgbd',
        ),
        pytest.param(
            'orbbec_gemini335le_rgbd_macro.xacro',
            'orbbec_gemini335le_rgbd',
            RGBD_DISABLED_SENSORS,
            {
                'sim_left_ir_enabled': 'True',
                'sim_left_ir_image_topic': 'left_ir/image',
                'sim_left_ir_camera_info_topic': 'left_ir/camera_info',
                'sim_left_ir_triggered': 'True',
            },
            'orbbec_gemini335le_rgbd: left infrared trigger topic must not be empty',
            id='rgbd-left-infrared',
        ),
        pytest.param(
            'orbbec_gemini335le_rgbd_macro.xacro',
            'orbbec_gemini335le_rgbd',
            RGBD_DISABLED_SENSORS,
            {
                'sim_right_ir_enabled': 'True',
                'sim_right_ir_image_topic': 'right_ir/image',
                'sim_right_ir_camera_info_topic': 'right_ir/camera_info',
                'sim_right_ir_triggered': 'True',
            },
            'orbbec_gemini335le_rgbd: right infrared trigger topic must not be empty',
            id='rgbd-right-infrared',
        ),
    ],
)
def test_wrappers_reject_empty_trigger_topics_in_triggered_mode(
    tmp_path: Path,
    macro_filename: str,
    macro_name: str,
    disabled_arguments: dict[str, str],
    arguments: dict[str, str],
    message: str,
) -> None:
    result = _run_wrapper(
        tmp_path, macro_filename=macro_filename, macro_name=macro_name, arguments=disabled_arguments | arguments
    )

    _assert_fatal(result, message)
