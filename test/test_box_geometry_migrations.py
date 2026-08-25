import shutil
import subprocess
from pathlib import Path

import pytest
from geometry_migration_helpers import (
    PACKAGE_ROOT,
    assert_fatal,
    expanded_root,
    float_attribute,
    geometry,
    run_macro,
    source_test_env,
)

LIDAR_SIMULATION_ARGUMENTS = {
    'sim_enabled': 'False',
    'sim_update_rate': '10.0',
    'sim_hor_fov_deg': '-90 90',
    'sim_hor_res_deg': '0.5',
    'sim_dist_span': '0.1 30.0',
}
REALSENSE_ARGUMENTS = {'use_sensor_frames_and_joints': 'False', 'joint_parent_fr_root_fr': '0 0 0 0 0 0'}
HOKUYO_ARGUMENTS = LIDAR_SIMULATION_ARGUMENTS | {'joint_parent_fr_root_fr': '0 0 0 0 0 0'}
LEUZE_ARGUMENTS = LIDAR_SIMULATION_ARGUMENTS | {'joint_parent_fr_root_fr': '0 0 0 0 0 0'}
SICK_ARGUMENTS = LIDAR_SIMULATION_ARGUMENTS | {'joint_parent_fr_root_fr': '0 0 0 0 0 0'}

BOX_COMPONENTS = [
    pytest.param(
        'urdf/sensors/cameras/realsense_d435_links_joints_macro.xacro',
        'realsense_d435_links_joints',
        'component_link',
        REALSENSE_ARGUMENTS,
        (0.02505, 0.09, 0.025),
        (-0.008225, -0.0175, 0.0),
        0.072,
        'robotics_description/meshes/sensors/cameras/realsense_d435/mesh.stl',
        'robotics_description/meshes/sensors/cameras/realsense_d435/mesh.stl',
        id='realsense-d435',
    ),
    pytest.param(
        'urdf/sensors/lidars/hokuyo_utm30lx_macro.xacro',
        'hokuyo_utm30lx',
        'component_root_link',
        HOKUYO_ARGUMENTS,
        (0.06, 0.06, 0.087),
        (0.0, 0.0, 0.0435),
        0.370,
        'robotics_description/meshes/sensors/lidars/hokuyo_utm_30lx/mesh.stl',
        'robotics_description/meshes/sensors/lidars/hokuyo_utm_30lx/mesh_low_res.stl',
        id='hokuyo-utm30lx',
    ),
    pytest.param(
        'urdf/sensors/lidars/leuze_rsl400_macro.xacro',
        'leuze_rsl400',
        'component_root_link',
        LEUZE_ARGUMENTS,
        (0.142645, 0.140254, 0.147911),
        (0.0, 0.0, 0.0739555),
        1.2,
        'robotics_description/meshes/sensors/lidars/leuze_rsl400/mesh.stl',
        'robotics_description/meshes/sensors/lidars/leuze_rsl400/mesh_low_res.stl',
        id='leuze-rsl400',
    ),
    pytest.param(
        'urdf/sensors/lidars/sick_s300_macro.xacro',
        'sick_s300',
        'component_root_link',
        SICK_ARGUMENTS,
        (0.106, 0.102, 0.152),
        (0.0, 0.0, 0.076),
        1.2,
        'robotics_description/meshes/sensors/lidars/sick_s300/mesh.stl',
        'robotics_description/meshes/sensors/lidars/sick_s300/mesh_low_res.stl',
        id='sick-s300',
    ),
]


@pytest.mark.parametrize(
    (
        'macro_file',
        'macro_name',
        'body_link',
        'base_arguments',
        'body_size',
        'body_origin',
        'body_mass',
        'detailed_mesh',
        'low_resolution_mesh',
    ),
    BOX_COMPONENTS,
)
@pytest.mark.parametrize(
    ('use_v_mesh', 'use_c_mesh', 'expected_visual', 'expected_collision'),
    [
        pytest.param('False', 'False', 'box', 'box', id='box-box'),
        pytest.param('True', 'True', 'mesh', 'mesh', id='mesh-mesh'),
        pytest.param('True', 'False', 'mesh', 'box', id='mesh-box'),
        pytest.param('False', 'True', 'box', 'mesh', id='box-mesh'),
    ],
)
def test_box_components_select_visual_and_collision_independently(
    tmp_path: Path,
    macro_file: str,
    macro_name: str,
    body_link: str,
    base_arguments: dict[str, str],
    body_size: tuple[float, float, float],
    body_origin: tuple[float, float, float],
    body_mass: float,
    detailed_mesh: str,
    low_resolution_mesh: str,
    use_v_mesh: str,
    use_c_mesh: str,
    expected_visual: str,
    expected_collision: str,
) -> None:
    del body_mass, detailed_mesh, low_resolution_mesh
    geometry_arguments = {'use_v_mesh': use_v_mesh, 'use_c_mesh': use_c_mesh}
    if macro_name != 'realsense_d435_links_joints':
        geometry_arguments |= {'v_mesh_use_low_res': 'True', 'c_mesh_use_low_res': 'True'}

    root = expanded_root(
        run_macro(tmp_path, macro_file=macro_file, macro_name=macro_name, arguments=base_arguments | geometry_arguments)
    )
    visual = geometry(root, body_link, 'visual')
    collision = geometry(root, body_link, 'collision')
    assert visual is not None
    assert collision is not None
    assert visual.tag == expected_visual
    assert collision.tag == expected_collision

    visual_origin = root.find(f"./link[@name='{body_link}']/visual/origin")
    collision_origin = root.find(f"./link[@name='{body_link}']/collision/origin")
    assert visual_origin is not None
    assert collision_origin is not None
    assert float_attribute(visual_origin, 'xyz') == pytest.approx(body_origin)
    assert float_attribute(collision_origin, 'xyz') == pytest.approx(body_origin)

    for shape in [visual, collision]:
        if shape.tag == 'box':
            assert float_attribute(shape, 'size') == pytest.approx(body_size)
        else:
            assert float_attribute(shape, 'scale') == pytest.approx((1.0, 1.0, 1.0))


@pytest.mark.parametrize(
    (
        'macro_file',
        'macro_name',
        'body_link',
        'base_arguments',
        'body_size',
        'body_origin',
        'body_mass',
        'detailed_mesh',
        'low_resolution_mesh',
    ),
    BOX_COMPONENTS,
)
def test_box_components_preserve_mesh_resolution_and_inertia(
    tmp_path: Path,
    macro_file: str,
    macro_name: str,
    body_link: str,
    base_arguments: dict[str, str],
    body_size: tuple[float, float, float],
    body_origin: tuple[float, float, float],
    body_mass: float,
    detailed_mesh: str,
    low_resolution_mesh: str,
) -> None:
    del body_size
    geometry_arguments = {'use_v_mesh': 'True', 'use_c_mesh': 'True'}
    if macro_name != 'realsense_d435_links_joints':
        geometry_arguments |= {'v_mesh_use_low_res': 'False', 'c_mesh_use_low_res': 'True'}

    root = expanded_root(
        run_macro(tmp_path, macro_file=macro_file, macro_name=macro_name, arguments=base_arguments | geometry_arguments)
    )
    visual = geometry(root, body_link, 'visual')
    collision = geometry(root, body_link, 'collision')
    assert visual is not None
    assert collision is not None
    assert visual.get('filename') == f'package://{detailed_mesh}'
    assert collision.get('filename') == f'package://{low_resolution_mesh}'

    inertial = root.find(f"./link[@name='{body_link}']/inertial")
    assert inertial is not None
    assert float(inertial.find('mass').get('value', '')) == pytest.approx(body_mass)
    assert float_attribute(inertial.find('origin'), 'xyz') == pytest.approx(body_origin)


@pytest.mark.parametrize(
    ('macro_file', 'macro_name', 'base_arguments', 'message'),
    [
        pytest.param(
            'urdf/sensors/cameras/realsense_d435_links_joints_macro.xacro',
            'realsense_d435_links_joints',
            REALSENSE_ARGUMENTS,
            "realsense_d435_links_joints: joint_parent_fr_root_fr must be 'x y z roll pitch yaw'",
            id='realsense-d435',
        ),
        pytest.param(
            'urdf/sensors/lidars/hokuyo_utm30lx_macro.xacro',
            'hokuyo_utm30lx',
            HOKUYO_ARGUMENTS,
            "hokuyo_utm30lx: joint_parent_fr_root_fr must be 'x y z roll pitch yaw'",
            id='hokuyo-utm30lx',
        ),
        pytest.param(
            'urdf/sensors/lidars/leuze_rsl400_macro.xacro',
            'leuze_rsl400',
            LEUZE_ARGUMENTS,
            "leuze_rsl400: joint_parent_fr_root_fr must be 'x y z roll pitch yaw'",
            id='leuze-rsl400',
        ),
        pytest.param(
            'urdf/sensors/lidars/sick_s300_macro.xacro',
            'sick_s300',
            SICK_ARGUMENTS,
            "sick_s300: joint_parent_fr_root_fr must be 'x y z roll pitch yaw'",
            id='sick-s300',
        ),
    ],
)
@pytest.mark.parametrize('transform', ['0 0 0 0 0', '0 0 0 0 0 0 0'])
def test_box_components_reject_invalid_parent_transform(
    tmp_path: Path, macro_file: str, macro_name: str, base_arguments: dict[str, str], message: str, transform: str
) -> None:
    result = run_macro(
        tmp_path,
        macro_file=macro_file,
        macro_name=macro_name,
        arguments=base_arguments | {'joint_parent_fr_root_fr': transform},
    )

    assert_fatal(result, message)


@pytest.mark.parametrize(
    ('macro_file', 'macro_name'),
    [
        pytest.param('urdf/sensors/lidars/hokuyo_utm30lx_macro.xacro', 'hokuyo_utm30lx'),
        pytest.param('urdf/sensors/lidars/leuze_rsl400_macro.xacro', 'leuze_rsl400'),
        pytest.param('urdf/sensors/lidars/sick_s300_macro.xacro', 'sick_s300'),
    ],
)
@pytest.mark.parametrize('topic', ['', '   '])
def test_box_lidars_reject_empty_topic_when_simulation_is_enabled(
    tmp_path: Path, macro_file: str, macro_name: str, topic: str
) -> None:
    result = run_macro(
        tmp_path,
        macro_file=macro_file,
        macro_name=macro_name,
        arguments=LIDAR_SIMULATION_ARGUMENTS
        | {'joint_parent_fr_root_fr': '0 0 0 0 0 0', 'sim_enabled': 'True', 'sim_topic': topic},
    )

    assert_fatal(result, f'{macro_name}: sim_topic must not be empty when simulation is enabled')


def _fork_arguments(**overrides: str) -> dict[str, str]:
    return {'joint_parent_fr_root_fr': '0 0 0 0 0 0', 'limits': '-0.1 0.1 1.0 100.0'} | overrides


@pytest.mark.parametrize(
    ('use_v_mesh', 'use_c_mesh', 'expected_visual', 'expected_collision'),
    [
        pytest.param('False', 'False', 'box', 'box', id='box-box'),
        pytest.param('True', 'True', 'mesh', 'mesh', id='mesh-mesh'),
        pytest.param('True', 'False', 'mesh', 'box', id='mesh-box'),
        pytest.param('False', 'True', 'box', 'mesh', id='box-mesh'),
    ],
)
def test_fork_tines_select_visual_and_collision_independently(
    tmp_path: Path, use_v_mesh: str, use_c_mesh: str, expected_visual: str, expected_collision: str
) -> None:
    root = expanded_root(
        run_macro(
            tmp_path,
            macro_file='urdf/extras/fork_simple/fork_simple_macro.xacro',
            macro_name='fork_simple',
            arguments=_fork_arguments(use_v_mesh=use_v_mesh, use_c_mesh=use_c_mesh),
        )
    )

    for side in ['left', 'right']:
        link_name = f'component_{side}_tine_link'
        assert geometry(root, link_name, 'visual').tag == expected_visual
        assert geometry(root, link_name, 'collision').tag == expected_collision


def test_fork_rejects_non_positive_carriage_height(tmp_path: Path) -> None:
    result = run_macro(
        tmp_path,
        macro_file='urdf/extras/fork_simple/fork_simple_macro.xacro',
        macro_name='fork_simple',
        arguments=_fork_arguments(carriage_size_z='0.0'),
    )

    assert_fatal(result, 'fork_simple: carriage_size_z must be greater than zero')


def _run_realsense_client_variant(
    tmp_path: Path, filename: str, replacements: dict[str, str] | None = None
) -> subprocess.CompletedProcess[str]:
    """Expand one package client, optionally replacing exact call attributes."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    source = (PACKAGE_ROOT / 'test' / 'test_xacros' / filename).read_text(encoding='utf-8')
    for old, new in (replacements or {}).items():
        assert old in source
        source = source.replace(old, new, 1)

    test_xacro = tmp_path / filename
    test_xacro.write_text(source, encoding='utf-8')
    return subprocess.run(
        [xacro_path, str(test_xacro)], capture_output=True, text=True, check=False, env=source_test_env(tmp_path)
    )


@pytest.mark.parametrize(
    ('filename', 'expected_sensors'),
    [
        pytest.param(
            'test_realsense_d435.xacro',
            {
                ('cam_color_frame', 'cam_color_gz_sensor', 'camera'),
                ('cam_depth_frame', 'cam_depth_gz_sensor', 'depth_camera'),
                ('cam_infra1_frame', 'cam_infra1_gz_sensor', 'camera'),
                ('cam_infra2_frame', 'cam_infra2_gz_sensor', 'camera'),
            },
            id='split-wrapper',
        ),
        pytest.param(
            'test_realsense_d435_rgbd.xacro',
            {
                ('cam_depth_frame', 'cam_rgbd_gz_sensor', 'rgbd_camera'),
                ('cam_infra1_frame', 'cam_infra1_gz_sensor', 'camera'),
                ('cam_infra2_frame', 'cam_infra2_gz_sensor', 'camera'),
            },
            id='rgbd-wrapper',
        ),
    ],
)
def test_realsense_wrapper_clients_create_expected_sensors(
    tmp_path: Path, filename: str, expected_sensors: set[tuple[str, str, str]]
) -> None:
    root = expanded_root(_run_realsense_client_variant(tmp_path, filename))
    actual_sensors = {
        (gazebo.get('reference'), sensor.get('name'), sensor.get('type'))
        for gazebo in root.findall('gazebo')
        for sensor in gazebo.findall('sensor')
    }
    assert actual_sensors == expected_sensors


@pytest.mark.parametrize(
    ('filename', 'attribute', 'message'),
    [
        pytest.param(
            'test_realsense_d435.xacro',
            'sim_color_image_topic="${rs_color_topic}"',
            'realsense_d435: color image and camera-info topics must not be empty',
            id='split-color',
        ),
        pytest.param(
            'test_realsense_d435.xacro',
            'sim_depth_image_topic="${rs_depth_topic}"',
            'realsense_d435: depth image and camera-info topics must not be empty',
            id='split-depth',
        ),
        pytest.param(
            'test_realsense_d435.xacro',
            'sim_infra1_image_topic="${rs_infra1_topic}"',
            'realsense_d435: infra1 image and camera-info topics must not be empty',
            id='split-infra1',
        ),
        pytest.param(
            'test_realsense_d435.xacro',
            'sim_infra2_image_topic="${rs_infra2_topic}"',
            'realsense_d435: infra2 image and camera-info topics must not be empty',
            id='split-infra2',
        ),
        pytest.param(
            'test_realsense_d435_rgbd.xacro',
            'sim_rgbd_base_topic="${rs_rgbd_base_topic}"',
            'realsense_d435_rgbd: RGB-D base topic must not be empty',
            id='rgbd',
        ),
        pytest.param(
            'test_realsense_d435_rgbd.xacro',
            'sim_infra1_image_topic="${rs_infra1_topic}"',
            'realsense_d435_rgbd: infra1 image and camera-info topics must not be empty',
            id='rgbd-infra1',
        ),
        pytest.param(
            'test_realsense_d435_rgbd.xacro',
            'sim_infra2_image_topic="${rs_infra2_topic}"',
            'realsense_d435_rgbd: infra2 image and camera-info topics must not be empty',
            id='rgbd-infra2',
        ),
    ],
)
def test_realsense_wrappers_reject_empty_topics_for_enabled_sensors(
    tmp_path: Path, filename: str, attribute: str, message: str
) -> None:
    result = _run_realsense_client_variant(tmp_path, filename, {attribute: attribute.split('=', 1)[0] + '="   "'})

    assert_fatal(result, message)


@pytest.mark.parametrize(
    ('filename', 'triggered_attribute', 'trigger_topic_attribute', 'message'),
    [
        pytest.param(
            'test_realsense_d435.xacro',
            'sim_depth_triggered="${rs_depth_triggered}"',
            'sim_depth_trigger_topic="${rs_depth_trigger_topic}"',
            'realsense_d435: depth trigger topic must not be empty',
            id='split-depth',
        ),
        pytest.param(
            'test_realsense_d435_rgbd.xacro',
            'sim_rgbd_triggered="${rs_rgbd_triggered}"',
            'sim_rgbd_trigger_topic="${rs_rgbd_trigger_topic}"',
            'realsense_d435_rgbd: RGB-D trigger topic must not be empty',
            id='rgbd',
        ),
    ],
)
def test_realsense_wrappers_reject_empty_trigger_topics_in_triggered_mode(
    tmp_path: Path, filename: str, triggered_attribute: str, trigger_topic_attribute: str, message: str
) -> None:
    triggered_name = triggered_attribute.split('=', 1)[0]
    trigger_topic_name = trigger_topic_attribute.split('=', 1)[0]
    result = _run_realsense_client_variant(
        tmp_path,
        filename,
        {triggered_attribute: triggered_name + '="True"', trigger_topic_attribute: trigger_topic_name + '=""'},
    )

    assert_fatal(result, message)
