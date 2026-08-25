from pathlib import Path

import pytest
from geometry_migration_helpers import assert_fatal, expanded_root, float_attribute, geometry, run_macro

WHEEL_MESH = 'robotics_description/meshes/wheels/standard_wheels/wheel.dae'
AIRY_ARGUMENTS = {
    'joint_parent_fr_root_fr': '0 0 0 0 0 0',
    'sim_lidar_enabled': 'False',
    'sim_lidar_update_rate': '10.0',
    'sim_lidar_hor_fov_deg': '-180 180',
    'sim_lidar_hor_res_deg': '0.5',
    'sim_lidar_ver_fov_deg': '-15 15',
    'sim_lidar_ver_res_deg': '1.0',
    'sim_lidar_dist_span': '0.1 100.0',
    'sim_imu_enabled': 'False',
}
HELIOS_ARGUMENTS = {
    'joint_parent_fr_root_fr': '0 0 0 0 0 0',
    'sim_enabled': 'False',
    'sim_update_rate': '10.0',
    'sim_hor_fov_deg': '-180 180',
    'sim_hor_res_deg': '0.5',
    'sim_ver_fov_deg': '-15 15',
    'sim_ver_res_deg': '1.0',
    'sim_dist_span': '0.1 100.0',
}

ROBOSENSE_COMPONENTS = [
    pytest.param(
        'urdf/sensors/lidars/robosense_airy_macro.xacro',
        'robosense_airy',
        AIRY_ARGUMENTS,
        0.03,
        0.063,
        0.25,
        'robotics_description/meshes/sensors/lidars/robosense_airy/mesh.stl',
        'robotics_description/meshes/sensors/lidars/robosense_airy/mesh_low_res.stl',
        id='robosense-airy',
    ),
    pytest.param(
        'urdf/sensors/lidars/robosense_helios_16_macro.xacro',
        'robosense_helios_16',
        HELIOS_ARGUMENTS,
        0.05,
        0.1005,
        0.99,
        'robotics_description/meshes/sensors/lidars/robosense_helios_16/mesh.stl',
        'robotics_description/meshes/sensors/lidars/robosense_helios_16/mesh_low_res.dae',
        id='robosense-helios-16',
    ),
]


@pytest.mark.parametrize(
    ('macro_file', 'macro_name', 'base_arguments', 'radius', 'length', 'mass', 'detailed_mesh', 'low_resolution_mesh'),
    ROBOSENSE_COMPONENTS,
)
@pytest.mark.parametrize(
    ('use_v_mesh', 'use_c_mesh', 'expected_visual', 'expected_collision'),
    [
        pytest.param('False', 'False', 'cylinder', 'cylinder', id='cylinder-cylinder'),
        pytest.param('True', 'True', 'mesh', 'mesh', id='mesh-mesh'),
        pytest.param('True', 'False', 'mesh', 'cylinder', id='mesh-cylinder'),
        pytest.param('False', 'True', 'cylinder', 'mesh', id='cylinder-mesh'),
    ],
)
def test_robosense_selects_visual_and_collision_independently(
    tmp_path: Path,
    macro_file: str,
    macro_name: str,
    base_arguments: dict[str, str],
    radius: float,
    length: float,
    mass: float,
    detailed_mesh: str,
    low_resolution_mesh: str,
    use_v_mesh: str,
    use_c_mesh: str,
    expected_visual: str,
    expected_collision: str,
) -> None:
    del mass, detailed_mesh, low_resolution_mesh
    root = expanded_root(
        run_macro(
            tmp_path,
            macro_file=macro_file,
            macro_name=macro_name,
            arguments=base_arguments
            | {
                'use_v_mesh': use_v_mesh,
                'use_c_mesh': use_c_mesh,
                'v_mesh_use_low_res': 'True',
                'c_mesh_use_low_res': 'True',
            },
        )
    )
    visual = geometry(root, 'component_root_link', 'visual')
    collision = geometry(root, 'component_root_link', 'collision')
    assert visual is not None
    assert collision is not None
    assert visual.tag == expected_visual
    assert collision.tag == expected_collision

    for shape in [visual, collision]:
        if shape.tag == 'cylinder':
            assert float(shape.get('radius', '')) == pytest.approx(radius)
            assert float(shape.get('length', '')) == pytest.approx(length)


@pytest.mark.parametrize(
    ('macro_file', 'macro_name', 'base_arguments', 'radius', 'length', 'mass', 'detailed_mesh', 'low_resolution_mesh'),
    ROBOSENSE_COMPONENTS,
)
def test_robosense_collision_uses_its_own_mesh_selection(
    tmp_path: Path,
    macro_file: str,
    macro_name: str,
    base_arguments: dict[str, str],
    radius: float,
    length: float,
    mass: float,
    detailed_mesh: str,
    low_resolution_mesh: str,
) -> None:
    del radius, length
    root = expanded_root(
        run_macro(
            tmp_path,
            macro_file=macro_file,
            macro_name=macro_name,
            arguments=base_arguments
            | {'use_v_mesh': 'True', 'v_mesh_use_low_res': 'False', 'use_c_mesh': 'True', 'c_mesh_use_low_res': 'True'},
        )
    )
    visual = geometry(root, 'component_root_link', 'visual')
    collision = geometry(root, 'component_root_link', 'collision')
    assert visual is not None
    assert collision is not None
    assert visual.get('filename') == f'package://{detailed_mesh}'
    assert collision.get('filename') == f'package://{low_resolution_mesh}'

    inertial = root.find("./link[@name='component_root_link']/inertial")
    assert inertial is not None
    assert float(inertial.find('mass').get('value', '')) == pytest.approx(mass)


@pytest.mark.parametrize(
    ('macro_file', 'macro_name', 'base_arguments'),
    [
        pytest.param(
            'urdf/sensors/lidars/robosense_airy_macro.xacro', 'robosense_airy', AIRY_ARGUMENTS, id='robosense-airy'
        ),
        pytest.param(
            'urdf/sensors/lidars/robosense_helios_16_macro.xacro',
            'robosense_helios_16',
            HELIOS_ARGUMENTS,
            id='robosense-helios-16',
        ),
    ],
)
@pytest.mark.parametrize('transform', ['0 0 0 0 0', '0 0 0 0 0 0 0'])
def test_robosense_rejects_invalid_parent_transform(
    tmp_path: Path, macro_file: str, macro_name: str, base_arguments: dict[str, str], transform: str
) -> None:
    result = run_macro(
        tmp_path,
        macro_file=macro_file,
        macro_name=macro_name,
        arguments=base_arguments | {'joint_parent_fr_root_fr': transform},
    )

    assert_fatal(result, f"{macro_name}: joint_parent_fr_root_fr must be 'x y z roll pitch yaw'")


@pytest.mark.parametrize('topic', ['', '   '])
def test_robosense_airy_rejects_empty_enabled_topics(tmp_path: Path, topic: str) -> None:
    lidar_result = run_macro(
        tmp_path,
        macro_file='urdf/sensors/lidars/robosense_airy_macro.xacro',
        macro_name='robosense_airy',
        arguments=AIRY_ARGUMENTS | {'sim_lidar_enabled': 'True', 'sim_lidar_base_topic': topic},
    )
    assert_fatal(lidar_result, 'robosense_airy: sim_lidar_base_topic must not be empty')

    imu_result = run_macro(
        tmp_path,
        macro_file='urdf/sensors/lidars/robosense_airy_macro.xacro',
        macro_name='robosense_airy',
        arguments=AIRY_ARGUMENTS | {'sim_imu_enabled': 'True', 'sim_imu_topic': topic},
    )
    assert_fatal(imu_result, 'robosense_airy: sim_imu_topic must not be empty')


@pytest.mark.parametrize('topic', ['', '   '])
def test_robosense_helios_rejects_empty_enabled_topic(tmp_path: Path, topic: str) -> None:
    result = run_macro(
        tmp_path,
        macro_file='urdf/sensors/lidars/robosense_helios_16_macro.xacro',
        macro_name='robosense_helios_16',
        arguments=HELIOS_ARGUMENTS | {'sim_enabled': 'True', 'sim_base_topic': topic},
    )

    assert_fatal(result, 'robosense_helios_16: sim_base_topic must not be empty')


WHEELS = [
    pytest.param(
        'urdf/wheels/wheel_macro.xacro',
        'wheel',
        {
            'radius': '0.1',
            'length': '0.05',
            'mass': '1.0',
            'rotation_joint_type': 'fixed',
            'joint_parent_fr_wheel_fr': '0 0 0 0 0 0',
            'sim_enabled': 'False',
            'sim_fdir1': '1 0 0',
            'sim_fdir1_frame': 'base_link',
        },
        id='wheel',
    ),
    pytest.param(
        'urdf/wheels/caster_wheel_macro.xacro',
        'caster_wheel',
        {
            'radius': '0.1',
            'length': '0.05',
            'mass': '1.0',
            'joint_parent_fr_wheel_fr': '0 0 0 0 0 0',
            'x_joint_steering_fr_rotation_fr': '0.05',
            'sim_enabled': 'False',
        },
        id='caster-wheel',
    ),
    pytest.param(
        'urdf/wheels/steerable_wheel_macro.xacro',
        'steerable_wheel',
        {
            'radius': '0.1',
            'length': '0.05',
            'mass': '1.0',
            'steering_joint_type': 'fixed',
            'rotation_joint_type': 'fixed',
            'joint_parent_fr_wheel_fr': '0 0 0 0 0 0',
            'sim_enabled': 'False',
        },
        id='steerable-wheel',
    ),
]


@pytest.mark.parametrize(('macro_file', 'macro_name', 'base_arguments'), WHEELS)
@pytest.mark.parametrize(
    ('v_mesh', 'expected_visual'),
    [pytest.param('', 'cylinder', id='primitive'), pytest.param(WHEEL_MESH, 'mesh', id='mesh')],
)
def test_wheels_select_visual_geometry_and_keep_primitive_collision(
    tmp_path: Path, macro_file: str, macro_name: str, base_arguments: dict[str, str], v_mesh: str, expected_visual: str
) -> None:
    root = expanded_root(
        run_macro(tmp_path, macro_file=macro_file, macro_name=macro_name, arguments=base_arguments | {'v_mesh': v_mesh})
    )
    visual = geometry(root, 'component_rotation_link', 'visual')
    collision = geometry(root, 'component_rotation_link', 'collision')
    assert visual is not None
    assert collision is not None
    assert visual.tag == expected_visual
    assert collision.tag == 'cylinder'

    if visual.tag == 'mesh':
        assert visual.get('filename') == f'package://{WHEEL_MESH}'
    else:
        assert float(visual.get('radius', '')) == pytest.approx(0.1)
        assert float(visual.get('length', '')) == pytest.approx(0.05)

    assert float(collision.get('radius', '')) == pytest.approx(0.1)
    assert float(collision.get('length', '')) == pytest.approx(0.05)
    visual_origin = root.find("./link[@name='component_rotation_link']/visual/origin")
    collision_origin = root.find("./link[@name='component_rotation_link']/collision/origin")
    assert float_attribute(visual_origin, 'rpy') == pytest.approx((1.5707963267948966, 0.0, 0.0))
    assert float_attribute(collision_origin, 'rpy') == pytest.approx((1.5707963267948966, 0.0, 0.0))


@pytest.mark.parametrize(('macro_file', 'macro_name', 'base_arguments'), WHEELS)
@pytest.mark.parametrize(
    ('field', 'value', 'message'),
    [
        pytest.param('radius', '0.0', 'radius must be greater than zero', id='zero-radius'),
        pytest.param('length', '-0.1', 'length must be greater than zero', id='negative-length'),
        pytest.param('mass', '0.0', 'mass must be greater than zero when inertial data is enabled', id='zero-mass'),
    ],
)
def test_wheels_reject_invalid_physical_properties(
    tmp_path: Path,
    macro_file: str,
    macro_name: str,
    base_arguments: dict[str, str],
    field: str,
    value: str,
    message: str,
) -> None:
    result = run_macro(
        tmp_path, macro_file=macro_file, macro_name=macro_name, arguments=base_arguments | {field: value}
    )

    assert_fatal(result, f'{macro_name}: {message}')


@pytest.mark.parametrize(('macro_file', 'macro_name', 'base_arguments'), WHEELS)
def test_wheels_allow_non_positive_mass_without_inertial_data(
    tmp_path: Path, macro_file: str, macro_name: str, base_arguments: dict[str, str]
) -> None:
    result = run_macro(
        tmp_path,
        macro_file=macro_file,
        macro_name=macro_name,
        arguments=base_arguments | {'mass': '-1.0', 'use_inertial': 'False'},
    )

    assert result.returncode == 0, result.stderr
