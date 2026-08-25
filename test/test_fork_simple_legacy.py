import shutil
import subprocess
from pathlib import Path

import pytest
from geometry_migration_helpers import assert_fatal, expanded_root, run_macro, source_test_env

LEGACY_MESH = 'robotics_description/meshes/extras/fork_simple/fork_simple_closed_tines.stl'


def _legacy_wrapper_arguments(**overrides: str) -> dict[str, str]:
    return {
        'scale': '1.0 1.0 1.0',
        'limits': '-0.10 0.10 1.0 100.0',
        'joint_parent_fr_root_fr': '0 0 0 0 0 0',
    } | overrides


def _legacy_helper_arguments(**overrides: str) -> dict[str, str]:
    return {
        'mesh': LEGACY_MESH,
        'mesh_scale': '1.0 1.0 1.0',
        'tine_size': '1.20 0.12 0.06',
        'tine_separation': '0.25',
        'tine_union_size_x': '0.03',
        'limits': '-0.10 0.10 1.0 100.0',
        'joint_parent_fr_root_fr': '0 0 0 0 0 0',
    } | overrides


def test_current_and_legacy_fork_macros_can_be_included_together(tmp_path: Path) -> None:
    """The renamed legacy entry point no longer collides with the current fork_simple macro."""
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    test_xacro = tmp_path / 'fork_current_and_legacy_validation.xacro'
    test_xacro.write_text(
        """<?xml version="1.0"?>
<robot name="fork_current_and_legacy_validation" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find robotics_description)/urdf/extras/fork_simple/fork_simple_macro.xacro"/>
  <xacro:include filename="$(find robotics_description)/urdf/extras/fork_simple/fork_simple_legacy_macro.xacro"/>
  <link name="base_link"/>
  <xacro:fork_simple name="fork_current"
                     parent_frame="base_link"
                     limits="-0.10 0.10 1.0 100.0"
                     joint_parent_fr_root_fr="0 0 0 0 0 0"/>
  <xacro:fork_simple_legacy name="fork_legacy"
                            parent_frame="base_link"
                            limits="-0.10 0.10 1.0 100.0"
                            joint_parent_fr_root_fr="0 1 0 0 0 0"/>
</robot>
""",
        encoding='utf-8',
    )
    result = subprocess.run(
        [xacro_path, str(test_xacro)], capture_output=True, text=True, check=False, env=source_test_env(tmp_path)
    )
    root = expanded_root(result)

    assert root.find("./link[@name='fork_current_root_link']") is not None
    assert root.find("./link[@name='fork_legacy_root_link']") is not None


@pytest.mark.parametrize('scale', ['1 1', '1 1 1 1'])
def test_legacy_wrapper_rejects_invalid_scale_arity(tmp_path: Path, scale: str) -> None:
    result = run_macro(
        tmp_path,
        macro_file='urdf/extras/fork_simple/fork_simple_legacy_macro.xacro',
        macro_name='fork_simple_legacy',
        arguments=_legacy_wrapper_arguments(scale=scale),
    )

    assert_fatal(result, "fork_simple_legacy: scale must be 'sx sy sz'")


@pytest.mark.parametrize('scale', ['0 1 1', '1 -1 1'])
def test_legacy_wrapper_rejects_non_positive_scale(tmp_path: Path, scale: str) -> None:
    result = run_macro(
        tmp_path,
        macro_file='urdf/extras/fork_simple/fork_simple_legacy_macro.xacro',
        macro_name='fork_simple_legacy',
        arguments=_legacy_wrapper_arguments(scale=scale),
    )

    assert_fatal(result, 'fork_simple_legacy: scale values must be greater than zero')


@pytest.mark.parametrize('mesh', ['', '   '])
def test_legacy_helper_rejects_blank_mesh(tmp_path: Path, mesh: str) -> None:
    result = run_macro(
        tmp_path,
        macro_file=('urdf/extras/fork_simple/generic_macros/fork_simple_links_joints_legacy_macro.xacro'),
        macro_name='fork_simple_links_joints_legacy',
        arguments=_legacy_helper_arguments(mesh=mesh),
    )

    assert_fatal(result, 'fork_simple_links_joints_legacy: mesh must not be empty')


@pytest.mark.parametrize('transform', ['0 0 0 0 0', '0 0 0 0 0 0 0'])
def test_legacy_helper_rejects_invalid_parent_transform(tmp_path: Path, transform: str) -> None:
    result = run_macro(
        tmp_path,
        macro_file=('urdf/extras/fork_simple/generic_macros/fork_simple_links_joints_legacy_macro.xacro'),
        macro_name='fork_simple_links_joints_legacy',
        arguments=_legacy_helper_arguments(joint_parent_fr_root_fr=transform),
    )

    assert_fatal(result, "fork_simple_links_joints_legacy: joint_parent_fr_root_fr must be 'x y z roll pitch yaw'")


def test_legacy_helper_requires_mass_with_inertial_elements(tmp_path: Path) -> None:
    result = run_macro(
        tmp_path,
        macro_file=('urdf/extras/fork_simple/generic_macros/fork_simple_links_joints_legacy_macro.xacro'),
        macro_name='fork_simple_links_joints_legacy',
        arguments=_legacy_helper_arguments(mass='   ', inertial_elements='1 0 0 1 0 1', center_of_mass='0 0 0'),
    )

    assert_fatal(result, 'fork_simple_links_joints_legacy: mass is required with inertial_elements')
