from pathlib import Path
import shutil
import subprocess

from geometry_migration_helpers import assert_fatal
from geometry_migration_helpers import source_test_env


def _run_fork_mockup(
    tmp_path: Path, *, limits: str, dynamics: str
) -> subprocess.CompletedProcess[str]:
    xacro_path = shutil.which('xacro')
    assert xacro_path, 'xacro is not installed'

    test_xacro = tmp_path / 'fork_mockup_validation.xacro'
    test_xacro.write_text(
        f"""<?xml version="1.0"?>
<robot name="fork_mockup_validation" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include
    filename="$(find robotics_description)/urdf/extras/forks/fork_mockup/fork_mockup_macro.xacro"/>
  <link name="base_link"/>
  <xacro:fork_mockup prefix="test_"
                     parent="base_link"
                     limits="{limits}"
                     dynamics="{dynamics}"
                     joint_parent_fr_fork_mockup_fr="0 0 0 0 0 0"/>
</robot>
""",
        encoding='utf-8',
    )

    return subprocess.run(
        [xacro_path, str(test_xacro)],
        capture_output=True,
        text=True,
        check=False,
        env=source_test_env(tmp_path),
    )


def test_fork_mockup_rejects_invalid_limits_arity(tmp_path: Path) -> None:
    result = _run_fork_mockup(tmp_path, limits='0 1 2', dynamics='10.0 1.0')

    assert_fatal(result, 'fork_mockup: limits must contain exactly 4 values')


def test_fork_mockup_rejects_invalid_dynamics_arity(tmp_path: Path) -> None:
    result = _run_fork_mockup(tmp_path, limits='-0.005 0.2 0.2 20000.0', dynamics='10.0')

    assert_fatal(result, 'fork_mockup: dynamics must contain exactly 2 values')


def test_fork_mockup_rejects_positive_lower_limit(tmp_path: Path) -> None:
    result = _run_fork_mockup(tmp_path, limits='0.01 0.2 0.2 20000.0', dynamics='10.0 1.0')

    assert_fatal(result, 'fork_mockup: limits lower value must be <= 0')


def test_fork_mockup_rejects_negative_upper_limit(tmp_path: Path) -> None:
    result = _run_fork_mockup(tmp_path, limits='-0.2 -0.01 0.2 20000.0', dynamics='10.0 1.0')

    assert_fatal(result, 'fork_mockup: limits upper value must be >= 0')


def test_fork_mockup_rejects_negative_damping(tmp_path: Path) -> None:
    result = _run_fork_mockup(tmp_path, limits='-0.005 0.2 0.2 20000.0', dynamics='-1.0 1.0')

    assert_fatal(result, 'fork_mockup: dynamics damping value must be >= 0')


def test_fork_mockup_rejects_negative_friction(tmp_path: Path) -> None:
    result = _run_fork_mockup(tmp_path, limits='-0.005 0.2 0.2 20000.0', dynamics='10.0 -1.0')

    assert_fatal(result, 'fork_mockup: dynamics friction value must be >= 0')
