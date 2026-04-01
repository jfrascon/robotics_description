#!/usr/bin/env python3
"""Generate the fork_simple STL variants using CadQuery geometry.

This script generates the current fork-tines mesh variants. It can generate the
closed-tines mesh and the open-tines mesh from the same geometry definition.

The rear fork body that joins both tines remains solid. Each tine is modeled as
a thin-walled box that is closed at the front. When `open_tines` is enabled,
that front face is moved `pocket_depth_x` meters toward the rear, while the
outer tine length stays unchanged.
"""

from __future__ import annotations

import argparse
from pathlib import Path

try:
    import cadquery as cq
except ImportError as exc:  # pragma: no cover
    raise SystemExit('cadquery is required for this generator. Install it with: pip install cadquery') from exc

# Editable tine dimensions (meters)
tine_len_x = 1.20
tine_len_y = 0.12
tine_len_z = 0.06
tine_separation = 0.25

# Editable default dimensions (meters)
# `tine_union_len_x` is the length in X of the rear block that joins both tines.
tine_union_len_x = 0.03

# Open-tine dimensions near the tine tip (meters)
pocket_depth_x = 0.10
tine_wall_thickness = 0.001


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the fork-tines mesh generator."""
    parser = argparse.ArgumentParser(description='Generate fork-tines STL variants using CadQuery geometry.')
    parser.add_argument('--tine-len-x', type=float, default=tine_len_x, help='Tine length in X in m.')
    parser.add_argument('--tine-len-y', type=float, default=tine_len_y, help='Tine width in Y in m.')
    parser.add_argument('--tine-len-z', type=float, default=tine_len_z, help='Tine size in Z (m).')
    parser.add_argument('--tine-separation', type=float, default=tine_separation, help='Tine center Y offset in m.')
    parser.add_argument('--tine-union-len-x', type=float, default=tine_union_len_x, help='Tine-union size in X (m).')
    parser.add_argument(
        '--pocket-depth-x',
        type=float,
        default=pocket_depth_x,
        help='Distance from the original tine front face to the shifted front face in open-tines mode (m).',
    )
    parser.add_argument(
        '--open-tines',
        action='store_true',
        help='Move the tine front face toward the rear. When omitted, the fork remains closed.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=None,
        help='Output STL file path. When omitted, the default name depends on --open-tines.',
    )
    return parser.parse_args()


def add_box(
    fork_simple: cq.Workplane, x_min: float, x_max: float, y_min: float, y_max: float, z_min: float, z_max: float
) -> cq.Workplane:
    """Union one axis-aligned box into the current CadQuery solid."""
    lx = x_max - x_min
    ly = y_max - y_min
    lz = z_max - z_min
    cx = (x_min + x_max) / 2.0
    cy = (y_min + y_max) / 2.0
    cz = (z_min + z_max) / 2.0
    box = cq.Workplane('XY').box(lx, ly, lz, centered=(True, True, True)).translate((cx, cy, cz))
    return fork_simple.union(box)


def build_simple_fork(
    tine_union_len_x_m: float, tine_sep_m: float, tine_len_x_m: float, tine_len_y_m: float, tine_len_z_m: float
) -> cq.Workplane:
    """Build the baseline simple fork solid as union of boxes."""
    x_min_tine_union = 0.0
    x_max_tine_union = tine_union_len_x_m
    y_min_tine_union = -tine_sep_m / 2.0
    y_max_tine_union = tine_sep_m / 2.0
    z_min_tine_union = 0.0
    z_max_tine_union = tine_len_z_m

    x_min_tine = 0.0
    x_max_tine = tine_len_x_m
    y_min_tine = tine_sep_m / 2.0
    y_max_tine = y_min_tine + tine_len_y_m
    z_min_tine = 0.0
    z_max_tine = tine_len_z_m

    x_min_load_stop = 0.0
    x_max_load_stop = tine_union_len_x_m
    y_min_load_stop = tine_sep_m / 2.0
    y_max_load_stop = y_min_tine + tine_len_y_m
    z_min_load_stop = tine_len_z_m
    z_max_load_stop = 2.0 * tine_len_z_m

    fork_simple = cq.Workplane('XY')
    fork_simple = add_box(
        fork_simple,
        x_min_tine_union,
        x_max_tine_union,
        y_min_tine_union,
        y_max_tine_union,
        z_min_tine_union,
        z_max_tine_union,
    )
    fork_simple = add_box(fork_simple, x_min_tine, x_max_tine, y_min_tine, y_max_tine, z_min_tine, z_max_tine)
    fork_simple = add_box(fork_simple, x_min_tine, x_max_tine, -y_max_tine, -y_min_tine, z_min_tine, z_max_tine)
    fork_simple = add_box(
        fork_simple,
        x_min_load_stop,
        x_max_load_stop,
        y_min_load_stop,
        y_max_load_stop,
        z_min_load_stop,
        z_max_load_stop,
    )
    fork_simple = add_box(
        fork_simple,
        x_min_load_stop,
        x_max_load_stop,
        -y_max_load_stop,
        -y_min_load_stop,
        z_min_load_stop,
        z_max_load_stop,
    )
    return fork_simple


def add_hollow_tine(
    fork_simple: cq.Workplane,
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
    rear_solid_len_x: float,
    wall_thickness: float,
    front_face_x_max: float,
) -> cq.Workplane:
    """Add one tine as a thin-walled box plus a solid rear attachment block."""
    rear_solid_x_max = x_min + rear_solid_len_x
    inner_y_min = y_min + wall_thickness
    inner_y_max = y_max - wall_thickness
    inner_z_min = z_min + wall_thickness
    inner_z_max = z_max - wall_thickness
    front_face_x_min = front_face_x_max - wall_thickness

    # Solid rear segment that attaches the hollow tine body to the fork trunk.
    fork_simple = add_box(fork_simple, x_min, rear_solid_x_max, y_min, y_max, z_min, z_max)

    # Bottom and top walls.
    fork_simple = add_box(fork_simple, x_min, x_max, y_min, y_max, z_min, inner_z_min)
    fork_simple = add_box(fork_simple, x_min, x_max, y_min, y_max, inner_z_max, z_max)

    # Left and right side walls.
    fork_simple = add_box(fork_simple, x_min, x_max, y_min, inner_y_min, inner_z_min, inner_z_max)
    fork_simple = add_box(fork_simple, x_min, x_max, inner_y_max, y_max, inner_z_min, inner_z_max)

    # Front face. In open-tines mode this face is moved toward the rear.
    fork_simple = add_box(fork_simple, front_face_x_min, front_face_x_max, y_min, y_max, z_min, z_max)
    return fork_simple


def build_open_tines_fork(
    tine_union_len_x_m: float,
    tine_sep_m: float,
    tine_len_x_m: float,
    tine_len_y_m: float,
    tine_len_z_m: float,
    open_tines: bool,
    pocket_depth_x_m: float,
    wall_thickness_m: float,
) -> cq.Shape:
    """Build the fork variant, optionally moving the tine front face rearward."""
    x_min_tine_union = 0.0
    x_max_tine_union = tine_union_len_x_m
    y_min_tine_union = -tine_sep_m / 2.0
    y_max_tine_union = tine_sep_m / 2.0
    z_min_tine_union = 0.0
    z_max_tine_union = tine_len_z_m

    x_min_tine = 0.0
    x_max_tine = tine_len_x_m
    y_min_tine = tine_sep_m / 2.0
    y_max_tine = y_min_tine + tine_len_y_m
    z_min_tine = 0.0
    z_max_tine = tine_len_z_m

    x_min_load_stop = 0.0
    x_max_load_stop = tine_union_len_x_m
    y_min_load_stop = tine_sep_m / 2.0
    y_max_load_stop = y_min_tine + tine_len_y_m
    z_min_load_stop = tine_len_z_m
    z_max_load_stop = 2.0 * tine_len_z_m

    front_face_x_max = tine_len_x_m if not open_tines else tine_len_x_m - pocket_depth_x_m

    fork_simple = cq.Workplane('XY')
    fork_simple = add_box(
        fork_simple,
        x_min_tine_union,
        x_max_tine_union,
        y_min_tine_union,
        y_max_tine_union,
        z_min_tine_union,
        z_max_tine_union,
    )
    fork_simple = add_hollow_tine(
        fork_simple,
        x_min=x_min_tine,
        x_max=x_max_tine,
        y_min=y_min_tine,
        y_max=y_max_tine,
        z_min=z_min_tine,
        z_max=z_max_tine,
        rear_solid_len_x=tine_union_len_x_m,
        wall_thickness=wall_thickness_m,
        front_face_x_max=front_face_x_max,
    )
    fork_simple = add_hollow_tine(
        fork_simple,
        x_min=x_min_tine,
        x_max=x_max_tine,
        y_min=-y_max_tine,
        y_max=-y_min_tine,
        z_min=z_min_tine,
        z_max=z_max_tine,
        rear_solid_len_x=tine_union_len_x_m,
        wall_thickness=wall_thickness_m,
        front_face_x_max=front_face_x_max,
    )
    fork_simple = add_box(
        fork_simple,
        x_min_load_stop,
        x_max_load_stop,
        y_min_load_stop,
        y_max_load_stop,
        z_min_load_stop,
        z_max_load_stop,
    )
    fork_simple = add_box(
        fork_simple,
        x_min_load_stop,
        x_max_load_stop,
        -y_max_load_stop,
        -y_min_load_stop,
        z_min_load_stop,
        z_max_load_stop,
    )
    return fork_simple.val()


def main() -> None:
    """Generate the selected fork-tines mesh variant and export STL."""
    args = parse_args()
    script_dir = Path(__file__).resolve().parent

    if args.output is None:
        default_output_name = 'fork_simple_open_tines.stl' if args.open_tines else 'fork_simple_closed_tines.stl'
        args.output = script_dir.joinpath(default_output_name)

    positive_params = {
        'tine_len_x': args.tine_len_x,
        'tine_len_y': args.tine_len_y,
        'tine_len_z': args.tine_len_z,
        'tine_separation': args.tine_separation,
        'tine_union_len_x': args.tine_union_len_x,
        'pocket_depth_x': args.pocket_depth_x,
    }
    for name, value in positive_params.items():
        if value <= 0.0:
            raise ValueError(f'Invalid argument: {name} must be > 0.0 (got {value}).')

    if args.pocket_depth_x >= args.tine_len_x:
        raise ValueError('pocket_depth_x must be smaller than tine_len_x.')

    if tine_wall_thickness <= 0.0:
        raise ValueError('tine_wall_thickness must be > 0.0.')

    if 2.0 * tine_wall_thickness >= args.tine_len_y:
        raise ValueError('2 * tine_wall_thickness must be smaller than tine_len_y.')

    if 2.0 * tine_wall_thickness >= args.tine_len_z:
        raise ValueError('2 * tine_wall_thickness must be smaller than tine_len_z.')

    if args.open_tines and args.pocket_depth_x <= tine_wall_thickness:
        raise ValueError('pocket_depth_x must be larger than tine_wall_thickness in open-tines mode.')

    fork_simple = build_open_tines_fork(
        tine_union_len_x_m=args.tine_union_len_x,
        tine_sep_m=args.tine_separation,
        tine_len_x_m=args.tine_len_x,
        tine_len_y_m=args.tine_len_y,
        tine_len_z_m=args.tine_len_z,
        open_tines=args.open_tines,
        pocket_depth_x_m=args.pocket_depth_x,
        wall_thickness_m=tine_wall_thickness,
    )

    cq.exporters.export(fork_simple, str(args.output))

    print(
        f'Wrote {args.output} '
        f'(args: tine_union_len_x={args.tine_union_len_x}, '
        f'tine_len_z={args.tine_len_z}, '
        f'tine_separation={args.tine_separation}, '
        f'tine_len_x={args.tine_len_x}, '
        f'tine_len_y={args.tine_len_y}, '
        f'open_tines={args.open_tines}, '
        f'pocket_depth_x={args.pocket_depth_x}, '
        f'output={args.output}).'
    )


if __name__ == '__main__':
    main()
