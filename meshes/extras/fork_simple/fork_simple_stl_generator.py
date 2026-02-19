#!/usr/bin/env python3
"""Generate a simple fork STL using CadQuery boolean unions."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

try:
    import cadquery as cq
except ImportError as exc:  # pragma: no cover
    raise SystemExit('cadquery is required for this generator. Install it with: pip install cadquery') from exc

# Editable tine dimensions (meters)
tine_separation = 0.25
tine_len_x = 1.20
tine_len_y = 0.12
tine_len_z = 0.06

# Editable default dimensions (meters)
tine_union_len_x = 0.03


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for simple fork mesh generation."""
    script_dir = Path(__file__).resolve().parent
    default_output = script_dir.joinpath(f'fork_simple_{datetime.now():%Y%m%d}.stl')
    parser = argparse.ArgumentParser(description='Generate a simple fork STL using CadQuery boolean unions.')
    parser.add_argument(
        '--tine-union-len-x',
        dest='tine_union_len_x',
        type=float,
        default=tine_union_len_x,
        help='Tine-union size in X (m).',
    )
    parser.add_argument('--tine-len-z', dest='tine_len_z', type=float, default=tine_len_z, help='Tine size in Z (m).')
    parser.add_argument('--tine-separation', type=float, default=tine_separation, help='Tine center Y offset in m.')
    parser.add_argument('--tine-len-x', type=float, default=tine_len_x, help='Tine length in X in m.')
    parser.add_argument('--tine-len-y', type=float, default=tine_len_y, help='Tine width in Y in m.')
    parser.add_argument(
        '--output',
        type=Path,
        default=default_output,
        help='Output STL file path (default: same directory as this script).',
    )
    return parser.parse_args()


def add_box(
    fork_simple: cq.Workplane, x_min: float, x_max: float, y_min: float, y_max: float, z_min: float, z_max: float
) -> cq.Workplane:
    """Union an axis-aligned box into the current CadQuery solid."""
    lx = x_max - x_min
    ly = y_max - y_min
    lz = z_max - z_min
    cx = (x_min + x_max) / 2.0
    cy = (y_min + y_max) / 2.0
    cz = (z_min + z_max) / 2.0

    # This line builds a box from min/max bounds in two conceptual steps:
    #
    # 1) cq.Workplane('XY').box(lx, ly, lz, centered=(True, True, True))
    #    - Creates a box with side lengths (lx, ly, lz), centered at the origin of the final fork body frame (global
    #      model frame), not at a local frame attached to each individual box.
    #    - Before translation it occupies:
    #        X in [-lx/2, +lx/2], Y in [-ly/2, +ly/2], Z in [-lz/2, +lz/2]
    #
    # 2) .translate((cx, cy, cz))
    #    - Moves that centered box so its center is exactly at (cx, cy, cz).
    #    - After translation, occupied ranges become:
    #        X in [cx - lx/2, cx + lx/2]
    #        Y in [cy - ly/2, cy + ly/2]
    #        Z in [cz - lz/2, cz + lz/2]
    #
    # With bounds input:
    #   lx = x_max - x_min,  cx = (x_min + x_max)/2
    #   ly = y_max - y_min,  cy = (y_min + y_max)/2
    #   lz = z_max - z_min,  cz = (z_min + z_max)/2
    # then:
    #   x_min = cx - lx/2, x_max = cx + lx/2
    #   y_min = cy - ly/2, y_max = cy + ly/2
    #   z_min = cz - lz/2, z_max = cz + lz/2
    #
    # So this construction is exactly equivalent to defining a box spanning
    # [x_min, x_max] x [y_min, y_max] x [z_min, z_max], but expressed in CadQuery's "size + pose" style.
    box = cq.Workplane('XY').box(lx, ly, lz, centered=(True, True, True)).translate((cx, cy, cz))
    return fork_simple.union(box)


def build_simple_fork(
    tine_union_len_x_m: float, tine_sep_m: float, tine_len_x_m: float, tine_len_y_m: float, tine_len_z_m: float
) -> cq.Workplane:
    """Build fork solid as a union of tine union, tines, and mirrored load stops."""
    # Frame convention:
    # - Origin is at the rear face of the tine union, centered in Y, on the bottom plane.
    # - +X goes from the rear union plate toward the tine tips.
    # - +Y is the side where the "left tine" is built; the right tine is mirrored to -Y.
    # - +Z points upward.
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


def main() -> None:
    """Generate the fork mesh with CadQuery and export STL."""
    args = parse_args()

    # Basic argument validation: geometric lengths/offsets must be strictly positive.
    positive_params = {
        'tine_union_len_x': args.tine_union_len_x,
        'tine_len_z': args.tine_len_z,
        'tine_separation': args.tine_separation,
        'tine_len_x': args.tine_len_x,
        'tine_len_y': args.tine_len_y,
    }
    for name, value in positive_params.items():
        if value <= 0.0:
            raise ValueError(f'Invalid argument: {name} must be > 0.0 (got {value}).')

    fork_simple = build_simple_fork(
        tine_union_len_x_m=args.tine_union_len_x,
        tine_sep_m=args.tine_separation,
        tine_len_x_m=args.tine_len_x,
        tine_len_y_m=args.tine_len_y,
        tine_len_z_m=args.tine_len_z,
    )

    # Export tessellated STL from unified B-Rep.
    cq.exporters.export(fork_simple, str(args.output))

    print(
        f'Wrote {args.output} '
        f'(args: tine_union_len_x={args.tine_union_len_x}, '
        f'tine_len_z={args.tine_len_z}, '
        f'tine_separation={args.tine_separation}, '
        f'tine_len_x={args.tine_len_x}, '
        f'tine_len_y={args.tine_len_y}, '
        f'output={args.output}).'
    )


if __name__ == '__main__':
    main()
