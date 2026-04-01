#!/usr/bin/env python3
"""Generate a simple thin-walled rectangular platform-body STL using CadQuery.

The generated body is not a solid block. It is a closed shell built from six
plates of constant thickness. Each plate is positioned so the final outer
dimensions match the requested X, Y, and Z sizes exactly. The plates do not
protrude beyond the requested bounding box.
"""

from __future__ import annotations

import argparse
from pathlib import Path

try:
    import cadquery as cq
except ImportError as exc:  # pragma: no cover
    raise SystemExit('cadquery is required for this generator. Install it with: pip install cadquery') from exc


body_len_x = 1.044
body_len_y = 0.650
body_len_z = 0.235
wall_thickness = 0.001


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the platform-body mesh generator."""
    parser = argparse.ArgumentParser(description='Generate a thin-walled rectangular platform-body STL using CadQuery.')
    parser.add_argument('--body-len-x', type=float, default=body_len_x, help='Body size in X in m.')
    parser.add_argument('--body-len-y', type=float, default=body_len_y, help='Body size in Y in m.')
    parser.add_argument('--body-len-z', type=float, default=body_len_z, help='Body size in Z in m.')
    parser.add_argument('--wall-thickness', type=float, default=wall_thickness, help='Thickness of each plate in m.')
    parser.add_argument(
        '--output',
        type=Path,
        default=Path(__file__).resolve().with_name('rectangular_body.stl'),
        help='Output STL file path.',
    )
    return parser.parse_args()


def add_box(
    body: cq.Workplane, x_min: float, x_max: float, y_min: float, y_max: float, z_min: float, z_max: float
) -> cq.Workplane:
    """Union one axis-aligned box into the current CadQuery solid."""
    lx = x_max - x_min
    ly = y_max - y_min
    lz = z_max - z_min
    cx = (x_min + x_max) / 2.0
    cy = (y_min + y_max) / 2.0
    cz = (z_min + z_max) / 2.0
    box = cq.Workplane('XY').box(lx, ly, lz, centered=(True, True, True)).translate((cx, cy, cz))
    return body.union(box)


def build_platform_body(
    *, body_len_x_m: float, body_len_y_m: float, body_len_z_m: float, wall_thickness_m: float
) -> cq.Workplane:
    """Build the platform body as a six-plate shell centered at the geometric origin."""
    x_min = -body_len_x_m / 2.0
    x_max = body_len_x_m / 2.0
    y_min = -body_len_y_m / 2.0
    y_max = body_len_y_m / 2.0
    z_min = -body_len_z_m / 2.0
    z_max = body_len_z_m / 2.0

    inner_x_min = x_min + wall_thickness_m
    inner_x_max = x_max - wall_thickness_m
    inner_y_min = y_min + wall_thickness_m
    inner_y_max = y_max - wall_thickness_m
    inner_z_min = z_min + wall_thickness_m
    inner_z_max = z_max - wall_thickness_m

    body = cq.Workplane('XY')

    # Bottom and top plates.
    body = add_box(body, x_min, x_max, y_min, y_max, z_min, inner_z_min)
    body = add_box(body, x_min, x_max, y_min, y_max, inner_z_max, z_max)

    # Left and right plates.
    body = add_box(body, x_min, x_max, y_min, inner_y_min, inner_z_min, inner_z_max)
    body = add_box(body, x_min, x_max, inner_y_max, y_max, inner_z_min, inner_z_max)

    # Front and rear plates.
    body = add_box(body, x_min, inner_x_min, inner_y_min, inner_y_max, inner_z_min, inner_z_max)
    body = add_box(body, inner_x_max, x_max, inner_y_min, inner_y_max, inner_z_min, inner_z_max)

    return body


def main() -> None:
    """Parse arguments, build the shell, and export the STL."""
    args = parse_args()

    if args.body_len_x <= 0.0 or args.body_len_y <= 0.0 or args.body_len_z <= 0.0:
        raise SystemExit('All body dimensions must be positive.')

    if args.wall_thickness <= 0.0:
        raise SystemExit('wall_thickness must be positive.')

    if args.wall_thickness * 2.0 >= min(args.body_len_x, args.body_len_y, args.body_len_z):
        raise SystemExit('wall_thickness is too large for the requested body dimensions.')

    body = build_platform_body(
        body_len_x_m=args.body_len_x,
        body_len_y_m=args.body_len_y,
        body_len_z_m=args.body_len_z,
        wall_thickness_m=args.wall_thickness,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(body, str(args.output))
    print(
        f'Wrote {args.output} '
        f'(args: body_len_x={args.body_len_x}, body_len_y={args.body_len_y}, '
        f'body_len_z={args.body_len_z}, wall_thickness={args.wall_thickness}).'
    )


if __name__ == '__main__':
    main()
