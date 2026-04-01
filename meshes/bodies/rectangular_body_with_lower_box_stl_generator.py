#!/usr/bin/env python3
"""Generate a thin-walled rectangular body with an attached lower box.

The main body is a closed six-plate shell. A second shell is attached to the
bottom face of the main body. That lower shell is open on its top side because
the bottom face of the main body already closes the volume.

The final mesh is centered at the geometric origin of the complete assembly.
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
lower_len_x = 0.490
lower_len_y = 0.420
lower_len_z = 0.105
wall_thickness = 0.001


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the composite rectangular-body generator."""
    parser = argparse.ArgumentParser(
        description='Generate a thin-walled rectangular body with an attached lower box using CadQuery.'
    )
    parser.add_argument('--body-len-x', type=float, default=body_len_x, help='Main body size in X in m.')
    parser.add_argument('--body-len-y', type=float, default=body_len_y, help='Main body size in Y in m.')
    parser.add_argument('--body-len-z', type=float, default=body_len_z, help='Main body size in Z in m.')
    parser.add_argument('--lower-len-x', type=float, default=lower_len_x, help='Lower body size in X in m.')
    parser.add_argument('--lower-len-y', type=float, default=lower_len_y, help='Lower body size in Y in m.')
    parser.add_argument('--lower-len-z', type=float, default=lower_len_z, help='Lower body size in Z in m.')
    parser.add_argument('--wall-thickness', type=float, default=wall_thickness, help='Plate thickness in m.')
    parser.add_argument(
        '--output',
        type=Path,
        default=Path(__file__).resolve().with_name('rectangular_body_with_lower_box.stl'),
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


def add_closed_shell(
    body: cq.Workplane,
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
    wall_thickness_m: float,
) -> cq.Workplane:
    """Add a closed rectangular shell."""
    inner_x_min = x_min + wall_thickness_m
    inner_x_max = x_max - wall_thickness_m
    inner_y_min = y_min + wall_thickness_m
    inner_y_max = y_max - wall_thickness_m
    inner_z_min = z_min + wall_thickness_m
    inner_z_max = z_max - wall_thickness_m

    body = add_box(body, x_min, x_max, y_min, y_max, z_min, inner_z_min)
    body = add_box(body, x_min, x_max, y_min, y_max, inner_z_max, z_max)
    body = add_box(body, x_min, x_max, y_min, inner_y_min, inner_z_min, inner_z_max)
    body = add_box(body, x_min, x_max, inner_y_max, y_max, inner_z_min, inner_z_max)
    body = add_box(body, x_min, inner_x_min, inner_y_min, inner_y_max, inner_z_min, inner_z_max)
    body = add_box(body, inner_x_max, x_max, inner_y_min, inner_y_max, inner_z_min, inner_z_max)
    return body


def add_open_top_shell(
    body: cq.Workplane,
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
    wall_thickness_m: float,
) -> cq.Workplane:
    """Add a rectangular shell that is open on the top side."""
    inner_x_min = x_min + wall_thickness_m
    inner_x_max = x_max - wall_thickness_m
    inner_y_min = y_min + wall_thickness_m
    inner_y_max = y_max - wall_thickness_m
    inner_z_min = z_min + wall_thickness_m

    body = add_box(body, x_min, x_max, y_min, y_max, z_min, inner_z_min)
    body = add_box(body, x_min, x_max, y_min, inner_y_min, inner_z_min, z_max)
    body = add_box(body, x_min, x_max, inner_y_max, y_max, inner_z_min, z_max)
    body = add_box(body, x_min, inner_x_min, inner_y_min, inner_y_max, inner_z_min, z_max)
    body = add_box(body, inner_x_max, x_max, inner_y_min, inner_y_max, inner_z_min, z_max)
    return body


def build_body_with_lower_box(
    *,
    body_len_x_m: float,
    body_len_y_m: float,
    body_len_z_m: float,
    lower_len_x_m: float,
    lower_len_y_m: float,
    lower_len_z_m: float,
    wall_thickness_m: float,
) -> cq.Workplane:
    """Build the complete body centered at the geometric origin of the full assembly."""
    total_z = body_len_z_m + lower_len_z_m
    z_center = 0.0

    body_z_min = z_center - total_z / 2.0 + lower_len_z_m
    body_z_max = body_z_min + body_len_z_m
    lower_z_min = body_z_min - lower_len_z_m
    lower_z_max = body_z_min

    body_x_min = -body_len_x_m / 2.0
    body_x_max = body_len_x_m / 2.0
    body_y_min = -body_len_y_m / 2.0
    body_y_max = body_len_y_m / 2.0

    lower_x_min = -lower_len_x_m / 2.0
    lower_x_max = lower_len_x_m / 2.0
    lower_y_min = -lower_len_y_m / 2.0
    lower_y_max = lower_len_y_m / 2.0

    body = cq.Workplane('XY')
    body = add_closed_shell(
        body,
        x_min=body_x_min,
        x_max=body_x_max,
        y_min=body_y_min,
        y_max=body_y_max,
        z_min=body_z_min,
        z_max=body_z_max,
        wall_thickness_m=wall_thickness_m,
    )
    body = add_open_top_shell(
        body,
        x_min=lower_x_min,
        x_max=lower_x_max,
        y_min=lower_y_min,
        y_max=lower_y_max,
        z_min=lower_z_min,
        z_max=lower_z_max,
        wall_thickness_m=wall_thickness_m,
    )
    return body


def main() -> None:
    """Parse arguments, build the geometry, and export the STL."""
    args = parse_args()

    dims = [
        ('body_len_x', args.body_len_x),
        ('body_len_y', args.body_len_y),
        ('body_len_z', args.body_len_z),
        ('lower_len_x', args.lower_len_x),
        ('lower_len_y', args.lower_len_y),
        ('lower_len_z', args.lower_len_z),
    ]
    for name, value in dims:
        if value <= 0.0:
            raise SystemExit(f'{name} must be positive.')

    if args.wall_thickness <= 0.0:
        raise SystemExit('wall_thickness must be positive.')

    if args.wall_thickness * 2.0 >= min(
        args.body_len_x, args.body_len_y, args.body_len_z, args.lower_len_x, args.lower_len_y, args.lower_len_z
    ):
        raise SystemExit('wall_thickness is too large for the requested dimensions.')

    body = build_body_with_lower_box(
        body_len_x_m=args.body_len_x,
        body_len_y_m=args.body_len_y,
        body_len_z_m=args.body_len_z,
        lower_len_x_m=args.lower_len_x,
        lower_len_y_m=args.lower_len_y,
        lower_len_z_m=args.lower_len_z,
        wall_thickness_m=args.wall_thickness,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(body, str(args.output))
    print(
        f'Wrote {args.output} '
        f'(args: body_len_x={args.body_len_x}, body_len_y={args.body_len_y}, '
        f'body_len_z={args.body_len_z}, lower_len_x={args.lower_len_x}, '
        f'lower_len_y={args.lower_len_y}, lower_len_z={args.lower_len_z}, '
        f'wall_thickness={args.wall_thickness}).'
    )


if __name__ == '__main__':
    main()
