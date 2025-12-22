#!/usr/bin/env python3
import math
from pathlib import Path


def build_ring(r_outer: float, r_inner: float, height: float, segments: int = 32):
    """Build a closed ring mesh (cylindrical shell)."""
    hz = height / 2.0
    positions = []

    # Top outer ring (z = +hz)
    for k in range(segments):
        angle = 2.0 * math.pi * k / segments
        x = r_outer * math.cos(angle)
        y = r_outer * math.sin(angle)
        positions.append((x, y, hz))

    # Top inner ring (z = +hz)
    for k in range(segments):
        angle = 2.0 * math.pi * k / segments
        x = r_inner * math.cos(angle)
        y = r_inner * math.sin(angle)
        positions.append((x, y, hz))

    # Bottom outer ring (z = -hz)
    for k in range(segments):
        angle = 2.0 * math.pi * k / segments
        x = r_outer * math.cos(angle)
        y = r_outer * math.sin(angle)
        positions.append((x, y, -hz))

    # Bottom inner ring (z = -hz)
    for k in range(segments):
        angle = 2.0 * math.pi * k / segments
        x = r_inner * math.cos(angle)
        y = r_inner * math.sin(angle)
        positions.append((x, y, -hz))

    triangles = []
    N = segments

    # Top surface
    for k in range(N):
        kn = (k + 1) % N
        outer_k = k
        outer_kn = kn
        inner_k = N + k
        inner_kn = N + kn

        triangles.append((outer_k, outer_kn, inner_kn))
        triangles.append((outer_k, inner_kn, inner_k))

    # Bottom surface (reversed winding)
    for k in range(N):
        kn = (k + 1) % N
        outer_k = 2 * N + k
        outer_kn = 2 * N + kn
        inner_k = 3 * N + k
        inner_kn = 3 * N + kn

        triangles.append((outer_k, inner_kn, outer_kn))
        triangles.append((outer_k, inner_k, inner_kn))

    # Outer side wall
    for k in range(N):
        kn = (k + 1) % N
        top_k = k
        top_kn = kn
        bot_k = 2 * N + k
        bot_kn = 2 * N + kn

        triangles.append((top_k, bot_k, bot_kn))
        triangles.append((top_k, bot_kn, top_kn))

    # Inner side wall
    for k in range(N):
        kn = (k + 1) % N
        top_k = N + k
        top_kn = N + kn
        bot_k = 3 * N + k
        bot_kn = 3 * N + kn

        triangles.append((top_k, bot_kn, bot_k))
        triangles.append((top_k, top_kn, bot_kn))

    return positions, triangles


def build_cylinder(radius: float, height: float, segments: int = 32):
    """Closed cylinder with top/bottom caps and side wall."""
    hz = height / 2.0
    positions = []

    # Top ring
    for k in range(segments):
        angle = 2.0 * math.pi * k / segments
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        positions.append((x, y, hz))

    # Bottom ring
    for k in range(segments):
        angle = 2.0 * math.pi * k / segments
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        positions.append((x, y, -hz))

    # Centers
    idx_center_top = len(positions)
    positions.append((0.0, 0.0, hz))
    idx_center_bottom = len(positions)
    positions.append((0.0, 0.0, -hz))

    triangles = []
    N = segments

    # Top cap fan
    for k in range(N):
        kn = (k + 1) % N
        triangles.append((idx_center_top, k, kn))

    # Bottom cap fan (reversed)
    for k in range(N):
        kn = (k + 1) % N
        triangles.append((idx_center_bottom, kn + N, k + N))

    # Side wall
    for k in range(N):
        kn = (k + 1) % N
        top_k = k
        top_kn = kn
        bot_k = N + k
        bot_kn = N + kn

        triangles.append((top_k, top_kn, bot_kn))
        triangles.append((top_k, bot_kn, bot_k))

    return positions, triangles


def build_spoke(r_outer: float, height: float, thickness: float):
    """Rectangular bar from center to radius r_outer, spanning full height."""
    hz = height / 2.0
    t = thickness / 2.0

    x0, x1 = 0.0, r_outer
    y0, y1 = -t, t
    z0, z1 = -hz, hz

    positions = [
        (x0, y0, z0),  # 0
        (x1, y0, z0),  # 1
        (x1, y1, z0),  # 2
        (x0, y1, z0),  # 3
        (x0, y0, z1),  # 4
        (x1, y0, z1),  # 5
        (x1, y1, z1),  # 6
        (x0, y1, z1),  # 7
    ]

    triangles = []

    # Bottom
    triangles.append((0, 1, 2))
    triangles.append((0, 2, 3))

    # Top
    triangles.append((4, 6, 5))
    triangles.append((4, 7, 6))

    # Side x = x0
    triangles.append((0, 3, 7))
    triangles.append((0, 7, 4))

    # Side x = x1
    triangles.append((1, 5, 6))
    triangles.append((1, 6, 2))

    # Side y = y0
    triangles.append((0, 4, 5))
    triangles.append((0, 5, 1))

    # Side y = y1
    triangles.append((3, 2, 6))
    triangles.append((3, 6, 7))

    return positions, triangles


def build_wheel_unified():
    """Build a single-geometry wheel with 3 material regions."""
    r_outer = 0.15
    r_inner = 0.105
    h_ring = 0.10  # tire width
    h_disc = 0.06  # slightly smaller to avoid z-fighting
    h_spoke = h_ring  # same as tire width

    segments = 32

    ring_pos, ring_tri = build_ring(r_outer, r_inner, h_ring, segments)
    base_ring = 0

    disc_pos, disc_tri = build_cylinder(r_inner, h_disc, segments)
    base_disc = len(ring_pos)

    spoke_pos, spoke_tri = build_spoke(r_inner, h_spoke, thickness=0.009)
    base_spoke = base_disc + len(disc_pos)

    positions = ring_pos + disc_pos + spoke_pos

    ring_tri = [tuple(base_ring + i for i in tri) for tri in ring_tri]
    disc_tri = [tuple(base_disc + i for i in tri) for tri in disc_tri]
    spoke_tri = [tuple(base_spoke + i for i in tri) for tri in spoke_tri]

    return positions, ring_tri, disc_tri, spoke_tri


def generate_dae_text() -> str:
    positions, ring_tri, disc_tri, spoke_tri = build_wheel_unified()

    # Positions array
    float_values = []
    for x, y, z in positions:
        float_values.extend([f'{x:.6f}', f'{y:.6f}', f'{z:.6f}'])
    float_array_str = ' '.join(float_values)

    def tris_to_p(tris):
        return ' '.join(' '.join(str(i) for i in tri) for tri in tris)

    ring_p = tris_to_p(ring_tri)
    disc_p = tris_to_p(disc_tri)
    spoke_p = tris_to_p(spoke_tri)

    nverts = len(positions)

    dae = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor>
      <author>ChatGPT</author>
      <authoring_tool>ChatGPT Wheel Unified Generator</authoring_tool>
    </contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>

  <library_effects>
    <effect id="fx_black">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <diffuse><color>0 0 0 1</color></diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
    <effect id="fx_gray">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <diffuse><color>0.5 0.5 0.5 1</color></diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
    <effect id="fx_red">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <diffuse><color>1 0 0 1</color></diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>

  <library_images/>

  <library_materials>
    <material id="mat_black" name="mat_black">
      <instance_effect url="#fx_black"/>
    </material>
    <material id="mat_gray" name="mat_gray">
      <instance_effect url="#fx_gray"/>
    </material>
    <material id="mat_red" name="mat_red">
      <instance_effect url="#fx_red"/>
    </material>
  </library_materials>

  <library_geometries>
    <geometry id="wheel-mesh" name="wheel-mesh">
      <mesh>
        <source id="wheel-mesh-positions">
          <float_array id="wheel-mesh-positions-array" count="{nverts * 3}">
            {float_array_str}
          </float_array>
          <technique_common>
            <accessor source="#wheel-mesh-positions-array" count="{nverts}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="wheel-mesh-vertices">
          <input semantic="POSITION" source="#wheel-mesh-positions"/>
        </vertices>
        <triangles material="mat_black" count="{len(ring_tri)}">
          <input semantic="VERTEX" source="#wheel-mesh-vertices" offset="0"/>
          <p>{ring_p}</p>
        </triangles>
        <triangles material="mat_gray" count="{len(disc_tri)}">
          <input semantic="VERTEX" source="#wheel-mesh-vertices" offset="0"/>
          <p>{disc_p}</p>
        </triangles>
        <triangles material="mat_red" count="{len(spoke_tri)}">
          <input semantic="VERTEX" source="#wheel-mesh-vertices" offset="0"/>
          <p>{spoke_p}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>

  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="wheel-node" name="wheel-node" type="NODE">
        <matrix sid="transform">1 0 0 0  0 1 0 0  0 0 1 0  0 0 0 1</matrix>
        <instance_geometry url="#wheel-mesh">
          <bind_material>
            <technique_common>
              <instance_material symbol="mat_black" target="#mat_black"/>
              <instance_material symbol="mat_gray"  target="#mat_gray"/>
              <instance_material symbol="mat_red"   target="#mat_red"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>

  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
"""
    return dae


def main():
    dae_text = generate_dae_text()
    out_path = Path('wheel.dae')
    out_path.write_text(dae_text)
    print(f'Written: {out_path.resolve()}')


if __name__ == '__main__':
    main()
