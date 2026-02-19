# How to Compute Inertia with MeshLab

## 1. Measure geometry in MeshLab

1. Open your STL in MeshLab.
2. Run:
   - `Filters -> Quality Measures and Computations -> Compute Geometric Measures`
3. From the log, copy:
   - `Mesh Volume = V`
   - `Center of Mass = (cx, cy, cz)`
   - `Inertia Tensor` (`Ixx, Iyy, Izz, Ixy, Ixz, Iyz`)

## 2. Important context: MeshLab assumes `rho = 1`

- MeshLab computes inertia with **uniform density** `rho = 1`.
- That means the reported inertia is in a reference mass scale, not yet using your real mass.
- In that reference scale:
  - `m_ref = rho * V = 1 * V = V`
- So the `V` above is specifically the **Mesh Volume reported by MeshLab**.

## 3. Compute the scale factor for your real mass

Let:

- `m_real` = real total mass of the physical object (kg, from a scale)
- `m_ref` = reference mass used by the inertia reported in MeshLab
- `V` = MeshLab `Mesh Volume`

General formula:

```text
s = m_real / m_ref
```

For MeshLab (with `rho = 1`), `m_ref = V`, so:

```text
s = m_real / V
```

## 4. Scale the full inertia tensor

Scale **all** tensor terms by `s`:

```text
I_real = s * I_meshlab
```

So:

- `Ixx_real = s * Ixx_meshlab`
- `Iyy_real = s * Iyy_meshlab`
- `Izz_real = s * Izz_meshlab`
- `Ixy_real = s * Ixy_meshlab`
- `Ixz_real = s * Ixz_meshlab`
- `Iyz_real = s * Iyz_meshlab`

## 5. Put values into URDF/Xacro

- `mass` = `m_real`
- `inertial origin xyz` = MeshLab center of mass `(cx, cy, cz)`
- `inertia` = scaled tensor components

## 6. Recommended mesh validity check

Run:

- `Filters -> Quality Measures and Computations -> Compute Topological Measures`

Prefer:

- `Boundary Edges = 0`
- `Non Manifold Edges = 0`
- `Non Manifold Vertices = 0`

This improves confidence in volume/inertia results.
