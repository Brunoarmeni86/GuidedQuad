# Clean-room comparison against licensed Exoside QuadRemesher

## Principle

Analyze public documentation and observable **inputs and outputs** of a legally installed and licensed Exoside build; do not decompile proprietary routines or use internal undocumented interfaces. Preserve separate source code history. No QuadRemesher engine or library is included.

## Benchmark corpus (to be supplied or generated)

1. Flat plane with a drawn diagonal hard guide and a soft direction guide.
2. Cylindrical tube with a circumferential loop (hard guide) and lengthwise flow.
3. Organic closed surface with two nested eye rings and a mouth loop.
4. Joint/bent limb with paint-based high- and low-density zones.
5. Irregular nonmanifold/degenerate geometry to assess error diagnostics.

For each *valid* corpus asset, compare the identical source mesh and target quad count. Record Exoside version, host, guide configuration, materials, adaptive sizing, actual output quad count and machine specs. As of 20 Nov 2026 Exoside documents engine 1.4.3 (released 15 Oct 2026); do not treat the previously inspected 1.2 DLL as the current benchmark engine. Official changelog: https://exoside.com/quadremesher/quadremesher-troubleshooting/ . GuidedQuad already generates single four-sided constrained patches, but must reach full automatic quad output on the same assets before global retopology comparisons become meaningful.

## Metrics and priorities

- **Hard guide geometric deviation:** maximum/mean surface distance from target curves to output mesh edge paths, normalized by bounding-box diagonal.
- **Guide connectivity:** expected connected loops actually present; count broken/missing loops.
- **Pole placement:** count valence-3/5 and out-of-region extraordinary vertices; avoid eyes/mouth deformation loops.
- **Flow alignment:** angular error modulo 90 degrees against reference tangent field.
- **Density target:** output quads total and local area distribution compared with red/blue map.
- **Mesh integrity:** all-quad fraction, manifoldness, winding consistency, flipped/inverted elements, self intersections.
- **Geometry:** source/retopo distance, aspect ratio, minimum/maximum interior quad angle.
- **Performance:** wall time, peak RAM and reproducibility at identical settings.

No quality or speed superiority over Exoside is claimed. Existing V0.3.x internal synthetic collision timings are not Exoside comparisons.
