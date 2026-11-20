# GuidedQuad

**Independent, constraint-first quadrangulation engine — experimental V0.3 branch**

GuidedQuad aims to produce precise, controllable all-quad retopology from artist-defined flow, hard boundaries and local density. The C++ core is independent of any custom Blender build. This experimental branch includes a limited Blender trial addon with a local feedback tracker, not a full automatic retopology extension.

> **Current status:** `main` is the V0.1 guide extractor. This experimental branch contains V0.2 constrained four-sided all-quad patches plus V0.3 guarded BVH surface reprojection. It is **not an automatic remesher** for arbitrary meshes. A separate diagnostic Blender addon alpha is now available in this experimental branch.

## What works now

- C++17 CMake library and command-line tool with no external dependencies.
- Extract region boundaries from per-triangle Face Set IDs.
- Extract continuous isocontours from per-vertex scalar fields, interpolated along triangle edges (a future Polypaint adapter).
- Detect open and closed guide paths, guide junctions and input non-manifold edges.
- Retain anchors to source vertices and source triangle edges.
- Write the resulting polylines as Wavefront OBJ `l` elements.
- Generate real quad faces within one four-sided guide patch, with hard corner preservation.
- Experimental CPU 4-RoSy face-direction propagation with weighted soft guides and locked face orientations (separate from actual hard quad-edge constraints).
- Optionally project only the patch interior onto a source triangulated surface using a CPU BVH with inversion guard and backtracking.
- Unit tests and an end-to-end projected OBJ export under `tests/`.

## Build and test

Requirements: C++17 compiler and CMake 3.16 or later.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## CLI demonstration

```sh
./build/guidedquad_cli facesets tests/square.obj tests/square_facesets.txt guides_facesets.obj
./build/guidedquad_cli contours tests/square.obj tests/square_scalar.txt guides_contour.obj 0
./build/guidedquad_cli flow tests/square.obj tests/square_flow.txt preview_flow.obj
./build/guidedquad_cli patch-loop tests/square.obj flat_quads.obj 0 1 2 3 8 8
./build/guidedquad_cli project-loop tests/square.obj tests/curved_surface.obj projected_quads.obj 0 1 2 3 8 8 2 1
```

On Windows with a multi-config CMake generator, the executable may be under `build/Release/guidedquad_cli.exe`.

The OBJ input must consist of triangle faces with positive indices. The Face Set file contains one region integer **per triangle**; the scalar file contains one number **per vertex**.

The `flow` command reads zero-based per-triangle guide vectors (`triangleIndex dx dy dz weight locked`) and exports **OBJ line directions only**, not a quad mesh. Face Set/Polypaint-to-flow conversion, global cross-field optimization, singularity-aware patch layout and a completed automatic remesher remain future work. See [cross-field notes](docs/CROSS_FIELD.md) and [comparison plan](docs/BENCHMARK_PLAN.md).

Full [V0.3 projection algorithm, limits and benchmarks](docs/PROJECTION_V03.md). The four-sided patch solver is documented in [V0.2 solver notes](docs/SOLVER_V02.md).

## First Blender trial and feedback tracker

The experimental Python addon is in [blender/guidedquad_trial](blender/guidedquad_trial).
It can validate quad collisions, extract sculpt Face Set guides and create a
four-sided quad patch preview, optionally projected onto a reference surface.
**It cannot yet automatically retopologize arbitrary organic meshes.**

On a fully green branch CI run, download the GuidedQuad_Blender_Trial_addon
artifact plus the matching Windows, macOS or Linux CLI artifact from the
**same run**. See [Blender trial installation](blender/README.md) and
[feedback test cases](blender/TRIAL_TEST_PLAN.md).

The local tracker writes one JSON report per operation. Exporting the ZIP is
manual: no uploaded data, scenes, OBJ geometry, textures or screenshots are
included. Python syntax and mocked-Blender bridge tests run in CI; a genuine
interactive Blender session has not yet been acceptance-tested.

The engine also has procedural curved-mesh tests at 24, 48 and 72 segments,
with multiscale bumps and local/broad deformations; these are **not scans**.
## Roadmap

See [Issue #1 — GuidedQuad roadmap](https://github.com/Brunoarmeni86/GuidedQuad/issues/1) and [architecture](docs/ARCHITECTURE.md).

Key future milestones: stock Blender Face Sets/Polypaint input, stroke centerline versus region-boundary modes, editable hard/soft constraints, directional 4-RoSy flow fields, independent adaptive density, constrained patch solver, all-quad output, local re-solving, UV/color/detail transfer and measured comparisons with Exoside/QuadriFlow.

## License and research references

Original project source: MIT — see [LICENSE](LICENSE).

Research references (no code copied into this V0.1): [Instant Meshes](https://github.com/wjakob/instant-meshes), [QuadriFlow](https://github.com/hjwdzh/QuadriFlow), [Geometry Central](https://github.com/nmwsharp/geometry-central), [Directional](https://github.com/avaxman/Directional), [QuadWild](https://github.com/nicopietroni/quadwild). Any future external dependency must undergo an explicit license and build review.

This repository is the primary home for GuidedQuad. The initial C++ reference code was moved from an isolated experimental branch of Blender Sculpt Extended; the released Sculpt Extended build was not changed.
