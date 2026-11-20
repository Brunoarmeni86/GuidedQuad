# GuidedQuad

**Independent, constraint-first quadrangulation engine — experimental V0.3 branch**

GuidedQuad aims to produce controllable all-quad retopology from artist-defined guides, hard boundaries and local density. Its C++ engine runs independently of any custom Blender build. This experimental branch now includes an **installable Blender trial extension and a local JSON tracker**, supporting one guided four-sided patch, not whole-mesh automatic remeshing.

> **Current status:** `main` remains the V0.1 guide extractor. This experimental stacked branch includes constrained four-sided patches, guarded BVH reprojection, collision checker optimizations, and a **V0.3.9 installable Blender trial** with a native CLI executable delivered separately. It is **not yet an automatic remesher** for arbitrary meshes.

## What works now

- C++17 CMake library and command-line tool with no external dependencies.
- Extract region boundaries from per-triangle Face Set IDs.
- Extract continuous isocontours from per-vertex scalar fields, interpolated along triangle edges (a future Polypaint adapter).
- Detect open and closed guide paths, guide junctions and input non-manifold edges.
- Retain anchors to source vertices and source triangle edges.
- Write the resulting polylines as Wavefront OBJ `l` elements.
- Generate real quad faces within one four-sided guide patch, with hard corner preservation.
- Optionally project only the patch interior onto a source triangulated surface using a CPU BVH with inversion guard and backtracking.
- Unit tests and an end-to-end projected OBJ export under `tests/`.

## Blender trial and diagnostics (V0.3.9 branch)

**[Installation, guide setup, one-click demo and report instructions](blender/README.md).** The experiment ships as a Blender 4.2+ Python extension plus a separately compiled `guidedquad_cli` binary. GitHub Actions packages an inner `guidedquad_trial_v0.3.9.zip` (install via *Get Extensions → Install from Disk*) and uploads Windows/macOS/Linux native CLI executables. The extension supports four separate open POLY/BEZIER curve objects, optional reprojection onto an evaluated mesh, quad collision verification and a **local JSON report** on both successful and failed attempts. It has a **Create Demo Patch** helper.

You can also build the extension ZIP yourself:

```sh
python scripts/package_blender_trial.py
python -m unittest discover -s blender/tests -v
```

**Caveat:** Python tracker, native CLI and geometry tests run in CI. Interactive Blender integration still needs real-world testing. The trial does **not** transfer UVs/polypaint or generate a complete quad mesh automatically.

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
./build/guidedquad_cli patch-loop tests/square.obj flat_quads.obj 0 1 2 3 8 8
./build/guidedquad_cli project-loop tests/square.obj tests/curved_surface.obj projected_quads.obj 0 1 2 3 8 8 2 1
```

On Windows with a multi-config CMake generator, the executable may be under `build/Release/guidedquad_cli.exe`.

The OBJ input must consist of triangle faces with positive indices. The Face Set file contains one region integer **per triangle**; the scalar file contains one number **per vertex**.

Full [V0.3 projection algorithm, limits and benchmarks](docs/PROJECTION_V03.md). The four-sided patch solver is documented in [V0.2 solver notes](docs/SOLVER_V02.md).

## Roadmap

See [Issue #1 — GuidedQuad roadmap](https://github.com/Brunoarmeni86/GuidedQuad/issues/1) and [architecture](docs/ARCHITECTURE.md).

Key future milestones: stock Blender Face Sets/Polypaint input, stroke centerline versus region-boundary modes, editable hard/soft constraints, directional 4-RoSy flow fields, independent adaptive density, constrained patch solver, all-quad output, local re-solving, UV/color/detail transfer and measured comparisons with Exoside/QuadriFlow.

## License and research references

Original project source: MIT — see [LICENSE](LICENSE).

Research references (no code copied into this V0.1): [Instant Meshes](https://github.com/wjakob/instant-meshes), [QuadriFlow](https://github.com/hjwdzh/QuadriFlow), [Geometry Central](https://github.com/nmwsharp/geometry-central), [Directional](https://github.com/avaxman/Directional), [QuadWild](https://github.com/nicopietroni/quadwild). Any future external dependency must undergo an explicit license and build review.

This repository is the primary home for GuidedQuad. The initial C++ reference code was moved from an isolated experimental branch of Blender Sculpt Extended; the released Sculpt Extended build was not changed.
