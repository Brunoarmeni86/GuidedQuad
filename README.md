# GuidedQuad

**Independent, constraint-first quadrangulation engine — experimental V0.1**

GuidedQuad aims to produce precise, controllable all-quad retopology from artist-defined flow, hard boundaries and local density. The engine is designed to run independently of any custom Blender build. A stock Blender extension is planned, but **not yet implemented**.

> **Current status:** V0.1 is a guide extraction prototype. It generates anchored guide polylines, **not quad meshes**. Do not install or advertise it as a working remesher yet.

## What works now

- C++17 CMake library and command-line tool with no external dependencies.
- Extract region boundaries from per-triangle Face Set IDs.
- Extract continuous isocontours from per-vertex scalar fields, interpolated along triangle edges (a future Polypaint adapter).
- Detect open and closed guide paths, guide junctions and input non-manifold edges.
- Retain anchors to source vertices and source triangle edges.
- Write the resulting polylines as Wavefront OBJ `l` elements.
- Unit-test examples under `tests/`.

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
```

On Windows with a multi-config CMake generator, the executable may be under `build/Release/guidedquad_cli.exe`.

The OBJ input must consist of triangle faces with positive indices. The Face Set file contains one region integer **per triangle**; the scalar file contains one number **per vertex**.

## Roadmap

See [Issue #1 — GuidedQuad roadmap](https://github.com/Brunoarmeni86/GuidedQuad/issues/1) and [architecture](docs/ARCHITECTURE.md).

Key future milestones: stock Blender Face Sets/Polypaint input, stroke centerline versus region-boundary modes, editable hard/soft constraints, directional 4-RoSy flow fields, independent adaptive density, constrained patch solver, all-quad output, local re-solving, UV/color/detail transfer and measured comparisons with Exoside/QuadriFlow.

## License and research references

Original project source: MIT — see [LICENSE](LICENSE).

Research references (no code copied into this V0.1): [Instant Meshes](https://github.com/wjakob/instant-meshes), [QuadriFlow](https://github.com/hjwdzh/QuadriFlow), [Geometry Central](https://github.com/nmwsharp/geometry-central), [Directional](https://github.com/avaxman/Directional), [QuadWild](https://github.com/nicopietroni/quadwild). Any future external dependency must undergo an explicit license and build review.

This repository is the primary home for GuidedQuad. The initial C++ reference code was moved from an isolated experimental branch of Blender Sculpt Extended; the released Sculpt Extended build was not changed.
