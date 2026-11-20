# GuidedQuad Blender Trial + Tracker — experimental V0.3.10

An **installable Blender 4.2+ Extension** for testing the independent GuidedQuad C++ core. It is **not** an automatic all-over quad remesher and **does not depend on** Blender Sculpt Extended.

The add-on generates **one guided four-sided all-quad patch** from four open curve objects, optionally projects its interior onto an evaluated mesh, checks quad collisions, and saves a **local JSON diagnostics report** you can share after inspecting it.

## Get the trial builds

Go to [GuidedQuad GitHub Actions](https://github.com/Brunoarmeni86/GuidedQuad/actions) and open the latest successful **GuidedQuad C++ tests** run for branch `feature/grease-pencil-annotation-guides-v0310`.

Download from the run's **Artifacts** section:

1. `guidedquad-blender-trial-v0310`: outer GitHub artifact archive containing `guidedquad_trial_v0.3.10.zip`. **Extract the outer artifact**, but **keep** the nested `guidedquad_trial_v0.3.10.zip` intact for installation.
2. `guidedquad-cli-windows-latest` if on Windows; choose the Linux/macOS artifact on those systems. Extract `guidedquad_cli.exe` (Windows) or `guidedquad_cli` (macOS/Linux) to a durable folder. On Unix, mark the executable as executable if needed (`chmod +x guidedquad_cli`).

Alternatively build the current C++ source using CMake as detailed in the root README and run `python scripts/package_blender_trial.py` to create the installable add-on zip.

## Installation

In standard Blender 4.2+ choose **Edit → Preferences → Get Extensions → Install from Disk** (wording may vary by version) and select the inner `guidedquad_trial_v0.3.10.zip`. Confirm and enable **GuidedQuad Trial + Tracker**. It appears in the viewport **N-panel → GuidedQuad** tab.

Specify the **Native Engine** executable using the folder/file control. The Python extension itself requires no Blender rebuild, but **quad generation will not work without the matching compiled C++ executable**.

## First-run smoke test (recommended)

After installing the extension and pointing **Native Engine** to the downloaded `guidedquad_cli.exe`, open the **GuidedQuad** N-panel in Object Mode and click **Create Demo Patch**. The add-on creates a small curved 16×16 reference surface and **four preassigned POLY curves** with matching corners. Then click **Generate GuidedQuad Trial**. A second all-quad object should appear with its interior projected toward the curved reference surface. Finally check **Report Folder** for a new JSON report. This is the quickest way to confirm Blender version compatibility, native execution, output import, and tracker saving together.

If the operation fails, the JSON will still include `status: "failed"` and a brief error. Keep that report for troubleshooting.

## New: Grease Pencil and Annotations as temporary guides (0.3.10)

Use **Guide Source → Grease Pencil** to read strokes from one Grease Pencil object, or **Guide Source → Notes / Annotations** to read viewport annotation strokes. The four stroke index fields are **zero-based** and map to Bottom, Right, Top and Left. The addon automatically fixes the direction of each chosen stroke so you do not need to draw in a particular direction. Assign four distinct strokes with endpoints at the desired corners; this is still a **single four-sided patch**, not an arbitrary line network solver.

Workflow:
1. Select the reference mesh. For Grease Pencil, use **Draw Mode → Stroke Placement → Surface / Project Onto Selected** where available and draw at least four separate open strokes in one object. For Notes, draw four separate Annotation strokes in the viewport.
2. Return to **Object Mode**, set the Guide Source in the GuidedQuad panel, and choose the Grease Pencil object if applicable.
3. Enter the zero-based stroke indices (0,1,2,3 if those are the first four usable strokes, in Bottom/Right/Top/Left order). Hidden layers, cyclic strokes, empty clicks and future keyframes are ignored. The addon reads the latest frame **not later** than the current timeline frame.
4. Optionally enable **Snap strokes to source** and set Maximum snap distance as a fraction of the source object's bounding diagonal. The snap uses **nearest surface point**, not a camera-view ray; on deep folds, close parallel sheets or thin cavities it can project onto the wrong side. Turn it off for strokes already drawn onto the correct surface.
5. Generate the patch. Stroke point data is converted directly into four in-memory polylines **only when the native engine needs the guides**. The original strokes remain untouched; no temporary Curve objects clutter the .blend.
6. The tracker JSON records the source type, chosen stroke indices and frames, selected point counts, and optional maximum snap displacement. Native CLI logs and full source geometry are not uploaded.

**Compatibility:** The reader handles GPv3's `frame.drawing.strokes` API (Blender 4.3+), legacy GP object frames (4.2), and Annotation `frame.strokes` through the context's annotation data. Blender changes its Grease Pencil API between releases; the pure-Python readers have unit tests, but the GP/Annotation UI path requires actual Blender testing. If the Annotation data is attached to another editor or not accessible from the active 3D viewport, the addon will report a missing source instead of silently guessing.

**Current limitations:** This phase does not assign an unlimited number of ink lines to a directional flow field, support multi-patch automatic layout, or automatically cut intersection/junctions between sketches. These need a future 4-RoSy/patch network stage. The sampling of dense hand-drawn strokes creates hard boundary knots; set U/V segments large enough to retain them or reduce/simplify the strokes first.

## Quick test — Blender object mode

1. Prepare a **reference mesh** with a local area that you want to approximate. Use a sculpt, subdivided model, or small test mesh. The mesh can contain n-gons; the addon exports its **evaluated triangulation**. The source object is not modified.
2. Create **four separate open Curve objects** describing the sides of one four-sided patch. Supported spline types: **POLY** and **BEZIER** with precisely one open spline per object. Each curve has at least two distinct sampled points.
3. Assign them in the panel as **Bottom / Right / Top / Left**. Each must describe a different side of the same four-sided region and their endpoints must touch at corners. The addon auto-reverses curve directions and snaps small endpoint mismatches inside the configured tolerance; it reports the original gap.
4. Choose **U Quads** and **V Quads** (start at 8–24 on a small surface). The patch solver preserves all sampled guide knots; your segment count must be sufficiently large to retain every input boundary knot. If you use many Bezier samples, increase U/V counts accordingly.
5. Set **Projection Steps** to 0 to test Coons generation without projection, then 1–2 for surface projection. Boundary curves must already lie on the source surface for a faithful result.
6. Leave **Check Output Collisions** enabled and choose **Report Folder**.
7. Press **Generate GuidedQuad Trial**. A new quad-only mesh object should appear, while your original mesh and curves remain unchanged.

The result is generated in **world coordinates** and has its own object. No UVs, polypaint, Face Set flow, high-resolution sculpt detail, or procedural automatic patch layout are transferred in this trial.

## Tracker — what to send back

The addon saves `guidedquad_<UTC-timestamp>.json` in the selected Report Folder **for successful and failed runs**. You can also press **Export Last Report**.

JSON fields include:
- Blender version and operating system family (but no automatic upload)
- U/V resolution, Bezier sampling, guide endpoint gap and chosen projection settings
- Evaluated input triangle count; output vertices and quads
- Native engine timing and total wall-clock duration in milliseconds
- Projection RMS, initial aspect and Jacobian, BVH builds/refits, dirty collision passes and queried faces
- **Post-projection output quality**: worst and p95 quad edge aspect ratio, minimum approximate face area, diagonal warpage angle and degenerate quad count
- Native collision-check exit code and success/failure reason

The tracker does **not** send anything over the network, export your blend file, or include the complete model. Review the JSON before sharing, particularly any error messages.

For useful feedback, test on three regions: **smooth curvature**, **deep concavity or close surface folds**, and **small hard features**. Note visually whether loops follow your guides and whether the surface projection stays on the correct side. Send the JSON reports plus a screenshot of the original and generated quad patch, or a sample mesh you explicitly choose to share.

## Known limitations

- **Not a full QuadRemesher competitor yet**: one explicit four-sided patch; no automatic guide extraction, 4-RoSy field, arbitrary closed-surface quadrangulation, UV/color transfer, or texture baking.
- The native engine is run synchronously; Blender UI may appear unresponsive for large meshes (user-selectable timeout).
- Curve boundaries are sampled and **hard-constrained**: if U/V segment counts are lower than sampled guide knots, generation fails with a tracker entry rather than silently discarding detail. Try simpler POLY splines for the first tests.
- Four corner positions must be geometrically consistent. This first adapter accepts only separate POLY/BEZIER curve objects, not vertex/edge selections inside Edit Mode.
- The native collision checker is floating-point approximate, not a watertight exact geometry guarantee.
- Executables are delivered separately for Windows/Linux/macOS through GitHub Actions; a ZIP with only Python code is **not a standalone remesher**.
- CI verifies C++, command-line roundtrips, Python syntax and pure-Python tracker/guide logic; it **does not yet run an automated Blender GUI integration test**. Your trial is the first real Blender environment validation.

For implementation detail see the root [README](../README.md) and [GuidedQuad roadmap](https://github.com/Brunoarmeni86/GuidedQuad/issues/1).
