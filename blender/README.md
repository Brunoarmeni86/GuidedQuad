# GuidedQuad Blender Trial + Tracker — experimental V0.3.9

An **installable Blender 4.2+ Extension** for testing the independent GuidedQuad C++ core. It is **not** an automatic all-over quad remesher and **does not depend on** Blender Sculpt Extended.

The add-on generates **one guided four-sided all-quad patch** from four open curve objects, optionally projects its interior onto an evaluated mesh, checks quad collisions, and saves a **local JSON diagnostics report** you can share after inspecting it.

## Get the trial builds

Go to [GuidedQuad GitHub Actions](https://github.com/Brunoarmeni86/GuidedQuad/actions) and open the latest successful **GuidedQuad C++ tests** run for branch `feature/blender-trial-tracker-v039-20261120`.

Download from the run's **Artifacts** section:

1. `guidedquad-blender-trial-v039`: outer GitHub artifact archive containing `guidedquad_trial_v0.3.9.zip`. **Extract the outer artifact**, but **keep** the nested `guidedquad_trial_v0.3.9.zip` intact for installation.
2. `guidedquad-cli-windows-latest` if on Windows; choose the Linux/macOS artifact on those systems. Extract `guidedquad_cli.exe` (Windows) or `guidedquad_cli` (macOS/Linux) to a durable folder. On Unix, mark the executable as executable if needed (`chmod +x guidedquad_cli`).

Alternatively build the current C++ source using CMake as detailed in the root README and run `python scripts/package_blender_trial.py` to create the installable add-on zip.

## Installation

In standard Blender 4.2+ choose **Edit → Preferences → Get Extensions → Install from Disk** (wording may vary by version) and select the inner `guidedquad_trial_v0.3.9.zip`. Confirm and enable **GuidedQuad Trial + Tracker**. It appears in the viewport **N-panel → GuidedQuad** tab.

Specify the **Native Engine** executable using the folder/file control. The Python extension itself requires no Blender rebuild, but **quad generation will not work without the matching compiled C++ executable**.

## First-run smoke test (recommended)

After installing the extension and pointing **Native Engine** to the downloaded `guidedquad_cli.exe`, open the **GuidedQuad** N-panel in Object Mode and click **Create Demo Patch**. The add-on creates a small curved 16×16 reference surface and **four preassigned POLY curves** with matching corners. Then click **Generate GuidedQuad Trial**. A second all-quad object should appear with its interior projected toward the curved reference surface. Finally check **Report Folder** for a new JSON report. This is the quickest way to confirm Blender version compatibility, native execution, output import, and tracker saving together.

If the operation fails, the JSON will still include `status: "failed"` and a brief error. Keep that report for troubleshooting.

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
