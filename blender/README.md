# GuidedQuad Trial & Feedback — first isolated Blender adapter

**Experimental diagnostic build**, not a complete automated quadrangulator.
The addon is independent of Blender Sculpt Extended. It uses the external
`guidedquad_cli` C++ executable from the SAME GitHub Actions commit; it does
not patch or modify Blender internals.

## What the trial does

1. **Check Quad Collisions**: select an all-quad mesh in Object Mode. Export
   the vertices/faces to a temporary OBJ in world coordinates, run the native
   `check-quads` BVH checker, and save a diagnostic report. An intersecting
   mesh reports a warning rather than claiming success.
2. **Extract Face Set Guides**: on a sculpt mesh with the Face Set face
   attribute `.sculpt_face_set`, triangulate the mesh temporarily and run the
   native `facesets` guide extractor. Generated paths appear as Blender
   Curve splines. A mesh without Face Sets reports a clear error.
3. **Create Four-Corner Test Patch**: select exactly one quad face in Edit
   Mode, then switch to Object Mode; set U/V density and invoke the operator.
   The selected quad outlines a four-sided patch; the external C++ solver
   generates real quad geometry as a new mesh object. Optionally choose a
   separate **Reference Mesh** for projection onto a triangulated source
   surface. This is a constrained *single-chart* test and is **not** automatic
   guided retopology of an arbitrary organic mesh.
4. **Export Feedback ZIP**: create a local ZIP containing only JSON reports
   and the tracker privacy note. No mesh, `.blend`, OBJ, textures, screenshots
   or videos are included. **No automatic upload** occurs.

## Installation (once the CI branch is green)

- Open the CI run for the trial-addon branch. Download the
  `GuidedQuad_Blender_Trial_addon` artifact, which contains a ZIP with the
  `guidedquad_trial` Python package.
- Download the matching `guidedquad-cli-windows-latest` or
  `guidedquad-cli-ubuntu-latest` or `guidedquad-cli-macos-latest` artifact
  from the *same run*. Unzip the CLI executable to a directory you control.
- In stock **Blender 4.2+**, install the inner
  `GuidedQuad_Trial_Addon.zip` using Preferences > Add-ons > Install from Disk
  (or the equivalent extension installation command). This is a legacy
  Python addon for the trial; packaging as a manifest-based Blender Extension
  is future work.
- Enable **GuidedQuad Trial & Feedback**. In 3D View > Sidebar > GuidedQuad
  set the absolute path to `guidedquad_cli.exe` (Windows) or
  `guidedquad_cli` (macOS/Linux). On Unix, give the downloaded binary
  execution permission if necessary.
- Choose a tracker folder, or accept `~/.guidedquad/reports`. Run the
  diagnostics. Share the generated feedback ZIP **only when you choose**.

**Security:** this addon launches a user-selected local executable. Use only
an executable downloaded from a build you trust. The addon does not connect
to any web service or transmit reports.

## Reporting a test outcome

Each run records: addon version, Blender version, OS family, operation,
elapsed time, CLI SHA-256, CLI exit code, source vertex/face/quad counts,
selected numeric parameters and bounded console output/error.
Object names are **excluded by default**; there is an explicit opt-in
checkbox. No filesystem paths or vertex coordinates are stored in the JSON.
A generated report has a unique report ID.

For a useful failure report, optionally note: exact steps, Blender version,
what you expected, what happened and whether the problem is reproducible.
Screenshots and original meshes are separate files that you may choose to
share manually; **the feedback ZIP never collects them**.

## Limitations and verification status

The adapter's Python syntax and tracker unit tests run in GitHub Actions.
Native C++ functionality is tested across Linux, Windows and macOS, including
Linux sanitizers. These checks **do not constitute interactive Blender UI
testing** on all platforms; consider the addon an alpha build until a Blender
runtime smoke test and your feedback are available.

The current code does not yet support arbitrary automatic 4-RoSy guided
quadrangulation, Polypaint density guides, UV and color transfer, or
performance measurements directly within a Blender sculpt stroke. The C++
core and its independent tests remain the primary validated component.
