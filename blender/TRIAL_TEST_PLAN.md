# GuidedQuad first manual trial: test plan and feedback IDs

**Do not begin manual testing until you have an addon ZIP and a matching CLI
artifact from the same green CI run.** This is an integration test, not a
claim of full automated retopology. Use a copy of each scene; no original
meshes are changed intentionally by these operators.

## Minimum environment

- Stock Blender 4.2 or newer, ideally a supported Blender 5.x release.
- Enable **GuidedQuad Trial & Feedback** from the installation ZIP.
- Select the matching `guidedquad_cli` executable from the same GitHub CI run.
- Set **Tracker folder**; leave **Include object names** OFF for privacy.
- After each operation the addon creates a JSON report; export a ZIP at the end
  and inspect it before optionally sharing.

## Manual cases (run one at a time)

| ID | Setup / action | Expected result |
|---|---|---|
| GQ-01 | On a default cube (all quad faces), run **Check Quad Collisions** | No detected self-intersection; tracker JSON marked success |
| GQ-02 | On a duplicated, partially intersecting disconnected quad object, check collisions | Warning `collision_found` and a JSON report; no Blender crash |
| GQ-03 | On a triangle-only object, invoke **Check Quad Collisions** | Clear error explaining all-quad input; report still created |
| GQ-04 | Sculpt mesh with at least two distinct Face Sets: **Extract Face Set Guides** | Curve with visible boundary paths and `guide_paths > 0` |
| GQ-05 | Mesh without a Face Set face attribute, extract guides | Clear missing-attribute error and a report |
| GQ-06 | Select one quad face in Edit Mode, return to Object Mode and create **Four-Corner Test Patch** at U=8,V=8 | Separate output mesh containing 64 quad faces; original unchanged |
| GQ-07 | Repeat GQ-06 with density U=16,V=12 | Output has 192 quad faces; boundaries coincide with selected guide face |
| GQ-08 | Use a simple separate reference mesh, enable **Project to reference**, then generate a patch | Projection runs or returns an explicit quality/guide error; captures CLI timings |
| GQ-09 | Leave CLI path empty or deliberately invalid, try a command | Clear error; JSON tracker report still created |
| GQ-10 | **Export Feedback ZIP** after several cases | ZIP contains JSON reports and README; **no OBJ, .blend, texture, screenshots or video** |
| GQ-11 | Rename a mesh and keep **Include object names** disabled | ZIP JSON contains geometry counts, but not object names |
| GQ-12 | Repeat a simple operation several times | Every run has a distinct report ID and no previously saved report is overwritten |

## Feedback quality and severity

- **Blocker**: crash, corrupt .blend, persistent UI failure or wrong CLI binary.
- **Critical**: silent mesh corruption, missed real intersection, false positive
  on a clean mesh, failed rollback or lost source data.
- **Major**: output topology/border differs from expected, significant speed
  degradation, error not recoverable without restarting Blender.
- **Minor**: unclear UI, awkward workflow, visual issue or slow noncritical step.

Please provide the test ID, whether it is reproducible, expected vs actual,
the generated feedback ZIP, and optionally an image/video **as a separate
attachment you choose to share**. If sharing geometry, first ensure it is
safe to disclose. No personal scene contents are transmitted automatically.

## Safety / accuracy limits

The C++ collision detector uses floating-point predicates and can be
imprecise around nearly tangent slivers. The Face Set guide operation uses
Blender's face-domain `.sculpt_face_set` attribute and may require adaptation
for a specific Blender version. Four-corner preview only remeshes one
explicitly selected four-sided region; global guide flow, automatic patch
layout, UV and color/detail transport are not implemented in this alpha.
