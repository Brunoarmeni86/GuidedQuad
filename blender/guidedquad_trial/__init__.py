# SPDX-License-Identifier: MIT
"""GuidedQuad isolated Blender diagnostic/trial adapter, not a global remesher."""
bl_info = {
    "name": "GuidedQuad Trial & Feedback",
    "author": "GuidedQuad contributors",
    "version": (0, 1, 0),
    "blender": (4, 2, 0),
    "location": "3D View > Sidebar > GuidedQuad",
    "description": "Experimental quad checker, Face Set guides, 4-corner patch preview and local feedback tracker",
    "category": "Mesh",
}

import datetime
import hashlib
import os
from pathlib import Path
import platform
import re
import subprocess
import tempfile
import time

import bpy
from bpy.props import BoolProperty, IntProperty, FloatProperty, PointerProperty, StringProperty
from bpy.types import Operator, Panel, PropertyGroup

from .tracker import save_report, export_bundle

ADDON_VERSION = "0.1.0-trial"
TIMEOUT_SECONDS = 120


def _report_directory(settings):
    value = settings.report_dir.strip()
    if not value:
        raise ValueError("Choose a report directory")
    if value.startswith("~"):
        value = os.path.expanduser(value)
    else:
        value = bpy.path.abspath(value)
    return Path(value).resolve()


def _cli_file(settings):
    value = settings.cli_path.strip()
    if not value:
        raise ValueError("Choose guidedquad_cli executable before running the engine")
    candidate = Path(bpy.path.abspath(value)).expanduser().resolve()
    if not candidate.is_file():
        raise ValueError("GuidedQuad CLI executable not found: check the configured path")
    return candidate


def _cli_hash(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            part = f.read(128 * 1024)
            if not part:
                break
            h.update(part)
    return h.hexdigest()


def _world_vertices(obj):
    mat = obj.matrix_world
    return [mat @ vert.co for vert in obj.data.vertices]


def _write_obj(path, positions, faces):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("# GuidedQuad temporary geometry; excluded from feedback bundle\n")
        for v in positions:
            f.write("v {:.17g} {:.17g} {:.17g}\n".format(v.x, v.y, v.z))
        for face in faces:
            f.write("f " + " ".join(str(int(i) + 1) for i in face) + "\n")


def _all_triangles(obj):
    if obj.type != "MESH":
        raise ValueError("Select a Mesh object")
    mesh = obj.data
    calculate = getattr(mesh, "calc_loop_triangles", None)
    if callable(calculate):
        calculate()
    triangles = [(tuple(t.vertices), int(t.polygon_index)) for t in mesh.loop_triangles]
    if not triangles:
        raise ValueError("Mesh has no triangulated faces")
    return _world_vertices(obj), triangles


def _write_triangulated_obj(path, obj):
    points, faces = _all_triangles(obj)
    _write_obj(path, points, [indices for indices, _ in faces])
    return faces


def _read_quad_obj(path):
    vertices, quads = [], []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            if parts[0] == "v":
                if len(parts) < 4:
                    raise ValueError("Invalid quad output vertex")
                vertices.append(tuple(float(x) for x in parts[1:4]))
            elif parts[0] == "f":
                if len(parts) != 5:
                    raise ValueError("Quad output contains a non-quad face")
                ids = tuple(int(part.split("/")[0]) - 1 for part in parts[1:])
                if any(i < 0 or i >= len(vertices) for i in ids):
                    raise ValueError("Quad output has an invalid vertex index")
                quads.append(ids)
    if not vertices or not quads:
        raise ValueError("Quad engine returned empty geometry")
    return vertices, quads


def _read_guide_obj(path):
    vertices, paths = [], []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            if parts[0] == "v" and len(parts) >= 4:
                vertices.append(tuple(float(v) for v in parts[1:4]))
            elif parts[0] == "l":
                ids = [int(x.split("/")[0]) - 1 for x in parts[1:]]
                if len(ids) >= 2:
                    if any(i < 0 or i >= len(vertices) for i in ids):
                        raise ValueError("Guide OBJ contains invalid indices")
                    paths.append(ids)
    return vertices, paths


def _focus_created_object(context, obj):
    # Do not depend on an OBJ import addon. Keep the new result visible in
    # the Outliner and active in the 3D View without changing source data.
    for selected in context.selected_objects:
        selected.select_set(False)
    obj.select_set(True)
    context.view_layer.objects.active = obj


def _create_quad_object(context, positions, quads):
    mesh = bpy.data.meshes.new("GuidedQuad Trial Patch")
    mesh.from_pydata(positions, [], quads)
    mesh.update()
    obj = bpy.data.objects.new("GuidedQuad_Trial_Patch", mesh)
    context.collection.objects.link(obj)
    obj.display_type = "SOLID"
    _focus_created_object(context, obj)
    return obj


def _create_guide_object(context, positions, paths):
    curve = bpy.data.curves.new("GuidedQuad FaceSet Guides", "CURVE")
    curve.dimensions = "3D"
    for indices in paths:
        closed = len(indices) > 2 and indices[0] == indices[-1]
        unique = indices[:-1] if closed else indices
        if len(unique) < 2:
            continue
        spline = curve.splines.new("POLY")
        spline.points.add(len(unique) - 1)
        for i, idx in enumerate(unique):
            xyz = positions[idx]
            spline.points[i].co = (xyz[0], xyz[1], xyz[2], 1.0)
        spline.use_cyclic_u = closed
    if not curve.splines:
        bpy.data.curves.remove(curve)
        return None
    obj = bpy.data.objects.new("GuidedQuad_FaceSet_Guides", curve)
    context.collection.objects.link(obj)
    _focus_created_object(context, obj)
    return obj


def _safe_console_text(message):
    # Reports never need raw user-home or temporary OBJ paths. Preserve
    # numeric diagnostics; redact the most common absolute path forms.
    message = message.replace(str(Path.home()), "<HOME>")
    message = re.sub(
        r"(?<![A-Za-z0-9])(?:[A-Za-z]:[\\/]|/(?:tmp|home|Users|mnt|var)/)[^\s\"']+",
        "<local-path>", message)
    return message[-2500:]


def _run_cli(executable, args):
    started = time.perf_counter()
    process = subprocess.run(
        [str(executable)] + [str(arg) for arg in args],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=TIMEOUT_SECONDS,
        check=False,
        shell=False,
    )
    elapsed_ms = (time.perf_counter() - started) * 1000
    return {
        "exit_code": int(process.returncode),
        "elapsed_ms": round(elapsed_ms, 3),
        "stdout": _safe_console_text(process.stdout),
        "stderr": _safe_console_text(process.stderr),
    }


def _object_summary(obj, settings):
    if obj is None or obj.type != "MESH":
        return {}
    out = {
        "vertices": len(obj.data.vertices),
        "faces": len(obj.data.polygons),
        "quad_faces": sum(len(p.vertices) == 4 for p in obj.data.polygons),
    }
    if settings.include_object_names:
        out["object_name"] = obj.name
    return out


def _record(settings, action, obj, status, started, parameters, **additional):
    data = {
        "addon_version": ADDON_VERSION,
        "blender_version": bpy.app.version_string,
        "platform": platform.system(),
        "action": action,
        "status": status,
        "duration_ms": round((time.perf_counter() - started) * 1000, 3),
        "object": _object_summary(obj, settings),
        "parameters": parameters,
    }
    data.update(additional)
    path = save_report(_report_directory(settings), data)
    settings.last_report = path.name
    settings.last_status = action + ": " + status
    return path


def _execute_action(operator, context, action, work):
    settings = context.scene.guidedquad_trial
    obj = context.active_object
    start = time.perf_counter()
    parameters = {}
    cli_info = {}
    try:
        executable = _cli_file(settings)
        cli_info["sha256"] = _cli_hash(executable)
        result = work(settings, obj, executable, parameters)
        cli_info.update(result.pop("cli", {}))
        status = result.pop("status", "success")
        location = _record(settings, action, obj, status, start, parameters,
                           cli=cli_info, results=result)
        operator.report({"WARNING"} if status != "success" else {"INFO"},
                        "GuidedQuad " + status + "; tracker: " + location.name)
        return {"FINISHED"}
    except Exception as exc:
        message = _safe_console_text("{}: {}".format(type(exc).__name__, str(exc)))
        try:
            _record(settings, action, obj, "error", start, parameters,
                    cli=cli_info, error=message[:1000])
        except Exception:
            pass
        settings.last_status = action + ": error"
        operator.report({"ERROR"}, message[:220])
        return {"CANCELLED"}


def _validate_quad_mesh(settings, obj, executable, parameters):
    if obj is None or obj.type != "MESH":
        raise ValueError("Select a quad Mesh object")
    faces = [tuple(p.vertices) for p in obj.data.polygons]
    if not faces or any(len(face) != 4 for face in faces):
        raise ValueError("Quad validation requires all-quad faces (no triangles or ngons)")
    parameters["quad_count"] = len(faces)
    with tempfile.TemporaryDirectory(prefix="guidedquad_") as tmp:
        quad_obj = Path(tmp) / "quads.obj"
        _write_obj(quad_obj, _world_vertices(obj), faces)
        cli = _run_cli(executable, ["check-quads", quad_obj])
    if cli["exit_code"] not in (0, 3):
        raise RuntimeError(cli["stderr"] or "quad collision engine failed")
    return {
        "status": "collision_found" if cli["exit_code"] == 3 else "success",
        "detected_collisions": (
            int(re.search(r"(\d+) detected pair", cli["stdout"]).group(1))
            if re.search(r"(\d+) detected pair", cli["stdout"]) else None
        ),
        "cli": cli,
    }


def _extract_faceset_guides(settings, obj, executable, parameters):
    if obj is None or obj.type != "MESH":
        raise ValueError("Select a sculpt Mesh with Face Sets")
    attribute = obj.data.attributes.get(".sculpt_face_set")
    if attribute is None or attribute.domain != "FACE":
        raise ValueError("No Face Set face attribute found (.sculpt_face_set)")
    with tempfile.TemporaryDirectory(prefix="guidedquad_") as tmp:
        tri_path = Path(tmp) / "triangles.obj"
        label_path = Path(tmp) / "regions.txt"
        output_path = Path(tmp) / "guides.obj"
        triangles = _write_triangulated_obj(tri_path, obj)
        labels = [int(attribute.data[polygon_id].value) for _, polygon_id in triangles]
        parameters.update({"triangles": len(triangles), "face_set_count": len(set(labels))})
        label_path.write_text("\n".join(str(x) for x in labels) + "\n",
                              encoding="utf-8")
        cli = _run_cli(executable, ["facesets", tri_path, label_path, output_path])
        if cli["exit_code"] != 0:
            raise RuntimeError(cli["stderr"] or "guide extraction failed")
        positions, paths = _read_guide_obj(output_path)
        if paths:
            _create_guide_object(bpy.context, positions, paths)
    return {"status": "success" if paths else "no_guides",
            "guide_paths": len(paths), "cli": cli}


def _single_selected_quad(obj):
    if obj is None or obj.type != "MESH":
        raise ValueError("Select a Mesh containing a four-sided guide face")
    if obj.mode != "OBJECT":
        raise ValueError("Switch to Object Mode after selecting one quad face")
    selected = [poly for poly in obj.data.polygons
                if poly.select and len(poly.vertices) == 4]
    if len(selected) != 1:
        if len(obj.data.polygons) == 1 and len(obj.data.polygons[0].vertices) == 4:
            return obj.data.polygons[0]
        raise ValueError("Exactly one quad face must be selected (in Edit Mode, then Object Mode)")
    return selected[0]


def _build_trial_patch(settings, obj, executable, parameters):
    face = _single_selected_quad(obj)
    points = _world_vertices(obj)
    corners = [points[k] for k in face.vertices]
    parameters.update({
        "u_segments": int(settings.u_segments),
        "v_segments": int(settings.v_segments),
        "project_to_reference": bool(settings.project_to_reference),
        "projection_iterations": int(settings.iterations),
    })
    with tempfile.TemporaryDirectory(prefix="guidedquad_") as tmp:
        border = Path(tmp) / "border.obj"
        output = Path(tmp) / "output.obj"
        _write_obj(border, corners, [(0, 1, 2), (0, 2, 3)])
        command = "patch-loop"
        args = [border, output, 0, 1, 2, 3, settings.u_segments, settings.v_segments]
        if settings.project_to_reference:
            source = settings.reference_mesh
            if source is None or source.type != "MESH":
                raise ValueError("Select a reference source Mesh for projection")
            reference = Path(tmp) / "reference.obj"
            _write_triangulated_obj(reference, source)
            command = "project-loop"
            args = [border, reference, output, 0, 1, 2, 3,
                    settings.u_segments, settings.v_segments,
                    settings.iterations, settings.strength]
        cli = _run_cli(executable, [command] + args)
        if cli["exit_code"] != 0:
            raise RuntimeError(cli["stderr"] or "patch solver rejected the guide")
        verts, quads = _read_quad_obj(output)
        _create_quad_object(bpy.context, verts, quads)
    return {"status": "success", "output_quads": len(quads),
            "output_vertices": len(verts), "cli": cli}


def _mesh_poll(self, obj):
    return obj.type == "MESH"


class GQTrialSettings(PropertyGroup):
    cli_path: StringProperty(name="GuidedQuad CLI", subtype="FILE_PATH",
                             description="Local guidedquad_cli.exe or guidedquad_cli")
    report_dir: StringProperty(name="Tracker folder", subtype="DIR_PATH",
                               default="~/.guidedquad/reports")
    include_object_names: BoolProperty(name="Include object names in tracker",
                                       description="Off by default for privacy", default=False)
    u_segments: IntProperty(name="Quad U", default=8, min=2, max=256)
    v_segments: IntProperty(name="Quad V", default=8, min=2, max=256)
    project_to_reference: BoolProperty(name="Project to reference", default=False)
    reference_mesh: PointerProperty(name="Reference Mesh", type=bpy.types.Object,
                                    poll=_mesh_poll)
    iterations: IntProperty(name="Iterations", default=2, min=0, max=20)
    strength: FloatProperty(name="Projection strength", default=1.0,
                            min=0.01, max=1.0)
    last_status: StringProperty(name="Last operation", default="No runs yet")
    last_report: StringProperty(name="Latest report", default="")


class GQ_OT_validate_quads(Operator):
    bl_idname = "guidedquad_trial.validate_quads"
    bl_label = "Check Quad Collisions"
    bl_description = "Validate selected all-quad Mesh with the native collision engine"

    def execute(self, context):
        return _execute_action(self, context, "check_quads", _validate_quad_mesh)


class GQ_OT_faceset_guides(Operator):
    bl_idname = "guidedquad_trial.faceset_guides"
    bl_label = "Extract Face Set Guides"
    bl_description = "Extract sculpt Face Set boundaries into a visible Curve object"

    def execute(self, context):
        return _execute_action(self, context, "extract_faceset_guides", _extract_faceset_guides)


class GQ_OT_patch_preview(Operator):
    bl_idname = "guidedquad_trial.patch_preview"
    bl_label = "Create Four-Corner Test Patch"
    bl_description = "Create a quad grid from one selected quad face; optional source projection"

    def execute(self, context):
        return _execute_action(self, context, "patch_preview", _build_trial_patch)


class GQ_OT_export_tracker(Operator):
    bl_idname = "guidedquad_trial.export_tracker"
    bl_label = "Export Feedback ZIP"
    bl_description = "Bundle JSON metrics only, never scenes/meshes; no automatic upload"

    def execute(self, context):
        settings = context.scene.guidedquad_trial
        try:
            path = export_bundle(_report_directory(settings))
            settings.last_status = "Feedback ZIP created"
            settings.last_report = path.name
            self.report({"INFO"}, "Feedback ZIP created: " + path.name)
            return {"FINISHED"}
        except Exception as exc:
            self.report({"ERROR"}, str(exc)[:220])
            return {"CANCELLED"}


class GQ_PT_trial(Panel):
    bl_label = "GuidedQuad — Trial"
    bl_idname = "GQ_PT_trial"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "GuidedQuad"

    def draw(self, context):
        layout = self.layout
        props = context.scene.guidedquad_trial
        layout.label(text="Experimental core, not global retopology", icon="INFO")
        layout.prop(props, "cli_path")
        col = layout.column(align=True)
        col.label(text="Diagnostics")
        col.operator("guidedquad_trial.validate_quads", icon="MOD_BOOLEAN")
        col.operator("guidedquad_trial.faceset_guides", icon="OUTLINER_OB_CURVE")
        box = layout.box()
        box.label(text="Four-corner patch preview")
        row = box.row(align=True)
        row.prop(props, "u_segments")
        row.prop(props, "v_segments")
        box.prop(props, "project_to_reference")
        if props.project_to_reference:
            box.prop(props, "reference_mesh")
            box.prop(props, "iterations")
            box.prop(props, "strength")
        box.operator("guidedquad_trial.patch_preview", icon="MESH_GRID")
        layout.separator()
        layout.label(text="Local feedback tracker")
        layout.prop(props, "report_dir")
        layout.prop(props, "include_object_names")
        layout.operator("guidedquad_trial.export_tracker", icon="FILE_ARCHIVE")
        layout.label(text=props.last_status[:72])
        if props.last_report:
            layout.label(text=props.last_report[:72])


CLASSES = (GQTrialSettings, GQ_OT_validate_quads, GQ_OT_faceset_guides,
           GQ_OT_patch_preview, GQ_OT_export_tracker, GQ_PT_trial)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.guidedquad_trial = PointerProperty(type=GQTrialSettings)


def unregister():
    if hasattr(bpy.types.Scene, "guidedquad_trial"):
        del bpy.types.Scene.guidedquad_trial
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
