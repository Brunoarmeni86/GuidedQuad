# SPDX-License-Identifier: MIT
"""GuidedQuad trial adapter: four guided curves -> native quad patch + tracker.

This intentionally does NOT claim automatic quadrangulation of arbitrary
closed meshes. Native guidedquad_cli must be built separately.
"""
import datetime
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import tempfile
import time

import bpy
from bpy.props import BoolProperty, FloatProperty, IntProperty, PointerProperty, StringProperty
from mathutils.geometry import interpolate_bezier

from .trial_core import (
    TrialError, SCHEMA_VERSION, orient_boundaries, parse_metrics,
    parse_quad_obj, save_report, write_guide,
)


def _is_type(object_type):
    def poll(_self, obj):
        return obj is not None and obj.type == object_type
    return poll


class GQ_TrialSettings(bpy.types.PropertyGroup):
    source: PointerProperty(name="Reference Mesh", type=bpy.types.Object, poll=_is_type("MESH"))
    bottom: PointerProperty(name="Bottom Guide", type=bpy.types.Object, poll=_is_type("CURVE"))
    right: PointerProperty(name="Right Guide", type=bpy.types.Object, poll=_is_type("CURVE"))
    top: PointerProperty(name="Top Guide", type=bpy.types.Object, poll=_is_type("CURVE"))
    left: PointerProperty(name="Left Guide", type=bpy.types.Object, poll=_is_type("CURVE"))
    executable: StringProperty(
        name="Native Engine", description="Path to guidedquad_cli or guidedquad_cli.exe",
        subtype="FILE_PATH",
    )
    report_folder: StringProperty(
        name="Report Folder", subtype="DIR_PATH", default="//GuidedQuadReports",
    )
    u_segments: IntProperty(name="U Quads", default=24, min=2, max=512)
    v_segments: IntProperty(name="V Quads", default=24, min=2, max=512)
    projection_iterations: IntProperty(name="Projection Steps", default=2, min=0, max=20)
    projection_strength: FloatProperty(name="Projection Strength", default=0.9,
                                       min=0.01, max=1.0)
    corner_tolerance: FloatProperty(
        name="Corner Gap (relative)", description="Maximum endpoint mismatch as a fraction of guide bounds",
        default=0.01, min=0.00001, max=0.2, precision=4,
    )
    bezier_resolution: IntProperty(
        name="Samples per Bezier Segment", default=12, min=2, max=64,
    )
    verify_collision: BoolProperty(name="Check Output Collisions", default=True)
    timeout_seconds: IntProperty(name="Native Timeout (s)", default=180, min=5, max=1800)
    last_report: StringProperty(options={'HIDDEN'})
    last_report_path: StringProperty(name="Last JSON", subtype="FILE_PATH", options={'HIDDEN'})


def _curve_samples(obj, bezier_resolution):
    if obj is None or obj.type != "CURVE" or len(obj.data.splines) != 1:
        raise TrialError("Each guide must be a curve object containing exactly one open spline")
    spline = obj.data.splines[0]
    if spline.use_cyclic_u:
        raise TrialError("Guide splines must be open, not cyclic")
    if spline.type == "POLY":
        points = [point.co.xyz.copy() for point in spline.points]
    elif spline.type == "BEZIER":
        bezier = spline.bezier_points
        if len(bezier) < 2:
            raise TrialError("Bezier guide requires at least two control points")
        points = []
        for i in range(len(bezier) - 1):
            a, b = bezier[i], bezier[i + 1]
            segment = interpolate_bezier(
                a.co, a.handle_right, b.handle_left, b.co, bezier_resolution + 1
            )
            if i:
                segment = segment[1:]
            points.extend(segment)
    else:
        raise TrialError("Only POLY and BEZIER guides are supported in this trial; convert NURBS")
    if len(points) < 2 or len(points) > 100000:
        raise TrialError("Guide has too few or too many evaluated points")
    result = []
    for point in points:
        world = obj.matrix_world @ point
        triple = (float(world.x), float(world.y), float(world.z))
        if not all(math.isfinite(x) for x in triple):
            raise TrialError("Nonfinite guide coordinates")
        result.append(triple)
    return result


def _write_source_obj(obj, depsgraph, path):
    if obj is None or obj.type != "MESH":
        raise TrialError("Choose a mesh object as reference")
    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh(preserve_all_data_layers=False, depsgraph=depsgraph)
    if mesh is None:
        raise TrialError("Blender could not evaluate the reference mesh")
    try:
        mesh.calc_loop_triangles()
        count_v, count_t = len(mesh.vertices), len(mesh.loop_triangles)
        if count_v < 3 or count_t < 1:
            raise TrialError("Reference mesh has no usable triangles")
        if count_v > 5_000_000 or count_t > 10_000_000:
            raise TrialError("Trial limit: 5M evaluated vertices / 10M triangles")
        with open(path, "w", encoding="utf-8", newline="\n") as handle:
            for vertex in mesh.vertices:
                world = obj.matrix_world @ vertex.co
                values = (world.x, world.y, world.z)
                if not all(math.isfinite(value) for value in values):
                    raise TrialError("Reference mesh contains nonfinite coordinates")
                handle.write("v " + " ".join(format(value, ".17g") for value in values) + "\n")
            for triangle in mesh.loop_triangles:
                handle.write("f " + " ".join(str(index + 1) for index in triangle.vertices) + "\n")
        return count_v, count_t
    finally:
        evaluated.to_mesh_clear()


def _create_quad_object(context, vertices, quads):
    mesh = bpy.data.meshes.new("GuidedQuad_Trial_Quads")
    try:
        mesh.from_pydata(vertices, [], quads)
        mesh.update()
        obj = bpy.data.objects.new("GuidedQuad_Trial_Result", mesh)
        context.scene.collection.objects.link(obj)
    except Exception:
        bpy.data.meshes.remove(mesh)
        raise
    # OBJ coordinates are world-space; output object remains at identity.
    for selected in tuple(context.selected_objects):
        selected.select_set(False)
    obj.select_set(True)
    context.view_layer.objects.active = obj
    return obj


def _report_folder(settings):
    folder = settings.report_folder or "//GuidedQuadReports"
    if folder.startswith("//") and not bpy.data.filepath:
        # Avoid writing into the unpredictable current process directory for
        # an unsaved .blend file.
        return bpy.utils.user_resource("CONFIG", path="guidedquad_trial_reports", create=True)
    return bpy.path.abspath(folder)


def _report_stamp():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")


class GQ_OT_run_trial(bpy.types.Operator):
    bl_idname = "guidedquad.run_trial"
    bl_label = "Generate GuidedQuad Trial"
    bl_description = "Run native four-curve patch generation and write a local diagnostics report"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        return context.mode == "OBJECT" and context.scene is not None

    def execute(self, context):
        settings = context.scene.guidedquad_trial
        report = {
            "schema": SCHEMA_VERSION,
            "time_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "status": "failed",
            "addon_version": "0.3.9-trial",
            "blender_version": bpy.app.version_string,
            "platform": platform.system(),
            "parameters": {
                "u_segments": settings.u_segments,
                "v_segments": settings.v_segments,
                "projection_iterations": settings.projection_iterations,
                "projection_strength": settings.projection_strength,
                "bezier_resolution": settings.bezier_resolution,
                "corner_tolerance": settings.corner_tolerance,
                "verify_collision": settings.verify_collision,
            },
            "metric_units": {"duration": "ms", "geometry": "world Blender units"},
        }
        t0 = time.perf_counter()
        fail_message = None
        try:
            if not settings.executable:
                raise TrialError("Select the compiled guidedquad_cli executable")
            executable = Path(bpy.path.abspath(settings.executable)).expanduser()
            if not executable.is_file():
                raise TrialError("Native guidedquad_cli file not found")
            if not settings.source or settings.source.type != "MESH":
                raise TrialError("Select a reference mesh object")
            guides = (settings.bottom, settings.right, settings.top, settings.left)
            if any(obj is None for obj in guides) or len(set(obj.as_pointer() for obj in guides)) != 4:
                raise TrialError("Assign four DIFFERENT open CURVE objects (Bottom / Right / Top / Left)")
            boundary, gap, diagonal = orient_boundaries(
                [_curve_samples(curve, settings.bezier_resolution) for curve in guides],
                settings.corner_tolerance,
            )
            report["guide"] = {
                "maximum_original_corner_gap": gap,
                "bounds_diagonal": diagonal,
                "point_counts": {name: len(points) for name, points in boundary.items()},
            }
            with tempfile.TemporaryDirectory(prefix="guidedquad_trial_") as folder:
                guide_path = os.path.join(folder, "guide.txt")
                source_path = os.path.join(folder, "source.obj")
                output_path = os.path.join(folder, "output.obj")
                write_guide(guide_path, boundary)
                v_count, t_count = _write_source_obj(
                    settings.source, context.evaluated_depsgraph_get(), source_path
                )
                report["source"] = {"vertices": v_count, "triangles": t_count}
                command = [
                    str(executable), "guided-patch", guide_path, source_path, output_path,
                    str(settings.u_segments), str(settings.v_segments),
                    str(settings.projection_iterations),
                    format(settings.projection_strength, ".9g"),
                ]
                started = time.perf_counter()
                proc = subprocess.run(
                    command, capture_output=True, text=True, encoding="utf-8",
                    errors="replace", timeout=settings.timeout_seconds, shell=False,
                )
                report["engine_duration_ms"] = round((time.perf_counter() - started) * 1000, 3)
                report["engine_return_code"] = proc.returncode
                # Temporary filenames may contain local path information; no
                # raw stdout/stderr or private mesh data is placed in the report.
                if proc.returncode:
                    raise TrialError("Native patch generation failed; check Blender console for details")
                metrics = parse_metrics(proc.stdout)
                report["engine_metrics"] = metrics
                vertices, quads = parse_quad_obj(output_path)
                if settings.verify_collision:
                    collision = subprocess.run(
                        [str(executable), "check-quads", output_path],
                        capture_output=True, text=True, encoding="utf-8",
                        errors="replace", timeout=settings.timeout_seconds, shell=False,
                    )
                    report["collision_exit_code"] = collision.returncode
                    if collision.returncode != 0:
                        raise TrialError("Generated patch failed native collision validation")
                obj = _create_quad_object(context, vertices, quads)
                report["output"] = {
                    "vertices": len(vertices),
                    "quads": len(quads),
                    "object_created": bool(obj),
                }
            report["status"] = "success"
        except (TrialError, OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
            fail_message = str(exc)
            report["error"] = fail_message
        except Exception as exc:
            # Preserve a useful report rather than silently swallowing unexpected
            # Blender API issues, but do not include an unredacted traceback.
            fail_message = f"Unexpected {type(exc).__name__}; see Blender console"
            report["error"] = fail_message
            import traceback
            traceback.print_exc()
        finally:
            report["total_duration_ms"] = round((time.perf_counter() - t0) * 1000, 3)
            settings.last_report = json.dumps(report, ensure_ascii=False, allow_nan=False)
            try:
                path = save_report(_report_folder(settings), report, "guidedquad_" + _report_stamp())
                settings.last_report_path = path
            except (OSError, ValueError) as exc:
                settings.last_report_path = ""
                self.report({"WARNING"}, f"Could not write report: {exc}")
        if fail_message:
            self.report({"ERROR"}, f"GuidedQuad: {fail_message[:190]}")
            return {"CANCELLED"}
        self.report({"INFO"}, "GuidedQuad trial generated; report saved")
        return {"FINISHED"}


class GQ_OT_export_report(bpy.types.Operator):
    bl_idname = "guidedquad.export_trial_report"
    bl_label = "Export Last Report"
    bl_description = "Write the last locally stored JSON report again"

    def execute(self, context):
        settings = context.scene.guidedquad_trial
        if not settings.last_report:
            self.report({"ERROR"}, "Run a trial first")
            return {"CANCELLED"}
        try:
            report = json.loads(settings.last_report)
            path = save_report(_report_folder(settings), report, "guidedquad_" + _report_stamp())
        except (OSError, ValueError) as exc:
            self.report({"ERROR"}, f"Cannot export report: {exc}")
            return {"CANCELLED"}
        settings.last_report_path = path
        self.report({"INFO"}, "Report saved to chosen folder")
        return {"FINISHED"}


class GQ_PT_trial_panel(bpy.types.Panel):
    bl_label = "GuidedQuad Trial"
    bl_idname = "GQ_PT_trial_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "GuidedQuad"

    def draw(self, context):
        layout = self.layout
        props = context.scene.guidedquad_trial
        col = layout.column(align=True)
        col.label(text="Native Engine")
        col.prop(props, "executable", text="")
        col.separator()
        col.label(text="Reference / Four-Sided Patch")
        col.prop(props, "source")
        for name in ("bottom", "right", "top", "left"):
            col.prop(props, name)
        col.label(text="Open POLY or BEZIER curves")
        col.separator()
        row = col.row(align=True)
        row.prop(props, "u_segments")
        row.prop(props, "v_segments")
        col.prop(props, "projection_iterations")
        col.prop(props, "projection_strength")
        config = col.box()
        config.label(text="Sampling and Validation")
        config.prop(props, "bezier_resolution")
        config.prop(props, "corner_tolerance")
        config.prop(props, "verify_collision")
        config.prop(props, "timeout_seconds")
        col.separator()
        col.prop(props, "report_folder")
        col.operator("guidedquad.run_trial", icon="MOD_REMESH")
        if props.last_report:
            row = col.row()
            row.operator("guidedquad.export_trial_report", icon="EXPORT")
            try:
                last = json.loads(props.last_report)
                status = last.get("status", "unknown")
                col.label(text="Last trial: " + status)
                col.label(text=f"Duration: {last.get('total_duration_ms', 0):.1f} ms")
            except (ValueError, TypeError):
                col.label(text="Last report unreadable")
            if props.last_report_path:
                col.label(text="Report saved in chosen folder")


CLASSES = (
    GQ_TrialSettings, GQ_OT_run_trial, GQ_OT_export_report, GQ_PT_trial_panel,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.guidedquad_trial = PointerProperty(type=GQ_TrialSettings)


def unregister():
    del bpy.types.Scene.guidedquad_trial
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
