# SPDX-License-Identifier: MIT
"""Stroke extraction without bpy dependencies (testable with Blender-like fakes).

Grease Pencil v3 (Blender >=4.3): layer.frames[n].drawing.strokes.
Legacy Grease Pencil (Blender 4.2) and Annotation: frame.strokes.
Only one frame per visible layer, chosen as latest frame <= scene frame.
Source data is never mutated or converted to permanent Curve datablocks.
"""
import math


class InkGuideError(ValueError):
    pass


def _frame(layer, current_frame):
    frames = getattr(layer, "frames", ())
    eligible = [f for f in frames if f.frame_number <= current_frame]
    if eligible:
        return max(eligible, key=lambda f: f.frame_number)
    # Do NOT read strokes from a future frame: this can generate unexpected
    # guides during animation before the first keyframe.
    return None


def _coords(point):
    pos = getattr(point, "position", None)
    if pos is None:
        pos = getattr(point, "co", None)
    if pos is None:
        raise InkGuideError("Stroke point has no position/co attribute")
    try:
        xyz = tuple(float(pos[i]) for i in range(3))
    except (TypeError, ValueError, IndexError) as exc:
        raise InkGuideError("Invalid Grease Pencil point coordinate") from exc
    if not all(math.isfinite(x) for x in xyz):
        raise InkGuideError("Nonfinite Grease Pencil point")
    return xyz


def extract_ink_strokes(data, current_frame, is_annotation=False):
    """Return local-space stroke polyline tuples and minimal provenance.

    The Annotation data coordinates are already global in the 3D view;
    object Grease Pencil coordinates require caller's matrix_world transform.
    """
    if data is None:
        raise InkGuideError("No Grease Pencil / Annotation data available")
    strokes = []
    for layer in data.layers:
        if getattr(layer, "hide", False) or getattr(layer, "annotation_hide", False):
            continue
        frame = _frame(layer, current_frame)
        if frame is None:
            continue
        drawing = frame if is_annotation else getattr(frame, "drawing", frame)
        if drawing is None:
            continue
        for index, stroke in enumerate(getattr(drawing, "strokes", ())):
            if getattr(stroke, "is_cyclic", False) or getattr(stroke, "use_cyclic", False):
                continue
            points = [_coords(p) for p in stroke.points]
            if len(points) < 2:
                continue
            # Ignore invisible/repeated click strokes, retaining diagnostics.
            if all(p == points[0] for p in points[1:]):
                continue
            strokes.append({"points": points, "layer": layer.info if is_annotation else layer.name,
                            "frame": frame.frame_number, "stroke_index": index})
            if len(strokes) > 10000:
                raise InkGuideError("Too many Grease Pencil strokes (limit 10,000)")
    return strokes


def select_ink_sides(strokes, indices):
    """Select four distinct strokes in bottom/right/top/left slot order."""
    if len(indices) != 4 or any(not isinstance(i, int) for i in indices):
        raise InkGuideError("Four integer stroke indices required")
    if len(set(indices)) != 4:
        raise InkGuideError("Bottom / Right / Top / Left must use distinct strokes")
    if min(indices) < 0 or max(indices) >= len(strokes):
        raise InkGuideError(
            f"Stroke index out of range: found {len(strokes)} usable open strokes")
    return [strokes[i]["points"] for i in indices]
