# SPDX-License-Identifier: MIT
"""Pure-Python interchange, validation and diagnostics for GuidedQuad trial.

No bpy imports: these helpers are unit-tested on Linux, Windows and macOS.
"""
import itertools
import json
import math
from pathlib import Path
import re

SCHEMA_VERSION = "guidedquad-trial-v1"
SIDE_NAMES = ("bottom", "right", "top", "left")


class TrialError(ValueError):
    pass


def _point(point):
    if len(point) != 3:
        raise TrialError("Guide point must have three coordinates")
    coords = tuple(float(value) for value in point)
    if not all(math.isfinite(x) for x in coords):
        raise TrialError("Guide coordinates must be finite")
    return coords


def _distance(a, b):
    return math.dist(a, b)


def orient_boundaries(curves, tolerance_fraction=0.01):
    """Resolve four independently directed open curves into the native chart.

    bottom: left->right, right: bottom->top, top: left->right,
    left: bottom->top.  Finds the lowest-error combination of reversals,
    then validates all four corners relative to the overall guide diagonal.
    """
    if len(curves) != 4:
        raise TrialError("Exactly four sides are required")
    if not (math.isfinite(tolerance_fraction) and 0 < tolerance_fraction <= 0.2):
        raise TrialError("Corner tolerance must be between 0 and 0.2")
    sides = []
    for curve in curves:
        points = [_point(point) for point in curve]
        if len(points) < 2 or len(points) > 100000:
            raise TrialError("Every side needs between 2 and 100000 points")
        if any(_distance(a, b) <= 1e-15 for a, b in zip(points, points[1:])):
            raise TrialError("Guide curve contains repeated consecutive points")
        sides.append(points)
    coords = [p for side in sides for p in side]
    extents = [max(p[i] for p in coords) - min(p[i] for p in coords)
               for i in range(3)]
    diagonal = math.sqrt(sum(value * value for value in extents))
    if not (diagonal > 0 and math.isfinite(diagonal)):
        raise TrialError("Guide bounds must be nondegenerate")

    best = None
    # Each input curve may be forward or backward, with no dependency on its
    # object direction. The solver expects four shared chart corners.
    for flips in itertools.product((False, True), repeat=4):
        b, r, t, l = [list(reversed(sides[i])) if flips[i] else sides[i]
                      for i in range(4)]
        errors = (_distance(b[-1], r[0]), _distance(r[-1], t[-1]),
                  _distance(t[0], l[-1]), _distance(l[0], b[0]))
        candidate = (max(errors), sum(errors), flips)
        if best is None or candidate < best[0]:
            best = (candidate, [b, r, t, l], errors)
    _, oriented, errors = best
    if max(errors) > max(1e-8, tolerance_fraction * diagonal):
        raise TrialError(
            "Guide endpoints do not meet in four corners "
            f"(maximum gap {max(errors):.6g}; allowed {tolerance_fraction * diagonal:.6g})"
        )
    # Snap matched corner endpoints to their midpoint to avoid a nonzero
    # corner gap reaching the C++ solver, while recording the gap separately.
    b, r, t, l = oriented
    for (left, i), (right, j) in (
        ((b, -1), (r, 0)), ((r, -1), (t, -1)),
        ((t, 0), (l, -1)), ((l, 0), (b, 0))
    ):
        mid = tuple((left[i][axis] + right[j][axis]) / 2.0 for axis in range(3))
        left[i] = mid
        right[j] = mid
    return dict(zip(SIDE_NAMES, oriented)), max(errors), diagonal


def write_guide(path, sides):
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        for name in SIDE_NAMES:
            points = sides[name]
            handle.write(f"{name} {len(points)}\n")
            for point in points:
                handle.write(" ".join(format(float(x), ".17g") for x in point) + "\n")


def parse_quad_obj(path, max_vertices=10_000_000, max_faces=10_000_000):
    vertices, quads = [], []
    with open(path, encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            tokens = line.strip().split()
            if not tokens or tokens[0].startswith("#"):
                continue
            if tokens[0] == "v":
                if len(tokens) < 4 or len(vertices) >= max_vertices:
                    raise TrialError(f"Invalid OBJ vertex on line {line_number}")
                vertices.append(_point(tokens[1:4]))
            elif tokens[0] == "f":
                if len(tokens) != 5 or len(quads) >= max_faces:
                    raise TrialError(f"Only quad faces accepted (line {line_number})")
                indices = []
                for raw in tokens[1:]:
                    stem = raw.split("/")[0]
                    if not stem.isdecimal() or int(stem) == 0:
                        raise TrialError(f"Invalid positive OBJ index (line {line_number})")
                    index = int(stem) - 1
                    if index >= len(vertices):
                        raise TrialError(f"OBJ index outside vertex range (line {line_number})")
                    indices.append(index)
                if len(set(indices)) != 4:
                    raise TrialError(f"Repeated quad vertex (line {line_number})")
                quads.append(tuple(indices))
    if not vertices or not quads:
        raise TrialError("Generated OBJ contains no quad mesh")
    return vertices, quads



def quad_mesh_quality(vertices, quads):
    """Cheap post-projection shape diagnostics, without claiming UV quality.

    Face area is computed from the existing (0, 2) quad diagonal. Warpage
    compares the two triangular halves. The percentiles refer to face-wise
    longest/shortest boundary edge length, not subdivision density.
    """
    def sub(a, b):
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0])

    aspect_ratios = []
    min_area = float("inf")
    max_warpage = 0.0
    degenerate = 0
    for face in quads:
        pts = [vertices[index] for index in face]
        edges = [math.dist(pts[i], pts[(i + 1) % 4]) for i in range(4)]
        shortest = min(edges)
        if shortest <= 0:
            degenerate += 1
            continue
        aspect_ratios.append(max(edges) / shortest)
        a = cross(sub(pts[1], pts[0]), sub(pts[2], pts[0]))
        b = cross(sub(pts[2], pts[0]), sub(pts[3], pts[0]))
        area_a, area_b = math.sqrt(sum(x * x for x in a)), math.sqrt(sum(x * x for x in b))
        area = 0.5 * (area_a + area_b)
        min_area = min(min_area, area)
        if area_a <= 0 or area_b <= 0:
            degenerate += 1
            continue
        cosine = max(-1.0, min(1.0, sum(a[i] * b[i] for i in range(3)) / (area_a * area_b)))
        max_warpage = max(max_warpage, math.degrees(math.acos(cosine)))
    aspect_ratios.sort()
    count = len(aspect_ratios)
    return {
        "quads_measured": len(quads),
        "degenerate_quads": degenerate,
        "worst_edge_aspect_ratio": max(aspect_ratios) if count else None,
        "p95_edge_aspect_ratio": aspect_ratios[min(count - 1, int(0.95 * count))]
                                 if count else None,
        "minimum_face_area": min_area if math.isfinite(min_area) else None,
        "maximum_diagonal_warpage_degrees": max_warpage,
    }

def parse_metrics(output):
    for line in output.splitlines():
        if line.startswith("GQ_METRICS "):
            metrics = {}
            for token in line.split()[1:]:
                name, _, raw = token.partition("=")
                if not name or not raw:
                    raise TrialError("Invalid CLI metric token")
                try:
                    number = float(raw)
                except ValueError as exc:
                    raise TrialError(f"Invalid CLI metric {name}") from exc
                if not math.isfinite(number):
                    raise TrialError(f"Non-finite CLI metric {name}")
                metrics[name] = number
            for required in ("quads", "vertices", "guide_error"):
                if required not in metrics:
                    raise TrialError(f"CLI metrics missing {required}")
            return metrics
    raise TrialError("Native engine did not output GQ_METRICS")


def save_report(directory, report, file_stem):
    root = Path(directory).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    stem = re.sub(r"[^A-Za-z0-9_-]", "_", file_stem)[:64] or "trial"
    path = root / (stem + ".json")
    # Explicitly allow one report per operation, with a caller-supplied unique
    # UTC stamp; no upload or network transfer occurs.
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
    return str(path)
