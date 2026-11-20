# SPDX-License-Identifier: MIT
"""Local-only GuidedQuad trial diagnostic reports. No Blender dependency."""
from __future__ import annotations

import datetime as _dt
import json
import os
from pathlib import Path
import uuid
import zipfile

SCHEMA = "guidedquad.trial.report.v1"
MAX_REPORT_BYTES = 256_000
MAX_REPORTS_IN_ARCHIVE = 200
README = """GuidedQuad trial diagnostic bundle
=================================
Contains only locally generated JSON reports. It does NOT include .blend files,
mesh vertices, scene screenshots, video, private file paths or source OBJ.
Reports are NEVER uploaded automatically. You may inspect the JSON files before
sharing the ZIP with the GuidedQuad developer.

Suggested feedback: exact steps, what you expected vs what happened, a
screenshot separately if you choose, and the report ID from the UI.
"""


def save_report(directory, data):
    """Write one unique, atomically-replaced report. Return its Path."""
    folder = Path(directory).expanduser()
    folder.mkdir(parents=True, exist_ok=True)
    now = _dt.datetime.now(_dt.timezone.utc)
    data = dict(data)
    data.setdefault("schema", SCHEMA)
    data.setdefault("timestamp_utc", now.isoformat(timespec="seconds"))
    data.setdefault("report_id", uuid.uuid4().hex[:12])
    filename = "gq_" + now.strftime("%Y%m%dT%H%M%S") + "_" + data["report_id"] + ".json"
    output = folder / filename
    raw = json.dumps(data, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    if len(raw.encode("utf-8")) > MAX_REPORT_BYTES:
        raise ValueError("diagnostic report exceeds size limit")
    temp = output.with_suffix(".json.tmp")
    try:
        with open(temp, "x", encoding="utf-8") as f:
            f.write(raw)
        os.replace(temp, output)
    finally:
        if temp.exists():
            temp.unlink()
    return output


def export_bundle(directory, max_reports=MAX_REPORTS_IN_ARCHIVE):
    """Bundle only report JSON files and a privacy notice, never geometry."""
    folder = Path(directory).expanduser()
    if not (1 <= max_reports <= MAX_REPORTS_IN_ARCHIVE):
        raise ValueError("invalid diagnostic bundle report limit")
    candidates = sorted(
        (p for p in folder.glob("gq_*.json") if p.is_file()),
        key=lambda p: p.name,
        reverse=True,
    )[:max_reports]
    if not candidates:
        raise ValueError("no GuidedQuad reports found")
    archive = folder / ("guidedquad_feedback_" +
                        _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%S") +
                        "_" + uuid.uuid4().hex[:6] + ".zip")
    included = 0
    with zipfile.ZipFile(archive, mode="x", compression=zipfile.ZIP_DEFLATED) as z:
        z.writestr("README.txt", README)
        for p in reversed(candidates):
            try:
                if p.stat().st_size > MAX_REPORT_BYTES:
                    continue
                # A partial/corrupt local report must never block export.
                with p.open("r", encoding="utf-8") as f:
                    report = json.load(f)
            except (OSError, UnicodeError, ValueError):
                continue
            if not isinstance(report, dict) or report.get("schema") != SCHEMA:
                continue
            z.write(p, arcname=p.name)
            included += 1
    if not included:
        archive.unlink(missing_ok=True)
        raise ValueError("no valid GuidedQuad JSON reports found")
    return archive
