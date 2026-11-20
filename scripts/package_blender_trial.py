#!/usr/bin/env python3
"""Build the ready-to-install Blender trial extension zip.

Usage: python scripts/package_blender_trial.py
The extension does NOT embed a platform executable: set guidedquad_cli path
in the panel or download the matching CI native executable artifact.
"""
import ast
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "blender" / "guidedquad_trial"
DIST = ROOT / "dist"
NAME = "guidedquad_trial_v0.3.9.zip"
FILES = ("__init__.py", "trial_core.py", "blender_manifest.toml")


def build():
    for file_name in FILES:
        path = SOURCE / file_name
        if not path.is_file():
            raise RuntimeError(f"Missing extension file: {path}")
        if path.suffix == ".py":
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    manifest = (SOURCE / "blender_manifest.toml").read_text(encoding="utf-8")
    if 'id = "guidedquad_trial"' not in manifest:
        raise RuntimeError("Unexpected Blender extension ID")
    DIST.mkdir(parents=True, exist_ok=True)
    archive = DIST / NAME
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as out:
        for name in FILES:
            out.write(SOURCE / name, arcname=name)
    with zipfile.ZipFile(archive) as check:
        if set(check.namelist()) != set(FILES) or check.testzip():
            raise RuntimeError("Extension archive verification failed")
    print("Packaged extension:", archive)
    print("Native guidedquad_cli must be provided separately.")
    return archive


if __name__ == "__main__":
    try:
        build()
    except Exception as exc:
        print("Packaging failed:", exc, file=sys.stderr)
        raise SystemExit(1)
