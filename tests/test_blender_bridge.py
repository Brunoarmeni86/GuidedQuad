# SPDX-License-Identifier: MIT
"""Blender-free adapter smoke tests. Not a replacement for Blender runtime QA."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest


def load_addon_with_fake_bpy():
    fake = types.ModuleType("bpy")
    fake_types = types.ModuleType("bpy.types")
    for name in ("Operator", "Panel", "PropertyGroup", "Object"):
        setattr(fake_types, name, type(name, (), {}))
    fake_props = types.ModuleType("bpy.props")
    for prop in ("BoolProperty", "IntProperty", "FloatProperty",
                 "PointerProperty", "StringProperty"):
        setattr(fake_props, prop, lambda **kwargs: None)
    fake.types = fake_types
    fake.props = fake_props
    fake.path = types.SimpleNamespace(abspath=lambda p: str(p))
    fake.app = types.SimpleNamespace(version_string="Mock Blender 5.x")
    fake.utils = types.SimpleNamespace(register_class=lambda cls: None,
                                       unregister_class=lambda cls: None)
    sys.modules["bpy"] = fake
    sys.modules["bpy.props"] = fake_props
    sys.modules["bpy.types"] = fake_types
    package = Path(__file__).resolve().parents[1] / "blender" / "guidedquad_trial"
    spec = importlib.util.spec_from_file_location(
        "guidedquad_trial", package / "__init__.py",
        submodule_search_locations=[str(package)])
    module = importlib.util.module_from_spec(spec)
    sys.modules["guidedquad_trial"] = module
    spec.loader.exec_module(module)
    return module


class BlenderAdapterSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.addon = load_addon_with_fake_bpy()

    def test_obj_roundtrip_and_invalid_quad(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "patch.obj"
            Vector = lambda x, y, z: types.SimpleNamespace(x=x, y=y, z=z)
            self.addon._write_obj(
                path, [Vector(0, 0, 0), Vector(1, 0, 0),
                       Vector(1, 1, 0), Vector(0, 1, 0)],
                [(0, 1, 2, 3)])
            points, quads = self.addon._read_quad_obj(path)
            self.assertEqual(len(points), 4)
            self.assertEqual(quads, [(0, 1, 2, 3)])
            path.write_text("v 0 0 0\nf 1 2 3\n")
            with self.assertRaises(ValueError):
                self.addon._read_quad_obj(path)

    def test_curve_parser_and_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "guides.obj"
            path.write_text("v 0 0 0\nv 1 0 0\nv 1 1 0\nl 1 2 3 1\n")
            points, paths = self.addon._read_guide_obj(path)
            self.assertEqual(len(points), 3)
            self.assertEqual(paths, [[0, 1, 2, 0]])
        process = self.addon._run_cli(
            Path(sys.executable), ["-c", "print('guidedquad-cli-smoke')"])
        self.assertEqual(process["exit_code"], 0)
        self.assertIn("guidedquad-cli-smoke", process["stdout"])
        self.assertGreaterEqual(process["elapsed_ms"], 0)

    def test_console_redacts_local_paths(self):
        raw = "Could not open /tmp/private_dataset/subject.obj at C:\\\\Users\\\\Private\\\\asset.obj"
        sanitized = self.addon._safe_console_text(raw)
        self.assertNotIn("/tmp/private_dataset", sanitized)
        self.assertNotIn("subject.obj", sanitized)
        self.assertIn("<local-path>", sanitized)

    def test_report_defaults_do_not_include_object_name(self):
        with tempfile.TemporaryDirectory() as tmp:
            poly = types.SimpleNamespace(vertices=(0, 1, 2, 3))
            obj = types.SimpleNamespace(
                type="MESH", name="Private Client Mesh",
                data=types.SimpleNamespace(vertices=[1, 2, 3, 4],
                                            polygons=[poly]))
            settings = types.SimpleNamespace(
                report_dir=tmp, include_object_names=False,
                last_report="", last_status="")
            path = self.addon._record(
                settings, "test", obj, "success", 0.0,
                {"test_case": "GQ-01"}, cli={"exit_code": 0})
            data = json.loads(path.read_text())
            self.assertEqual(data["object"]["vertices"], 4)
            self.assertNotIn("object_name", data["object"])
            self.assertNotIn("Private Client Mesh", path.read_text())
            self.assertTrue(settings.last_report.startswith("gq_"))


if __name__ == "__main__":
    unittest.main()
