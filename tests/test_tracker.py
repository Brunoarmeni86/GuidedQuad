# SPDX-License-Identifier: MIT
import json
import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender" / "guidedquad_trial"))
from tracker import SCHEMA, save_report, export_bundle  # noqa: E402


class TrackerTest(unittest.TestCase):
    def test_reports_and_no_geometry_in_zip(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder / "secret_source.obj").write_text("secret mesh geometry")
            (folder / "production.blend").write_bytes(b"secret blend")
            payload = {
                "action": "check_quads", "status": "success",
                "parameters": {"quads": 64, "elapsed_ms": 12.1},
            }
            a = save_report(folder, payload)
            b = save_report(folder, payload)
            self.assertNotEqual(a, b)
            self.assertEqual(json.loads(a.read_text())["schema"], SCHEMA)
            archive = export_bundle(folder)
            import zipfile
            with zipfile.ZipFile(archive) as z:
                names = z.namelist()
                self.assertEqual(len(names), 3)
                self.assertIn("README.txt", names)
                self.assertIn(a.name, names)
                self.assertIn(b.name, names)
                self.assertNotIn("secret_source.obj", names)
                self.assertNotIn("production.blend", names)

    def test_invalid_bundle_and_data(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                export_bundle(tmp)
            with self.assertRaises(ValueError):
                export_bundle(tmp, max_reports=0)
            with self.assertRaises(ValueError):
                save_report(tmp, {"blob": "x" * 300_000})
            self.assertEqual(list(Path(tmp).glob("gq_*.json")), [])
            with self.assertRaises(ValueError):
                export_bundle(tmp)
            (Path(tmp) / "gq_untrusted.json").write_text('{"schema":"wrong"}')
            save_report(tmp, {"action": "test"})
            archive = export_bundle(tmp)
            import zipfile
            with zipfile.ZipFile(archive) as z:
                self.assertEqual(len(z.namelist()), 2)


if __name__ == "__main__":
    unittest.main()
