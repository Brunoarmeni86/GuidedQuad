# SPDX-License-Identifier: MIT
import importlib.util
from pathlib import Path
import tempfile
import unittest

CORE_PATH = Path(__file__).resolve().parents[1] / "guidedquad_trial" / "trial_core.py"
spec = importlib.util.spec_from_file_location("guidedquad_trial_core", CORE_PATH)
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)


class TrialCoreTests(unittest.TestCase):
    def setUp(self):
        # Every side has one central guide knot, which must be preserved.
        self.curves = [
            [(0, 0, 0), (0.5, -0.05, 0), (1, 0, 0)],
            [(1, 0, 0), (1.02, 0.5, 0), (1, 1, 0)],
            [(0, 1, 0), (0.5, 1.04, 0), (1, 1, 0)],
            [(0, 0, 0), (-0.03, 0.5, 0), (0, 1, 0)],
        ]

    def test_all_guide_direction_combinations(self):
        for mask in range(16):
            guides = [list(reversed(points)) if (mask >> i) & 1 else points
                      for i, points in enumerate(self.curves)]
            oriented, gap, diagonal = core.orient_boundaries(guides)
            self.assertEqual(gap, 0)
            self.assertGreater(diagonal, 1)
            for name in core.SIDE_NAMES:
                self.assertEqual(len(oriented[name]), 3)

    def test_guide_endpoint_snap(self):
        guides = [list(side) for side in self.curves]
        guides[1][-1] = (1.003, 1, 0)
        oriented, gap, _ = core.orient_boundaries(guides)
        self.assertGreater(gap, 0)
        self.assertEqual(oriented["right"][-1], oriented["top"][-1])
        self.assertLess(gap, 0.01)

    def test_guide_corner_mismatch_fails(self):
        guides = [list(side) for side in self.curves]
        guides[1][0] = (10, 0, 0)
        with self.assertRaises(core.TrialError):
            core.orient_boundaries(guides, 0.001)

    def test_repeated_invalid_nonfinite_points(self):
        with self.assertRaises(core.TrialError):
            core.orient_boundaries([
                [(0, 0, 0), (0, 0, 0)], *self.curves[1:]
            ])
        with self.assertRaises(core.TrialError):
            core.orient_boundaries([
                [(0, 0, 0), (float("nan"), 1, 0)], *self.curves[1:]
            ])

    def test_write_guide_order_and_precision(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "guide.txt"
            oriented, _, _ = core.orient_boundaries(self.curves)
            core.write_guide(path, oriented)
            lines = path.read_text(encoding="utf-8").splitlines()
            self.assertEqual([line for line in lines if line.split()[0] in
                              core.SIDE_NAMES], ["bottom 3", "right 3", "top 3", "left 3"])

    def test_parse_quad_obj(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "output.obj"
            path.write_text("v 0 0 0\nv 1 0 0\nv 1 1 0\n"
                            "v 0 1 0\nf 1 2 3 4\n", encoding="utf-8")
            verts, quads = core.parse_quad_obj(path)
            self.assertEqual(len(verts), 4)
            self.assertEqual(quads, [(0, 1, 2, 3)])
            path.write_text("v 0 0 0\nf 1 1 1 1\n", encoding="utf-8")
            with self.assertRaises(core.TrialError):
                core.parse_quad_obj(path)

    def test_parse_native_metrics(self):
        line = "GQ_METRICS vertices=25 quads=16 guide_error=0 projection_final_rms=0.0015"
        values = core.parse_metrics("warning\n" + line + "\n")
        self.assertEqual(values["quads"], 16.0)
        self.assertAlmostEqual(values["projection_final_rms"], 0.0015)
        with self.assertRaises(core.TrialError):
            core.parse_metrics("No native result\n")
        with self.assertRaises(core.TrialError):
            core.parse_metrics("GQ_METRICS vertices=25 quads=inf guide_error=0")

    def test_output_quality_metrics(self):
        coords = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]
        values = core.quad_mesh_quality(coords, [(0, 1, 2, 3)])
        self.assertEqual(values["degenerate_quads"], 0)
        self.assertAlmostEqual(values["worst_edge_aspect_ratio"], 1)
        self.assertAlmostEqual(values["p95_edge_aspect_ratio"], 1)
        self.assertAlmostEqual(values["minimum_face_area"], 1)
        self.assertAlmostEqual(values["maximum_diagonal_warpage_degrees"], 0)

        warped = list(coords)
        warped[-1] = (0, 1, 0.25)
        warped_metrics = core.quad_mesh_quality(warped, [(0, 1, 2, 3)])
        self.assertGreater(warped_metrics["maximum_diagonal_warpage_degrees"], 0)

    def test_tracker_file_is_local_json(self):
        with tempfile.TemporaryDirectory() as root:
            payload = {"schema": core.SCHEMA_VERSION, "status": "success"}
            path = core.save_report(root, payload, "sample:name")
            self.assertTrue(path.endswith("sample_name.json"))
            self.assertIn('"status": "success"', Path(path).read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
