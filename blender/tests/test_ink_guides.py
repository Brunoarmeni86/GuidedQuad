# SPDX-License-Identifier: MIT
"""API-shape tests without bpy; actual Blender integration remains a user trial."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as Obj
import unittest

MOD = Path(__file__).resolve().parents[1] / "guidedquad_trial" / "ink_guides.py"
spec = importlib.util.spec_from_file_location("guidedquad_trial_ink_guides", MOD)
ink = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ink)


def stroke(points, v3=True, closed=False):
    values = [Obj(position=p) if v3 else Obj(co=p) for p in points]
    return Obj(points=values, is_cyclic=closed)


def layer(frames, name="guide", hidden=False, annotation=False):
    return Obj(frames=frames, name=name, info=name, hide=hidden)


class InkGuideTests(unittest.TestCase):
    def setUp(self):
        self.sides = [
            [(0,0,0),(.5,0,0),(1,0,0)],
            [(1,0,0),(1,1,0)],
            [(0,1,0),(1,1,0)],
            [(0,0,0),(0,1,0)],
        ]

    def test_gp_v3_at_current_frame(self):
        one = Obj(frame_number=1, drawing=Obj(strokes=[stroke(p) for p in self.sides]))
        future = Obj(frame_number=10, drawing=Obj(strokes=[stroke([(10,0,0),(20,0,0)])]))
        data = Obj(layers=[layer([one,future])])
        selected = ink.extract_ink_strokes(data, 5)
        self.assertEqual(len(selected), 4)
        self.assertTrue(all(s["frame"]==1 for s in selected))
        self.assertEqual(ink.select_ink_sides(selected, (0,1,2,3)),self.sides)
        self.assertEqual(ink.extract_ink_strokes(data, 0),[])

    def test_annotations_legacy(self):
        annotation_frame = Obj(
            frame_number=1,
            strokes=[stroke(p, v3=False) for p in self.sides],
        )
        data = Obj(layers=[layer([annotation_frame], annotation=True)])
        got=ink.extract_ink_strokes(data, 1, is_annotation=True)
        self.assertEqual(ink.select_ink_sides(got, (0,1,2,3)), self.sides)
        self.assertEqual(got[0]["layer"],"guide")

    def test_hidden_cyclic_invalid_strokes(self):
        frame=Obj(frame_number=1, drawing=Obj(strokes=[
            stroke(self.sides[0], closed=True),
            stroke([(2,2,0),(2,2,0)]),
            stroke(self.sides[1]),
        ]))
        hidden=layer([frame], hidden=True)
        shown=layer([frame])
        data=Obj(layers=[hidden,shown])
        found=ink.extract_ink_strokes(data,1)
        self.assertEqual(len(found),1)
        self.assertEqual(found[0]["points"],self.sides[1])

    def test_invalid_selection_and_coordinates(self):
        selected=[{"points":p} for p in self.sides]
        for ids in ((0,0,1,2),(0,1,2,10),(-1,1,2,3)):
            with self.assertRaises(ink.InkGuideError):
                ink.select_ink_sides(selected,ids)
        frame=Obj(frame_number=1,drawing=Obj(strokes=[
            stroke([(0,0,0),(float('nan'),0,0)]),
        ]))
        with self.assertRaises(ink.InkGuideError):
            ink.extract_ink_strokes(Obj(layers=[layer([frame])]),1)


if __name__ == "__main__":
    unittest.main()
