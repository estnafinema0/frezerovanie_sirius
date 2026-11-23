"""Optional offline inspector checks; use the same isolated OCP environment."""
import tempfile
import unittest
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

from analyze_step import inspect


class StepInspection(unittest.TestCase):
    def test_trimmed_solid(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'Контроль с пробелом.STEP'
            box = BRepPrimAPI_MakeBox(20, 20, 5).Shape()
            axis = gp_Ax2(gp_Pnt(10, 10, 0), gp_Dir(0, 0, 1))
            hole = BRepPrimAPI_MakeCylinder(axis, 2, 5).Shape()
            shape = BRepAlgoAPI_Cut(box, hole).Shape()
            writer = STEPControl_Writer()
            self.assertEqual(writer.Transfer(shape, STEPControl_AsIs), IFSelect_RetDone)
            self.assertEqual(writer.Write(str(path)), IFSelect_RetDone)
            report = inspect(path)
            self.assertTrue(report['valid_brep'])
            self.assertEqual(report['solids'], 1)
            for actual, expected in zip(report['size_mm'], [20, 20, 5]):
                self.assertAlmostEqual(actual, expected, places=6)
            self.assertEqual(report['source_length_units'], ['millimetre'])
            self.assertEqual(sum('inner' in f['wire_roles'] for f in report['faces']), 2)
            self.assertFalse(report['cam_eligible'])

    def test_rejects_broken_and_oversized_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'broken.STEP'
            path.write_text('invalid STEP')
            with self.assertRaises(ValueError):
                inspect(path)
            with path.open('wb') as stream:
                stream.truncate(50 * 1024 * 1024 + 1)
            with self.assertRaisesRegex(ValueError, '50 MiB'):
                inspect(path)


if __name__ == '__main__':
    unittest.main()
