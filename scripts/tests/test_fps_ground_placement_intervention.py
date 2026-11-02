from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).parents[1]))
from fps_ground_placement_intervention import translate


class GroundPlacement(unittest.TestCase):
    def test_translation_preserves_ground_and_local_pose(self):
        lines=['5 "profile" 1 0','75 1 0','0 2 3','0 0 0 1','0 0 0','0 0 0',
               '1','"body" -1 0 0 0 0 0 0 1','1','2 0 0 0 0 0 0 1 0.8 0 "shape with spaces.bin"','1',
               '0 4 5 6 0 0 0 1 0 60 0 0 1 0','7 8 9 0 0 0 1','local pose','animation pose']
        with tempfile.TemporaryDirectory() as directory:
            source,output=[Path(directory)/x for x in ('source','output')]
            source.write_text('\n'.join(lines)+'\n')
            translate(source,output,0)
            self.assertEqual(source.read_bytes(),output.read_bytes())
            translate(source,output,1000)
            expected=lines.copy()
            expected[2]='1000 2 3';expected[11]=lines[11].replace('0 4 5','0 1004 5')
            expected[12]='1007 8 9 0 0 0 1'
            self.assertEqual(output.read_text().splitlines(),expected)
            translate(source,output,0,target='ground')
            self.assertEqual(source.read_bytes(),output.read_bytes())
            report=translate(source,output,-1000,target='ground')
            expected=lines.copy()
            expected[9]=lines[9].replace('2 0 0 0','2 -1000 0 0',1)
            self.assertEqual(output.read_text().splitlines(),expected)
            self.assertEqual(report['translation_target'],'ground')
            self.assertEqual(source.read_text().splitlines(),lines)
            with self.assertRaises(ValueError):translate(source,source,1)
