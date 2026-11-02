from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
from fps_pose_intervention import replace


class PoseIntervention(unittest.TestCase):
    def test_only_selected_window_and_candidate_change(self):
        header = ['5 "profile" 1 0','75 1 0','0 0 0','0 0 0 1','0 0 0','0 0 0',
                  '1','"body" -1 0 0 0 0 0 0 1','1','ground','2']
        frames = ['0 command','root','loc0','anim0','1 command','root','loc1','anim1']
        text = '\n'.join(header + frames) + '\n'
        with tempfile.TemporaryDirectory() as directory:
            a,b,out = [Path(directory)/name for name in ('a','b','out')]
            a.write_text(text); b.write_text(text.replace('loc','donor-loc').replace('anim','donor-anim'))
            replace(a,b,out,'locomotion',1,2)
            self.assertEqual(out.read_text(),text.replace('loc1','donor-loc1'))
            replace(a,b,out,'animation',0,1)
            self.assertEqual(out.read_text(),text.replace('anim0','donor-anim0'))
            self.assertEqual(a.read_text(),text)
            with self.assertRaises(ValueError): replace(a,b,a)
