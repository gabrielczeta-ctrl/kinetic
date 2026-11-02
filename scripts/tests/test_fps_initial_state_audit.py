import importlib.util
from pathlib import Path
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('fps_audit',Path(__file__).parents[1]/'fps_initial_state_audit.py')
module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


class InitialStateTests(unittest.TestCase):
    def test_matching_states_do_not_imply_fps_qualification(self):
        text=('RAGDOLL_REPLAY_RIG profile_id=x profile_hash=1 mass=75 scale=1 root=0,0,1 '
              'rotation=0,0,0,1 velocity=0,0,0 angular_velocity=0,0,0\n'
              'RAGDOLL_REPLAY_BONE index=0 name=x parent=-1 position=0,0,1 rotation=0,0,0,1\n'
              'RAGDOLL_REPLAY_INITIAL body=0 body_id=1 position=0,0,1 rotation=0,0,0,1 '
              'velocity=0,0,0 angular_velocity=0,0,0\n')
        with tempfile.TemporaryDirectory() as directory:
            a=Path(directory)/'a'; b=Path(directory)/'b'
            a.write_text(text); b.write_text(text)
            report=module.compare(a,b)
            self.assertTrue(report['initial_state_matches'])
            self.assertFalse(report['qualification_ready'])
            b.write_text(text.replace('body_id=1','body_id=2'))
            self.assertFalse(module.compare(a,b)['initial_state_matches'])
            b.write_text(text.replace('angular_velocity=0,0,0',''))
            with self.assertRaises(KeyError): module.compare(a,b)


if __name__=='__main__': unittest.main()
