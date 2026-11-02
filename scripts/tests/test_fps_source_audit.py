from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
from fps_source_audit import audit


class SourceAudit(unittest.TestCase):
    def test_stop_boundary_and_missing_tail(self):
        text = 'RAGDOLL_FPS_SOURCE_PLAN schema=1 scenario=S1 epoch_half_tick=20 start_step=60 stop_step=660 end_step=960\n'
        for tick, movement in ((0, 0), (120, 1), (1319, 1), (1320, 0), (1920, 0)):
            text += f'RAGDOLL_FPS_SOURCE epoch_half_tick=20 half_tick={tick} movement={movement} yaw_change=0\n'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'log'
            path.write_text(text)
            self.assertTrue(audit(path)['source_valid'])
            self.assertFalse(audit(path)['qualification_ready'])
            path.write_text(text.replace('half_tick=1320 movement=0', 'half_tick=1320 movement=1'))
            self.assertFalse(audit(path)['source_valid'])
            path.write_text('\n'.join(text.splitlines()[:-1]))
            self.assertFalse(audit(path)['source_valid'])

    def test_restart_turn_at_each_sampling_rate(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'log'
            for spacing in (1, 2, 4):
                text = 'RAGDOLL_FPS_SOURCE_PLAN schema=2 scenario=S2 epoch_half_tick=20 start_step=60 stop_step=300 restart_step=540 final_stop_step=792 end_step=1092\n'
                previous = 0.0
                for tick in range(0, 2185, spacing):
                    movement = int(120 <= tick < 600 or 1080 <= tick < 1584)
                    yaw = min(1.68, max(0.0, (tick-1080)/60))
                    text += f'RAGDOLL_FPS_SOURCE epoch_half_tick=20 half_tick={tick} movement={movement} yaw_change={yaw-previous:.17g} authored_yaw={yaw:.17g}\n'
                    previous = yaw
                path.write_text(text)
                self.assertTrue(audit(path)['source_valid'], spacing)
                path.write_text(text.replace('authored_yaw=1.6799999999999999','authored_yaw=1.7'))
                self.assertFalse(audit(path)['source_valid'])

    def test_straight_restart_rejects_a_yaw_pulse(self):
        text = 'RAGDOLL_FPS_SOURCE_PLAN schema=2 scenario=S2Straight epoch_half_tick=20 start_step=60 stop_step=300 restart_step=540 final_stop_step=792 end_step=1092\n'
        for tick, movement in ((0,0),(120,1),(600,0),(1080,1),(1184,1),(1584,0),(2184,0)):
            text += f'RAGDOLL_FPS_SOURCE epoch_half_tick=20 half_tick={tick} movement={movement} yaw_change=0 authored_yaw=0\n'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'log';path.write_text(text)
            self.assertTrue(audit(path)['source_valid'])
            path.write_text(text.replace('yaw_change=0','yaw_change=0.1',1))
            self.assertFalse(audit(path)['source_valid'])
