"""Small loader tests; no game, recordings or browser installation required."""
import tempfile
import hashlib
import json
import unittest
from pathlib import Path
from serve import load_trace


class TraceTests(unittest.TestCase):
    def test_joins_by_step_not_row_order_and_preserves_missing_values(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            name = 'example-bodies.csv'
            (root / name).write_text('step,body,x,y,z\n2,0,1,2,3\n0,0,4,5,6\n')
            (root / 'example.csv').write_text('step,time,pelvis_up\n0,0.01,\n2,0.03,0.9\n')
            data = load_trace(root, name, [name])
            self.assertEqual([f['step'] for f in data['frames']], [0, 2])
            self.assertEqual(data['frames'][1]['metrics']['time'], 0.03)
            self.assertIsNone(data['frames'][0]['metrics']['pelvis_up'])
            with self.assertRaises(ValueError):
                load_trace(root, '../example-bodies.csv', [name])

    def test_duplicate_and_nonfinite_positions_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            name = 'example-bodies.csv'
            for rows in ('0,0,nan,0,0\n', '0,0,1,2,3\n0,0,4,5,6\n'):
                (root / name).write_text('step,body,x,y,z\n' + rows)
                with self.assertRaises(ValueError):
                    load_trace(root, name, [name])

    def test_reaction_alignment_and_artifact_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            name = 'scene-bodies.csv'
            body = root / name
            body.write_text('step,body,x,y,z\n0,0,1,2,3\n1,0,4,5,6\n')
            reaction = root / 'scene-reactions.csv'
            reaction.write_text('step,behavior,speed_scale\n1,limp,0.79\n0,none,1\n')
            metadata = {'schema_version': 1, 'files': {name: hashlib.sha256(body.read_bytes()).hexdigest()}}
            (root / 'scene-scene.json').write_text(json.dumps(metadata))
            data = load_trace(root, name, [name])
            self.assertEqual(data['frames'][1]['metrics']['behavior'], 'limp')
            self.assertEqual(data['frames'][1]['metrics']['speed_scale'], 0.79)
            reaction.write_text('step,behavior\n0,none\n')
            with self.assertRaisesRegex(ValueError, 'cover every'):
                load_trace(root, name, [name])
            reaction.write_text('step,behavior\n0,none\n0,limp\n')
            with self.assertRaisesRegex(ValueError, 'duplicate'):
                load_trace(root, name, [name])
            reaction.write_text('step,behavior\n0,none\n1,limp\n')
            body.write_text(body.read_text().replace('1,2,3', '2,2,3'))
            with self.assertRaisesRegex(ValueError, 'hash differs'):
                load_trace(root, name, [name])


if __name__ == '__main__':
    unittest.main()
