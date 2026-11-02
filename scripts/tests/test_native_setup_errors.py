import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
from native_setup_errors import setup_error


class SetupErrors(unittest.TestCase):
    def test_failed_camera_or_menu_invalidates_setup(self):
        for text in (
            "[11:49:21.604 E] Can't start Menu[scripts/omw/mwui/init.lua]; Lua error: nil",
            "[11:49:21.765 E] Can't start L@0x1[scripts/omw/camera/camera.lua]; Lua error: nil",
            '[11:49:21.765 E] Lua error: onUpdate failed',
        ):
            self.assertEqual(setup_error(text), text)

    def test_ordinary_output_is_not_a_script_failure(self):
        for text in ('[11:49:21.765 I] TEST_OK case', '[11:49:21.765 W] missing texture',
                     'test expects Lua error: example', '[11:49:21.765 E] missing sound'):
            self.assertIsNone(setup_error(text))
