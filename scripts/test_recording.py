"""Bounded, window-only video evidence for unattended Windows integration tests."""

import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


class TestRecording:
    MAX_BYTES = 24 * 1024 * 1024
    TOTAL_BYTES = 512 * 1024 * 1024
    MAX_SECONDS = 60

    def __init__(self, directory):
        self.directory = Path(directory) if directory else None
        self.process = None
        self.log = None
        self.metadata = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.stop()

    def start(self, name):
        self.stop()
        if self.directory is None:
            return
        self.directory.mkdir(parents=True, exist_ok=True)
        used = sum(p.stat().st_size for p in self.directory.iterdir() if p.is_file())
        if used + self.MAX_BYTES > self.TOTAL_BYTES or shutil.disk_usage(self.directory).free < 1024**3:
            print('[ RECORDING] skipped: recording budget or free-space reserve reached')
            return
        ffmpeg = shutil.which('ffmpeg')
        if os.name != 'nt' or not ffmpeg:
            print('[ RECORDING] unavailable: requires Windows and ffmpeg')
            return
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        self.path = self.directory / f'{stamp}.mp4'
        self.metadata = {
            'test': name, 'requested_start_utc': stamp,
            'requested_start_monotonic': time.monotonic(),
            'capture': 'OpenMW window only', 'video_fps': 15,
            'maximum_seconds': self.MAX_SECONDS, 'maximum_bytes': self.MAX_BYTES,
            'synchronization': 'host receipt timestamps; not exact physics/video frame alignment',
        }
        self.log = self.path.with_suffix('.ffmpeg.log').open('w', encoding='utf-8')
        command = [
            ffmpeg, '-hide_banner', '-loglevel', 'warning', '-n',
            '-f', 'gdigrab', '-framerate', '15', '-draw_mouse', '0', '-i', 'title=OpenMW',
            '-an', '-vf', 'scale=1280:720:force_original_aspect_ratio=decrease,pad=1280:720:(ow-iw)/2:(oh-ih)/2',
            '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '28',
            '-maxrate', '1800k', '-bufsize', '3600k', '-pix_fmt', 'yuv420p',
            '-t', str(self.MAX_SECONDS), '-fs', str(self.MAX_BYTES),
            '-movflags', '+faststart', str(self.path),
        ]
        try:
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                            stderr=self.log, creationflags=subprocess.CREATE_NO_WINDOW)
        except OSError as error:
            self.metadata['error'] = str(error)
            self.stop()

    def stop(self):
        if self.metadata is None:
            return
        if self.process is not None:
            try:
                self.process.communicate(b'q\n', timeout=5)
            except (BrokenPipeError, OSError):
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.communicate()
                self.metadata['forced_stop'] = True
            self.metadata['returncode'] = self.process.returncode
        self.metadata['stop_monotonic'] = time.monotonic()
        self.metadata['bytes'] = self.path.stat().st_size if self.path.exists() else 0
        if self.log:
            self.log.close()
            errors = [line for line in self.path.with_suffix('.ffmpeg.log').read_text(encoding='utf-8').splitlines()
                      if 'error' in line.lower() or 'failed' in line.lower()]
            self.metadata['capture_errors'] = errors[:8]
        self.path.with_suffix('.json').write_text(json.dumps(self.metadata, indent=2), encoding='utf-8')
        print(f'[ RECORDING] {self.path} (see sidecar for capture status)')
        self.process = None
        self.log = None
        self.metadata = None
