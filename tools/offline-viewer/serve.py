"""Read-only local viewer for existing ragdoll scenario CSV exports."""
import argparse
import csv
import hashlib
import json
import math
import socket
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import webbrowser
import urllib.request

WORKSPACE = Path(__file__).resolve().parents[3]
PAGE = Path(__file__).with_name('index.html')
LIMIT = 24 * 1024 * 1024


def catalog(root):
    result = []
    for path in root.glob('runtime-*/**/*-bodies.csv'):
        resolved = path.resolve()
        if not resolved.is_relative_to(root.resolve()):
            continue
        if path.stat().st_size <= LIMIT:
            result.append(path.relative_to(root).as_posix())
    return sorted(result)


def read_csv(path):
    if path.stat().st_size > LIMIT:
        raise ValueError('CSV exceeds the 24 MiB viewer limit')
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def number(value):
    if value is None or value == '':
        return None
    try:
        result = float(value)
        if not math.isfinite(result):
            raise ValueError('Nonfinite measurement')
        return result
    except ValueError:
        if value.lower() in ('nan', 'inf', '-inf', 'infinity', '-infinity'):
            raise ValueError('Nonfinite measurement')
        return value


def load_trace(root, name, allowed):
    if name not in allowed:
        raise ValueError('Select a catalogued trace')
    path = (root / name).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('Trace is outside workspace')
    frames = {}
    for row in read_csv(path):
        step, body = int(row['step']), int(row['body'])
        values = [float(row[key]) for key in ('x', 'y', 'z')]
        if step < 0 or body < 0 or not all(math.isfinite(v) for v in values):
            raise ValueError('Invalid body position or index')
        frame = frames.setdefault(step, {'step': step, 'bodies': {}, 'metrics': {}})
        if body in frame['bodies']:
            raise ValueError('Duplicate step/body')
        frame['bodies'][body] = values
    if not frames:
        raise ValueError('Empty body trace')
    metrics_path = path.with_name(path.name.replace('-bodies.csv', '.csv'))
    if metrics_path.exists():
        if not metrics_path.resolve().is_relative_to(root.resolve()):
            raise ValueError('Metrics are outside workspace')
        for row in read_csv(metrics_path):
            step = int(row['step'])
            if step in frames:
                frames[step]['metrics'] = {k: number(v) for k, v in row.items() if k}
    reaction_path = path.with_name(path.name.replace('-bodies.csv', '-reactions.csv'))
    if reaction_path.exists():
        if not reaction_path.resolve().is_relative_to(root.resolve()):
            raise ValueError('Reaction metrics are outside workspace')
        seen = set()
        for row in read_csv(reaction_path):
            step = int(row['step'])
            if step in seen or step not in frames:
                raise ValueError('Reaction step is duplicate or has no body frame')
            seen.add(step)
            frames[step]['metrics'].update({k: number(v) for k, v in row.items() if k != 'step'})
        if seen != set(frames):
            raise ValueError('Reaction metrics do not cover every body frame')
    metadata = scene_metadata(root, name)
    if metadata:
        for filename, expected in metadata.get('files', {}).items():
            candidate = path.with_name(filename)  # no directories allowed
            if not candidate.resolve().is_relative_to(root.resolve()) or candidate.stat().st_size > LIMIT:
                raise ValueError('Scene artifact is outside workspace or too large')
            if hashlib.sha256(candidate.read_bytes()).hexdigest() != expected:
                raise ValueError('Scene artifact hash differs from its recorded provenance')
    return {'source': name, 'modified': path.stat().st_mtime,
            'metricsSource': metrics_path.relative_to(root).as_posix() if metrics_path.exists() else None,
            'scene': metadata,
            'frames': [frames[k] for k in sorted(frames)]}


def scene_metadata(root, name):
    path = (root / name).with_name(Path(name).name.replace('-bodies.csv', '-scene.json'))
    if not path.exists():
        return None
    if not path.resolve().is_relative_to(root.resolve()) or path.stat().st_size > LIMIT:
        raise ValueError('Scene metadata is outside workspace or too large')
    value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict) or value.get('schema_version') != 1:
        raise ValueError('Unsupported scene metadata')
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=18765)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    allowed = catalog(WORKSPACE)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            nonlocal allowed
            request = urlparse(self.path)
            try:
                if request.path == '/':
                    payload, mime = PAGE.read_bytes(), 'text/html; charset=utf-8'
                elif request.path == '/api/catalog':
                    allowed = catalog(WORKSPACE)
                    payload, mime = json.dumps(allowed).encode(), 'application/json'
                elif request.path == '/api/scenes':
                    allowed = catalog(WORKSPACE)
                    scenes = []
                    for name in allowed:
                        meta = scene_metadata(WORKSPACE, name)
                        if meta and meta.get('task') == 'RXN-01':
                            scenes.append({'trace': name, 'title': meta.get('title', name),
                                           'scene': meta.get('scene'),
                                           'reactions_enabled': meta.get('reactions_enabled')})
                    payload, mime = json.dumps(scenes).encode(), 'application/json'
                elif request.path == '/api/trace':
                    name = parse_qs(request.query).get('name', [''])[0]
                    payload = json.dumps(load_trace(WORKSPACE, name, allowed), allow_nan=False).encode()
                    mime = 'application/json'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(payload)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(payload)
            except (ValueError, KeyError, OSError) as error:
                self.send_error(400, str(error))

        def log_message(self, *_):
            pass

    class LocalServer(ThreadingHTTPServer):
        allow_reuse_address = False

        def server_bind(self):
            if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            super().server_bind()

    url = f'http://127.0.0.1:{args.port}'
    try:
        server = LocalServer(('127.0.0.1', args.port), Handler)
    except OSError:
        # Reopening the launcher must reuse only an identifiable viewer.
        if args.no_browser:
            raise
        with urllib.request.urlopen(url + '/api/scenes', timeout=2) as response:
            scenes = json.load(response)
        if not isinstance(scenes, list):
            raise ValueError('The selected port is not an offline scene viewer')
        webbrowser.open(url)
        return
    print(f'{url} - {len(allowed)} existing traces. Ctrl+C stops the viewer.', flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    server.serve_forever()


if __name__ == '__main__':
    main()
