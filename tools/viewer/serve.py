"""Serve the behaviour viewer (read-only, localhost).

usage: serve.py [--port 8765] [--data DIR]

/            the viewer page (tools/viewer/index.html, viewer.js)
/data/...    build_data.py output (default: <dev root>/viewer-data)
/files/...   film clips (.mp4) under the dev root, with byte ranges so the
             video can seek
"""
import http.server
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.json': 'application/json',
         '.mp4': 'video/mp4', '.css': 'text/css', '.png': 'image/png'}


def resolve(base, rel):
    path = (base / rel).resolve()
    return path if path.is_relative_to(base.resolve()) and path.is_file() else None


class Handler(http.server.BaseHTTPRequestHandler):
    data = ROOT / 'viewer-data'

    def log_message(self, *args):
        pass

    def do_GET(self):
        url = self.path.split('?', 1)[0]
        rel = url.lstrip('/')
        if rel in ('', 'index.html'):
            path = HERE / 'index.html'
        elif rel == 'viewer.js':
            path = HERE / 'viewer.js'
        elif rel.startswith('data/'):
            path = resolve(self.data, rel[5:])
        elif rel.startswith('files/') and rel.endswith('.mp4'):
            path = resolve(ROOT, rel[6:])
        else:
            path = None
        if path is None or not path.exists():
            self.send_error(404)
            return
        size = path.stat().st_size
        ctype = TYPES.get(path.suffix, 'application/octet-stream')
        rng = re.match(r'bytes=(\d*)-(\d*)', self.headers.get('Range', ''))
        start, end = 0, size - 1
        if rng and path.suffix == '.mp4':
            if rng.group(1):
                start = int(rng.group(1))
            if rng.group(2):
                end = min(int(rng.group(2)), size - 1)
            self.send_response(206)
            self.send_header('Content-Range', 'bytes %d-%d/%d' % (start, end, size))
        else:
            self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('Content-Length', str(end - start + 1))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        with open(path, 'rb') as f:
            f.seek(start)
            left = end - start + 1
            while left > 0:
                chunk = f.read(min(1 << 16, left))
                if not chunk:
                    break
                try:
                    self.wfile.write(chunk)
                except (BrokenPipeError, ConnectionResetError):
                    return
                left -= len(chunk)


def main():
    args = sys.argv[1:]
    port = 8765
    if '--port' in args:
        port = int(args[args.index('--port') + 1])
    if '--data' in args:
        Handler.data = Path(args[args.index('--data') + 1])
    server = http.server.ThreadingHTTPServer(('127.0.0.1', port), Handler)
    print('behaviour viewer: http://127.0.0.1:%d/' % port, flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
