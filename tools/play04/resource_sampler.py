"""Machine and engine load during a test run (run_watched.py starts it).

Samples every second: machine CPU and free memory, and the CPU and memory of
the openmw*.exe processes started after the sampler (the user's own game,
running before, is left out). stop() returns a summary and writes it with the
samples to OUTPUT/resource-usage.json; the ledger copies the summary into
each case row ('res').

Why (2026-11-01): to know whether more shards or the 4x step multiplier
(OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER) would overload the machine. Without
psutil the run goes on unmeasured.
"""
import json
import threading
import time

try:
    import psutil
except ImportError:
    psutil = None

GB = 1024 ** 3


def _openmw():
    for p in psutil.process_iter(['name']):
        if (p.info['name'] or '').lower().startswith('openmw'):
            yield p


class ResourceSampler:
    def __init__(self, interval=1.0):
        self.interval = interval
        self.samples = []
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if psutil is None:
            return self
        self._protected = {p.pid for p in _openmw()}
        self._procs = {}
        self._start = time.time()
        psutil.cpu_percent(None)  # primes the machine counter
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def _tick(self):
        cpu, rss, count = 0.0, 0, 0
        for p in _openmw():
            if p.pid in self._protected:
                continue
            proc = self._procs.setdefault(p.pid, p)
            try:
                # The first reading of a new process is 0 (no interval yet).
                cpu += proc.cpu_percent(None)
                rss += proc.memory_info().rss
                count += 1
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        mem = psutil.virtual_memory()
        self.samples.append({
            's': round(time.time() - self._start, 1),
            'cpu': psutil.cpu_percent(None),       # machine, % of all cores
            'free_gb': round(mem.available / GB, 2),
            'engines': count,
            'engine_cores': round(cpu / 100, 2),   # openmw CPU in cores
            'engine_gb': round(rss / GB, 2),
        })

    def _run(self):
        while not self._stop.wait(self.interval):
            try:
                self._tick()
            except Exception:  # a measurement must never fail the run
                pass

    def stop(self, output=None):
        if self._thread is None:
            return None
        self._stop.set()
        self._thread.join(timeout=10)
        busy = [s for s in self.samples if s['engines']] or self.samples
        if not busy:
            return None
        avg = lambda key: round(sum(s[key] for s in busy) / len(busy), 1)
        summary = {
            'wall_s': round(time.time() - self._start),
            'cores': psutil.cpu_count(),
            'mem_gb': round(psutil.virtual_memory().total / GB, 1),
            'cpu_avg': avg('cpu'), 'cpu_peak': max(s['cpu'] for s in busy),
            'engine_cores_avg': avg('engine_cores'),
            'engine_cores_peak': max(s['engine_cores'] for s in busy),
            'engine_gb_peak': max(s['engine_gb'] for s in busy),
            'free_gb_min': min(s['free_gb'] for s in busy),
            'engines_peak': max(s['engines'] for s in busy),
        }
        if output is not None and output.is_dir():
            (output / 'resource-usage.json').write_text(
                json.dumps({'summary': summary, 'samples': self.samples}, separators=(',', ':')))
        return summary


def line(summary):
    if summary is None:
        return 'resources: not measured (psutil missing or no samples)'
    s = summary
    return ('resources: %ds, machine CPU avg %.0f%% peak %.0f%% of %d cores; engines %.1f cores avg, '
            '%.1f peak, %.1f GB peak; free memory min %.1f of %.0f GB'
            % (s['wall_s'], s['cpu_avg'], s['cpu_peak'], s['cores'], s['engine_cores_avg'],
               s['engine_cores_peak'], s['engine_gb_peak'], s['free_gb_min'], s['mem_gb']))
