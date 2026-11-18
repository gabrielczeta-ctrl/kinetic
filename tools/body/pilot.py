"""PILOT: play the player from outside the game (the test_pilot suite).

The game posts an observation and waits, frozen, for the next action; an
action holds for its `steps` frames (pilot_global.lua lists the keys). So a
slow pilot (a person at a prompt, a language model) and a fast one (a script,
a learning loop) use the same seam, and nothing moves while the pilot decides
(up to 5 minutes an action, then the session ends).

usage: python tools/body/pilot.py serve NAME [setup K=V ...] [--play] [--env K=V ...]
           start the game (returns when the session ends; run it in the background)
       python tools/body/pilot.py act NAME K=V ...
           send one action, print the observation that follows it
       python tools/body/pilot.py obs NAME
           print the latest observation
       python tools/body/pilot.py bot NAME [setup K=V ...] [--play] [--seconds S]
           the baseline scripted fighter, start to end (also the example of
           driving a session from Python: Session below)
e.g.   python tools/body/pilot.py bot first weapon=iron+longsword foe=hlaalu+guard --play

setup keys: weapon foe foeweapon dist health foehealth passive hits
Output: runtime-PILOT-NAME in the dev root (obs.jsonl, act.txt in its pilot/).
Never while the player's game is open (the runner refuses).
"""
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sentinel  # noqa: E402


def run_dir(name):
    return sentinel.ROOT / ('runtime-PILOT-%s' % name)


class Session:
    """One pilot session: reset() starts the game, step() sends an action and
    returns the next observation (None when the session is over)."""

    def __init__(self, name):
        self.out = run_dir(name)
        self.dir = Path(str(self.out) + '.pilot')
        self.proc = None

    def lines(self):
        try:
            return (self.dir / 'obs.jsonl').read_text(encoding='utf-8').splitlines()
        except OSError:
            return []

    def send(self, fields):
        with open(self.dir / 'act.txt', 'a', encoding='ascii', newline='\n') as stream:
            stream.write(' '.join('%s=%s' % (k, str(v).replace(' ', '+')) for k, v in fields.items()) + '\n')

    def wait(self, seen, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            lines = self.lines()
            if len(lines) > seen:
                return json.loads(lines[-1])
            if self.proc is not None and self.proc.poll() is not None:
                return None
            time.sleep(0.005)
        return None

    def launch(self, envs, play, timeout=1800):
        import shutil
        subprocess.run(['powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
                        str(sentinel.ROOT / 'tools-workspace/Assert-DiskSpace.ps1')], check=True,
                       stdout=subprocess.DEVNULL)
        shutil.rmtree(self.out, ignore_errors=True)
        shutil.rmtree(self.dir, ignore_errors=True)
        self.dir.mkdir(parents=True)
        cfg = sentinel.TREE / 'files/data/ragdoll'
        env = ['OPENMW_RAGDOLL_SETTINGS=%s' % (cfg / 'ragdoll.cfg'),
               'OPENMW_RAGDOLL_ACTIVE_BODY=%s' % (cfg / 'active-body.cfg'),
               'OPENMW_RAGDOLL_FOOTFALL=1', 'OPENMW_RAGDOLL_DAMAGE_TRACE=1',
               'OPENMW_RAGDOLL_PGC=1,1,1,1', 'OPENMW_RAGDOLL_PGC_BRACE=1']
        env += (sentinel.play_envs() if play else []) + list(envs)
        env.append('OPENMW_RAGDOLL_PILOT_DIR=%s' % self.dir)
        head = subprocess.run(['git', '-C', str(sentinel.TREE), 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True).stdout.strip()
        engine = os.environ.get('RAGDOLL_TEST_ENGINE') or str(sentinel.BUILD / 'RelWithDebInfo/openmw.exe')
        cmd = [sys.executable, str(sentinel.TREE / 'tools/play04/run_watched.py'), '--engine', engine,
               '--engine-source', head + '-dirty', '--output', str(self.out), '--suite', 'test_pilot',
               '--test-filter', '^PILOT', '--parallel', '1', '--suite-timeout', str(timeout),
               '--diagnostic-budget-mib', '1200']
        if os.environ.get('RAGDOLL_TEST_RECORD') == '1':
            cmd += ['--manual-recording']
        for e in env:
            cmd += ['--env', e]
        log = open(str(self.out) + '.runner.log', 'w')
        self.proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)

    def reset(self, setup=None, envs=(), play=True):
        """Start the game; returns the first observation."""
        self.launch(envs, play)
        if self.wait(0, 600) is None:
            raise RuntimeError('the game never said hello, see %s.runner.log' % self.out)
        return self.step(setup or {'weapon': 'iron longsword'}, timeout=120)

    def step(self, action, timeout=60):
        seen = len(self.lines())
        self.send(action)
        return self.wait(seen, timeout)

    def close(self):
        self.send({'end': 1})
        if self.proc is not None:
            self.proc.wait(timeout=180)


def bot(session, obs, seconds):
    """The baseline: close to reach facing the foe, then a click chop and a
    dragged cut in turn, backing off a step after each. Returns a summary."""
    start_foe, start_self = obs.get('f_health'), obs.get('p_health')
    attacks = 0
    while obs and not obs.get('done') and obs['t'] < seconds:
        if obs.get('p_dead') or obs.get('f_dead') or 'dist' not in obs:
            break
        if str(obs.get('p_mode', 'standing')).lower() != 'standing':
            obs = session.step({'steps': 12, 'face': 1})  # down or recovering: wait
        elif obs['dist'] > 125:
            obs = session.step({'steps': 6, 'face': 1, 'fwd': 1, 'run': int(obs['dist'] > 400)})
        else:
            attacks += 1
            if attacks % 2:
                session.step({'steps': 5, 'face': 1, 'lmb': 1})
                obs = session.step({'steps': 50, 'face': 1, 'lmb': 0})
            else:
                session.step({'steps': 12, 'face': 1, 'lmb': 1})
                session.step({'steps': 21, 'face': 1, 'mdx': 1800})
                session.step({'steps': 6, 'face': 1, 'mdx': -1200})
                obs = session.step({'steps': 30, 'face': 1, 'lmb': 0, 'fwd': -0.5})
    last = obs or {}
    return {'attacks': attacks, 'seconds': last.get('t'),
            'foe_health': (start_foe, last.get('f_health')), 'health': (start_self, last.get('p_health')),
            'foe_dead': last.get('f_dead'), 'dead': last.get('p_dead')}


def fields(args):
    return dict(a.split('=', 1) for a in args if '=' in a and not a.startswith('--'))


def main():
    args = sys.argv[1:]
    if len(args) < 2:
        sys.exit(__doc__)
    mode, name = args[0], args[1]
    rest = args[2:]
    envs = [rest[i + 1] for i, a in enumerate(rest) if a == '--env']
    plain = [a for i, a in enumerate(rest) if a != '--env' and (i == 0 or rest[i - 1] not in ('--env', '--seconds'))]
    session = Session(name)
    if mode == 'serve':
        print(json.dumps(session.reset(fields(plain), envs, '--play' in rest)))
        sys.stdout.flush()
        session.proc.wait()
    elif mode == 'act':
        obs = session.step(fields(plain), timeout=120)
        print(json.dumps(obs) if obs else 'no observation (session over?)')
    elif mode == 'obs':
        lines = session.lines()
        print(lines[-1] if lines else 'no observation yet')
    elif mode == 'bot':
        seconds = float(rest[rest.index('--seconds') + 1]) if '--seconds' in rest else 60.0
        obs = session.reset(fields(plain), envs, '--play' in rest)
        summary = bot(session, obs, seconds)
        session.close()
        print(json.dumps(summary))
    else:
        sys.exit(__doc__)


if __name__ == '__main__':
    main()
