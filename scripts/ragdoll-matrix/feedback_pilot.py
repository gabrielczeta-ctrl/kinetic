"""Capped R7 PPO feasibility run. Rewards never replace physical acceptance."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import queue
import subprocess
import threading
import time

import numpy as np
import torch
from torch import nn
from torch.distributions import Normal


CASES = [(60, 0, 0, -1, -1), (45, 0, 0, -1, -1), (60, 1200, 0, 120, -1)]


class Worker:
    def __init__(self, executable, errors):
        self.errors = open(errors, "w", encoding="utf-8")
        self.process = subprocess.Popen([str(executable), "--feedback-pilot"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.errors, text=True, bufsize=1)
        self.messages = queue.Queue()
        def read():
            for line in self.process.stdout:
                self.messages.put(line)
            self.messages.put(None)
        threading.Thread(target=read, daemon=True).start()
        try:
            if self.receive() != "READY 57 6":
                raise RuntimeError("worker protocol mismatch")
        except BaseException:
            self.close()
            raise

    def send(self, text):
        self.process.stdin.write(text + "\n")
        self.process.stdin.flush()

    def receive(self):
        line = self.messages.get(timeout=10)
        if line is None:
            raise RuntimeError("worker exited; inspect its stderr")
        return line.strip()

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.errors.close()


class Policy(nn.Module):
    def __init__(self):
        super().__init__()
        def net(output):
            return nn.Sequential(nn.Linear(63, 64), nn.Tanh(), nn.Linear(64, 64), nn.Tanh(), nn.Linear(64, output))
        self.actor, self.critic = net(6), net(1)
        nn.init.zeros_(self.actor[-1].weight)
        nn.init.zeros_(self.actor[-1].bias)
        self.log_std = nn.Parameter(torch.full((6,), math.log(.10)))

    def distribution(self, observations):
        return Normal(self.actor(observations), self.log_std.clamp(-4, -.5).exp())


def step_message(line):
    parts = line.split()
    if len(parts) != 60 or parts[0] != "STEP":
        raise RuntimeError("malformed STEP: " + line[:120])
    values = np.asarray([float(v) for v in parts[3:]], dtype=np.float32)
    if not np.isfinite(values).all():
        raise RuntimeError("non-finite observation")
    return int(parts[1]), float(parts[2]), values


class Pilot:
    def __init__(self, args):
        self.args = args
        self.root = Path(args.out).resolve()
        if self.root.exists() and any(self.root.iterdir()):
            raise ValueError("output must be fresh")
        self.root.mkdir(parents=True, exist_ok=True)
        torch.set_num_threads(1)
        torch.manual_seed(20260905)
        np.random.seed(20260905)
        self.policy = Policy()
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=3e-4)
        self.started = time.monotonic()
        self.steps = 0
        self.workers = []
        self.journal = gzip.open(self.root / "applied-actions.jsonl.gz", "wt", encoding="utf-8")
        self.events = open(self.root / "events.jsonl", "w", encoding="utf-8", buffering=1)
        try:
            for index in range(3):
                self.workers.append(Worker(Path(args.exe).resolve(), self.root / ("worker-%d-errors.log" % index)))
        except BaseException:
            self.close()
            raise
        self.event("identity", exe=str(Path(args.exe).resolve()), sha256=hashlib.sha256(Path(args.exe).read_bytes()).hexdigest(),
                   seed=20260905, torch=torch.__version__, max_steps=args.max_steps, max_seconds=args.max_seconds,
                   workers=3, cases=CASES)

    def event(self, kind, **values):
        data = dict(kind=kind, steps=self.steps, seconds=round(time.monotonic() - self.started, 3), **values)
        self.events.write(json.dumps(data) + "\n")
        if kind != "update":
            print(json.dumps(data), flush=True)

    def within_budget(self, reserve=0):
        return self.steps + 1260 + reserve <= self.args.max_steps and time.monotonic() - self.started < self.args.max_seconds - 5

    def rollout(self, cases=CASES, stochastic=False, replay_actions=None, label="train"):
        if len(cases) != 3 or not self.within_budget():
            raise RuntimeError("insufficient episode budget")
        obs = []
        for worker, case in zip(self.workers, cases):
            worker.send("EP " + " ".join(str(v) for v in case))
        for worker in self.workers:
            step, _, values = step_message(worker.receive())
            if step != 0:
                raise RuntimeError("episode did not start at zero")
            obs.append(values)
        previous = np.zeros((3, 6), dtype=np.float32)
        states, actions, log_probs, values, rewards, applied = [], [], [], [], [], []
        episode_returns = np.zeros(3)
        summaries = []
        for step in range(420):
            if time.monotonic() - self.started >= self.args.max_seconds:
                raise TimeoutError("twenty-minute pilot limit")
            x = torch.from_numpy(np.concatenate((np.stack(obs), previous), axis=1))
            with torch.no_grad():
                distribution = self.policy.distribution(x)
                latent = distribution.sample() if stochastic else distribution.mean
                logp = distribution.log_prob(latent).sum(-1)
                value = self.policy.critic(x).squeeze(-1)
                proposed = torch.tanh(latent).numpy()
            filtered = .8 * previous + .2 * proposed
            if step < 30:
                filtered.fill(0)
            if replay_actions is not None:
                filtered = np.asarray(replay_actions[step], dtype=np.float32)
            if not np.isfinite(filtered).all() or np.max(np.abs(filtered)) > 1:
                raise RuntimeError("invalid bounded action")
            for worker, action in zip(self.workers, filtered):
                worker.send(" ".join(format(float(v), ".9g") for v in action))
            following, reward = [], []
            for worker in self.workers:
                line = worker.receive()
                if step == 419:
                    prefix, earned, data = line.split(" ", 2)
                    if prefix != "END":
                        raise RuntimeError("missing episode result")
                    reward.append(float(earned))
                    summaries.append(json.loads(data))
                else:
                    next_step, earned, observation = step_message(line)
                    if next_step != step + 1:
                        raise RuntimeError("observation/action timestamp mismatch")
                    following.append(observation)
                    reward.append(earned)
            self.steps += 3
            reward = np.asarray(reward, dtype=np.float32)
            reward -= .01 * np.square(filtered).sum(1) + .02 * np.square(filtered - previous).sum(1)
            episode_returns += reward
            states.append(x.numpy()); actions.append(latent.numpy()); log_probs.append(logp.numpy())
            values.append(value.numpy()); rewards.append(reward); applied.append(filtered.copy())
            previous = filtered
            obs = following
        self.journal.write(json.dumps(dict(label=label, step_end=self.steps, cases=cases,
                                         actions=np.asarray(applied).tolist())) + "\n")
        self.journal.flush()
        self.event("rollout", label=label, stochastic=stochastic, returns=episode_returns.tolist(), metrics=summaries)
        return dict(states=np.asarray(states), actions=np.asarray(actions), log_probs=np.asarray(log_probs),
                    values=np.asarray(values), rewards=np.asarray(rewards), applied=np.asarray(applied),
                    summaries=summaries, returns=episode_returns)

    def update(self, rollout):
        rewards, values = rollout["rewards"], rollout["values"]
        advantages = np.zeros_like(rewards)
        last = np.zeros(3, dtype=np.float32)
        for t in reversed(range(420)):
            next_value = values[t + 1] if t < 419 else np.zeros(3)
            delta = rewards[t] + .99 * next_value - values[t]
            last = delta + .99 * .95 * last
            advantages[t] = last
        returns = advantages + values
        # Actions before request are deliberately ignored by the environment.
        def flatten(array):
            return torch.from_numpy(array[30:].reshape((-1,) + array.shape[2:])).float()
        states, actions, old_logp = (flatten(rollout[k]) for k in ("states", "actions", "log_probs"))
        targets, adv = flatten(returns), flatten(advantages)
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        for _ in range(4):
            for batch in torch.randperm(len(adv)).split(256):
                logp = self.policy.distribution(states[batch]).log_prob(actions[batch]).sum(-1)
                # Tanh's Jacobian cancels in the likelihood ratio at the same
                # stored latent action. No transformed-entropy approximation.
                ratio = (logp - old_logp[batch]).exp()
                actor_loss = -torch.minimum(ratio * adv[batch], ratio.clamp(.8, 1.2) * adv[batch]).mean()
                critic_loss = (self.policy.critic(states[batch]).squeeze(-1) - targets[batch]).square().mean()
                loss = actor_loss + .5 * critic_loss
                if not torch.isfinite(loss):
                    raise RuntimeError("non-finite learning update")
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(list(self.policy.actor.parameters()) + [self.policy.log_std], .5)
                nn.utils.clip_grad_norm_(self.policy.critic.parameters(), .5)
                self.optimizer.step()
        self.event("update", actor_loss=float(actor_loss.detach()), critic_loss=float(critic_loss.detach()))

    @staticmethod
    def rank(result, cases=CASES):
        violations = []
        for m, case in zip(result["summaries"], cases):
            violations.append(sum((m["fall_step"] >= 0, m["airborne"] > 0, m["caps"] > 0,
                                   m["slide"] > 1.5, m["saturated"] * 20 > m["reference_frames"],
                                   m["final_speed"] < .5 * case[0], m["transfers"] < 4)))
        # Selection is diagnostic; full named physical tests remain mandatory.
        return max(violations), sum(violations), -float(result["returns"].mean())

    def close(self):
        for worker in self.workers:
            worker.close()
        self.journal.close()
        self.events.close()

    def run(self):
        baseline = self.rollout(stochastic=False, label="zero-baseline")
        expected = [227, 392, 320]
        if [m["fall_step"] for m in baseline["summaries"]] != expected:
            raise RuntimeError("zero-policy baseline differs from retained matrix")
        repeated = self.rollout(replay_actions=baseline["applied"], label="zero-replay")
        if [m["hash"] for m in repeated["summaries"]] != [m["hash"] for m in baseline["summaries"]]:
            raise RuntimeError("zero-policy replay differs")
        best = self.rank(baseline)
        torch.save(self.policy.state_dict(), self.root / "selected.pt")
        if self.args.check_only:
            self.event("interface_checked", rank=best)
            return
        iteration = 0
        # Reserve four complete batches for frozen evaluation and replay.
        while self.within_budget(reserve=4 * 1260):
            result = self.rollout(stochastic=True)
            self.update(result)
            iteration += 1
            if iteration % 20 == 0 and self.within_budget(reserve=4 * 1260):
                evaluated = self.rollout(stochastic=False, label="training-case-evaluation")
                rank = self.rank(evaluated)
                if rank < best:
                    best = rank
                    torch.save(self.policy.state_dict(), self.root / "selected.pt")
                    self.event("selection", rank=rank, iteration=iteration)
        torch.save(self.policy.state_dict(), self.root / "last.pt")
        self.policy.load_state_dict(torch.load(self.root / "selected.pt", weights_only=True))
        self.event("policy_frozen", rank=best, iterations=iteration)
        if self.within_budget(reserve=3 * 1260):
            selected = self.rollout(label="frozen-training-cases")
            replay = self.rollout(replay_actions=selected["applied"], label="frozen-replay")
            if [m["hash"] for m in selected["summaries"]] != [m["hash"] for m in replay["summaries"]]:
                raise RuntimeError("selected policy replay differs")
            held = json.loads(Path(self.args.held_out).read_text())["cases"]
            cases = [(c["speed"], c["impulse"][0], c["impulse"][1], c["push_step"], -1) for c in held]
            evaluation = self.rollout(cases=cases, label="held-out")
            replay = self.rollout(cases=cases, replay_actions=evaluation["applied"], label="held-out-replay")
            if [m["hash"] for m in evaluation["summaries"]] != [m["hash"] for m in replay["summaries"]]:
                raise RuntimeError("held-out replay differs")
        self.event("stopped", reason="bounded pilot complete", rank=best, qualified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--held-out", required=True)
    parser.add_argument("--max-steps", type=int, default=1_000_000)
    parser.add_argument("--max-seconds", type=int, default=1200)
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    if not 2520 <= args.max_steps <= 1_000_000 or not 10 <= args.max_seconds <= 1200:
        parser.error("pilot limits: 2520..1000000 steps, 10..1200 seconds")
    pilot = Pilot(args)
    try:
        pilot.run()
    except BaseException as error:
        torch.save(pilot.policy.state_dict(), pilot.root / "interrupted.pt")
        pilot.event("stopped", reason=type(error).__name__ + ": " + str(error), qualified=False)
        raise
    finally:
        pilot.close()


if __name__ == "__main__":
    main()
