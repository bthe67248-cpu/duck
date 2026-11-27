r"""train_ppo.py -- train a Duckling V4 walking policy with PPO on the CPU (MuJoCo, multiprocessing), export ONNX.

    python training/train_ppo.py --out training/runs/walk_v4 [--iters 3000] [--workers 4] [--envs-per-worker 32]
                                 [--init-onnx policies/velstand.onnx] [--resume training/runs/walk_v4/last.pt]
                                 [--model model/duck_v4_gc_scene.xml]

Writes <out>/last.pt (resumable checkpoint), <out>/best.pt, <out>/log.csv and <out>/policy.onnx (the actor with the
observation normalizer baked in: input "obs" [1, 61] -> output "actions" [1, 14], the same contract as the
pretrained Microduck policies, so scripts/motions.py --walk-policy and world_sim run it directly).

--init-onnx warm-starts the actor from an ONNX policy of the same shape (e.g. upstream velstand.onnx, if you have
it): the MLP weights are copied, and its baked-in normalizer (if any) is absorbed into the first layer. Fine-tuning
from a policy that already walks needs far fewer steps than training from scratch.

Architecture and PPO settings follow upstream microduck_rl (actor and critic MLP 512-256-128 ELU, gamma 0.99,
lambda 0.95, clip 0.2, entropy 0.01, learning rate adapted per iteration on a KL target of 0.01 within [1e-4, 1e-3], epochs stopped early above 0.02). The critic additionally sees privileged
state (true base velocity, height, foot contacts, servo delay and stiffness), which only exists in simulation.
"""
import argparse, csv, json, math, os, sys, time
import multiprocessing as mp
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, str(Path(__file__).resolve().parent))
from duck_env import ACT_DIM, CRIT_EXTRA, OBS_DIM, DuckEnv  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------- workers
def worker(conn, xml, n_envs, seed, cfg):
    torch.set_num_threads(1)
    envs = [DuckEnv(xml=xml, seed=seed * 1000 + i, cfg=cfg) for i in range(n_envs)]
    obs = np.stack([e.obs() for e in envs]); crit = np.stack([e.critic_extra() for e in envs])
    conn.send((obs, crit))
    stats = {}
    while True:
        msg = conn.recv()
        if msg[0] == "step":
            acts = msg[1]
            O, C, R, D, T = [], [], [], [], []
            for e, a in zip(envs, acts):
                o, r, done, fell, info = e.step(a)
                for k, v in info.items():
                    stats[k] = stats.get(k, 0.0) + v * 0.02
                trunc = done and not fell
                if done:
                    stats["episodes"] = stats.get("episodes", 0) + 1
                    stats["falls"] = stats.get("falls", 0) + int(fell)
                    stats["ep_len_s"] = stats.get("ep_len_s", 0) + e.t
                    o = e.reset()
                O.append(o); C.append(e.critic_extra()); R.append(r); D.append(done); T.append(trunc)
            conn.send((np.stack(O), np.stack(C), np.array(R, np.float32), np.array(D), np.array(T)))
        elif msg[0] == "stats":
            conn.send(stats); stats = {}
        elif msg[0] == "cfg":
            for e in envs:
                e.cfg.update(msg[1])
            conn.send(True)
        elif msg[0] == "close":
            return


class VecEnv:
    def __init__(self, xml, workers, per, cfg, seed):
        ctx = mp.get_context("fork")
        self.pipes, self.procs = [], []
        for w in range(workers):
            a, b = ctx.Pipe()
            p = ctx.Process(target=worker, args=(b, xml, per, seed + w, cfg), daemon=True); p.start()
            self.pipes.append(a); self.procs.append(p)
        first = [p.recv() for p in self.pipes]
        self.obs = np.concatenate([f[0] for f in first]); self.crit = np.concatenate([f[1] for f in first])
        self.per = per

    def step(self, acts):
        for i, p in enumerate(self.pipes):
            p.send(("step", acts[i * self.per:(i + 1) * self.per]))
        res = [p.recv() for p in self.pipes]
        return [np.concatenate([r[k] for r in res]) for k in range(5)]

    def stats(self):
        for p in self.pipes: p.send(("stats",))
        tot = {}
        for p in self.pipes:
            for k, v in p.recv().items(): tot[k] = tot.get(k, 0) + v
        return tot

    def set_cfg(self, upd):
        for p in self.pipes: p.send(("cfg", upd))
        for p in self.pipes: p.recv()

    def close(self):
        for p in self.pipes: p.send(("close",))


# ---------------------------------------------------------------------------- model
class Normalizer(nn.Module):
    def __init__(self, n, clip=1e4):   # effectively unclipped, like upstream rsl_rl's empirical normalization
        super().__init__()
        self.register_buffer("mean", torch.zeros(n)); self.register_buffer("var", torch.ones(n))
        self.register_buffer("count", torch.tensor(1e-4)); self.clip = clip

    def update(self, x):
        bm, bv, bc = x.mean(0), x.var(0, unbiased=False), x.shape[0]
        d = bm - self.mean; tot = self.count + bc
        self.mean += d * bc / tot
        self.var = (self.var * self.count + bv * bc + d ** 2 * self.count * bc / tot) / tot
        self.count = tot

    def forward(self, x):
        return torch.clamp((x - self.mean) / torch.sqrt(self.var + 1e-8), -self.clip, self.clip)


def mlp(i, o, hidden=(512, 256, 128)):
    layers, n = [], i
    for h in hidden:
        layers += [nn.Linear(n, h), nn.ELU()]; n = h
    layers.append(nn.Linear(n, o))
    return nn.Sequential(*layers)


class ActorCritic(nn.Module):
    def __init__(self):
        super().__init__()
        self.obs_norm = Normalizer(OBS_DIM); self.crit_norm = Normalizer(OBS_DIM + CRIT_EXTRA)
        self.actor = mlp(OBS_DIM, ACT_DIM); self.critic = mlp(OBS_DIM + CRIT_EXTRA, 1)
        self.log_std = nn.Parameter(torch.full((ACT_DIM,), math.log(0.5)))
        nn.init.uniform_(self.actor[-1].weight, -0.01, 0.01); nn.init.zeros_(self.actor[-1].bias)

    def dist(self, obs):
        mu = self.actor(self.obs_norm(obs))
        return torch.distributions.Normal(mu, self.log_std.exp().expand_as(mu))

    def value(self, cobs):
        return self.critic(self.crit_norm(cobs)).squeeze(-1)


class Exported(nn.Module):
    """What goes into the ONNX: normalizer + actor mean."""
    def __init__(self, ac):
        super().__init__(); self.norm, self.actor = ac.obs_norm, ac.actor

    def forward(self, obs):
        return self.actor(self.norm(obs))


def export_onnx(ac, path, meta):
    import onnx
    m = Exported(ac).eval()
    torch.onnx.export(m, torch.zeros(1, OBS_DIM), str(path), input_names=["obs"], output_names=["actions"],
                      dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}}, opset_version=17, dynamo=False)
    model = onnx.load(str(path))
    for k, v in meta.items():
        e = model.metadata_props.add(); e.key, e.value = k, str(v)
    onnx.save(model, str(path))


def warm_start_from_onnx(ac, path):
    """Copy an ONNX MLP policy (Gemm/MatMul layers, 61 -> 512 -> 256 -> 128 -> 14) into the actor. A Sub/Div (or
    Mul) normalizer in front is folded into the first layer; the actor's own normalizer is then left at identity
    and frozen-free (it keeps learning, starting from the source's statistics)."""
    import onnx
    from onnx import numpy_helper
    g = onnx.load(str(path)).graph
    init = {t.name: numpy_helper.to_array(t) for t in g.initializer}
    mats = []
    for n in g.node:
        if n.op_type == "Gemm":
            W = init[n.input[1]]; b = init[n.input[2]]
            trans = next((a.i for a in n.attribute if a.name == "transB"), 0)
            mats.append((W if trans else W.T, b))
        elif n.op_type == "MatMul" and n.input[1] in init:
            mats.append((init[n.input[1]].T, None))
        elif n.op_type == "Add" and mats and mats[-1][1] is None and n.input[1] in init:
            mats[-1] = (mats[-1][0], init[n.input[1]])
    lin = [l for l in ac.actor if isinstance(l, nn.Linear)]
    if len(mats) != len(lin) or any(W.shape != tuple(l.weight.shape) for (W, _), l in zip(mats, lin)):
        raise ValueError(f"{path}: layer shapes {[W.shape for W, _ in mats]} do not match the actor")
    # normalizer constants (Sub then Div, or Sub then Mul), if present before the first layer
    mean = np.zeros(OBS_DIM, np.float32); scale = np.ones(OBS_DIM, np.float32)
    for n in g.node:
        if n.op_type in ("Gemm", "MatMul"):
            break
        c = [init[i] for i in n.input if i in init]
        if n.op_type == "Sub" and c: mean = c[0].reshape(-1).astype(np.float32)
        if n.op_type == "Div" and c: scale = 1.0 / c[0].reshape(-1).astype(np.float32)
        if n.op_type == "Mul" and c: scale = c[0].reshape(-1).astype(np.float32)
    with torch.no_grad():
        for (W, b), l in zip(mats, lin):
            l.weight.copy_(torch.as_tensor(W)); l.bias.copy_(torch.as_tensor(b if b is not None else 0))
        ac.obs_norm.mean.copy_(torch.as_tensor(mean)); ac.obs_norm.var.copy_(torch.as_tensor(1.0 / scale ** 2))
        ac.obs_norm.count.fill_(1e6)                          # trust the source statistics; they still adapt
        ac.log_std.fill_(math.log(0.15))
    return len(mats)


# ---------------------------------------------------------------------------- PPO
def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "training" / "runs" / "walk_v4"))
    ap.add_argument("--model", default=str(ROOT / "model" / "duck_v4_gc_scene.xml"))
    ap.add_argument("--iters", type=int, default=3000)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 0))
    ap.add_argument("--envs-per-worker", type=int, default=32)
    ap.add_argument("--steps", type=int, default=24, help="steps per env per iteration (upstream: 24)")
    ap.add_argument("--epochs", type=int, default=5); ap.add_argument("--minibatches", type=int, default=4)
    ap.add_argument("--lr", type=float, default=3e-4); ap.add_argument("--entropy", type=float, default=0.01)
    ap.add_argument("--lr-min", type=float, default=1e-4); ap.add_argument("--lr-max", type=float, default=1e-3)
    ap.add_argument("--target-kl", type=float, default=0.01)
    ap.add_argument("--init-onnx"); ap.add_argument("--resume")
    ap.add_argument("--critic-warmup", type=int, default=0,
                    help="iterations at the start that train only the critic (actor frozen); use ~30 with --init-onnx "
                         "so an untrained critic cannot drag a pretrained actor away")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--cfg", default="{}", help="JSON overrides of duck_env.DEFAULT_CFG")
    ap.add_argument("--max-hours", type=float, default=1e9)
    a = ap.parse_args(argv)
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(a.seed); torch.set_num_threads(2)
    cfg = json.loads(a.cfg)
    ac = ActorCritic()
    opt = torch.optim.Adam(ac.parameters(), lr=a.lr)
    it0, best = 0, -1e9
    if a.resume:
        ck = torch.load(a.resume, weights_only=False)
        ac.load_state_dict(ck["model"]); opt.load_state_dict(ck["opt"]); it0 = ck["iter"] + 1; best = ck.get("best", best)
    elif a.init_onnx:
        print(f"warm start: {warm_start_from_onnx(ac, a.init_onnx)} layers from {a.init_onnx}", flush=True)
    venv = VecEnv(a.model, a.workers, a.envs_per_worker, cfg, a.seed * 100)
    N = a.workers * a.envs_per_worker
    obs, crit = torch.as_tensor(venv.obs), torch.as_tensor(venv.crit)
    lr = a.lr
    logf = open(out / "log.csv", "a", newline=""); log = csv.writer(logf)
    if it0 == 0: log.writerow(["iter", "steps", "mean_step_reward", "ep_len_s", "fall_rate", "lr", "std", "kl", "fps", "terms"])
    t_start = time.time()
    for it in range(it0, a.iters):
        t0 = time.time()
        # curriculum: smoothness is introduced after the gait exists (upstream lesson), pushes ramp up
        frac = min(1.0, it / max(1, 0.5 * a.iters))
        venv.set_cfg({"w_rate": -0.1 - 0.4 * frac, "push_dv": 0.1 + 0.2 * frac})
        B_obs, B_crit, B_act, B_logp, B_rew, B_done, B_val, B_trunc_val = [], [], [], [], [], [], [], []
        with torch.no_grad():
            for _ in range(a.steps):
                dist = ac.dist(obs); act = dist.sample()
                logp = dist.log_prob(act).sum(-1); val = ac.value(torch.cat([obs, crit], 1))
                o2, c2, r, d, tr = venv.step(act.numpy())
                o2, c2 = torch.as_tensor(o2), torch.as_tensor(c2)
                r = torch.as_tensor(r); d = torch.as_tensor(d); tr = torch.as_tensor(tr)
                # time-outs are not failures: bootstrap with the current state's value (rsl_rl's approximation)
                r = r + 0.99 * tr.float() * val
                B_obs.append(obs); B_crit.append(crit); B_act.append(act); B_logp.append(logp)
                B_rew.append(r); B_done.append(d.float()); B_val.append(val)
                obs, crit = o2, c2
            last_val = ac.value(torch.cat([obs, crit], 1))
        T = a.steps
        rew, done, val = torch.stack(B_rew), torch.stack(B_done), torch.stack(B_val)
        # normalizers move once per iteration; the old log-probs are recomputed with the moved normalizer so the
        # PPO ratio compares like with like
        with torch.no_grad():
            allo, allc = torch.cat(B_obs), torch.cat(B_crit)
            ac.obs_norm.update(allo); ac.crit_norm.update(torch.cat([allo, allc], 1))
            B_logp = list(ac.dist(allo).log_prob(torch.cat(B_act)).sum(-1).reshape(T, N))
        adv = torch.zeros_like(rew); gae = torch.zeros(N)
        for t in reversed(range(T)):
            nv = last_val if t == T - 1 else val[t + 1]
            delta = rew[t] + 0.99 * nv * (1 - done[t]) - val[t]
            gae = delta + 0.99 * 0.95 * (1 - done[t]) * gae
            adv[t] = gae
        ret = adv + val
        flat = lambda x: x.reshape(T * N, *x.shape[2:])
        Xo, Xc, Xa, Xl, Xadv, Xret, Xv = map(flat, (torch.stack(B_obs), torch.stack(B_crit), torch.stack(B_act),
                                                    torch.stack(B_logp), adv, ret, val))
        Xadv = (Xadv - Xadv.mean()) / (Xadv.std() + 1e-8)
        kls = []
        mb = T * N // a.minibatches
        old_mu = ac.actor(ac.obs_norm(Xo)).detach(); old_std = ac.log_std.exp().detach()
        for _ in range(a.epochs):
            perm = torch.randperm(T * N)
            for k in range(a.minibatches):
                idx = perm[k * mb:(k + 1) * mb]
                dist = ac.dist(Xo[idx]); lp = dist.log_prob(Xa[idx]).sum(-1)
                ratio = torch.exp(lp - Xl[idx])
                pl = -torch.min(ratio * Xadv[idx], torch.clamp(ratio, 0.8, 1.2) * Xadv[idx]).mean()
                v = ac.value(torch.cat([Xo[idx], Xc[idx]], 1))
                vc = Xv[idx] + torch.clamp(v - Xv[idx], -0.2, 0.2)
                vl = torch.max((v - Xret[idx]) ** 2, (vc - Xret[idx]) ** 2).mean()
                ent = dist.entropy().sum(-1).mean()
                loss = (vl if it < a.critic_warmup else pl + 1.0 * vl - a.entropy * ent)
                opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(ac.parameters(), 1.0); opt.step()
            with torch.no_grad():
                ac.log_std.clamp_(math.log(0.05), math.log(1.0))
                # analytic KL(old || new) of the diagonal Gaussians over the whole batch
                mu, sd = ac.actor(ac.obs_norm(Xo)), ac.log_std.exp()
                kl = (torch.log(sd / old_std) + (old_std ** 2 + (old_mu - mu) ** 2) / (2 * sd ** 2) - 0.5).sum(-1).mean().item()
                kls.append(kl)
            if kl > 2.0 * a.target_kl:                        # early stop: this batch has moved the policy enough
                break
        # learning rate adapted once per iteration on the KL target, bounded (rsl_rl-style schedule, but per
        # iteration: with Adam every minibatch step moves the policy by about the same amount, so a per-minibatch
        # rule ratchets the rate down to its floor on small CPU batches)
        if it < a.critic_warmup: pass                         # actor frozen: KL is 0, keep the starting rate
        elif kls[-1] > 1.5 * a.target_kl: lr = max(a.lr_min, lr / 1.5)
        elif kls[-1] < 0.5 * a.target_kl: lr = min(a.lr_max, lr * 1.5)
        for gp in opt.param_groups: gp["lr"] = lr
        st = venv.stats()
        eps = max(1, st.get("episodes", 0))
        mean_r = float(rew.mean())
        fps = T * N / (time.time() - t0)
        terms = {k: round(v / (T * N * 0.02) , 3) for k, v in st.items() if k not in ("episodes", "falls", "ep_len_s")}
        row = [it, (it + 1) * T * N, round(mean_r, 4), round(st.get("ep_len_s", 0) / eps, 2),
               round(st.get("falls", 0) / eps, 3), f"{lr:.2e}", round(float(ac.log_std.detach().exp().mean()), 3),
               round(float(np.mean(kls)), 4), int(fps), json.dumps(terms)]
        log.writerow(row); logf.flush()
        if it % 10 == 0:
            print(" ".join(str(x) for x in row[:9]), terms, flush=True)
        ck = dict(model=ac.state_dict(), opt=opt.state_dict(), iter=it, best=best, cfg=cfg, args=vars(a))
        if it % 25 == 0 or it == a.iters - 1:
            torch.save(ck, out / "last.pt")
        if mean_r > best and it > 20:
            best = mean_r; ck["best"] = best; torch.save(ck, out / "best.pt")
        if (time.time() - t_start) / 3600 > a.max_hours:
            print("time budget reached", flush=True); torch.save(ck, out / "last.pt"); break
    venv.close()
    meta = dict(producer="duck/training/train_ppo.py", model=Path(a.model).name, iterations=it + 1,
                obs_dim=OBS_DIM, action_dim=ACT_DIM, control_hz=50, init_onnx=a.init_onnx or "")
    export_onnx(ac, out / "policy.onnx", meta)
    print("wrote", out / "policy.onnx", flush=True)


if __name__ == "__main__":
    main()
