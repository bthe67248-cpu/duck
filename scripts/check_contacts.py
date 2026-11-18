r"""check_contacts.py -- S1 validation of the head/shield collision proxies on model/duck_v3_gc_scene.xml.

Usage:  .venv\Scripts\python.exe scripts\check_contacts.py
1. STAND pose: any contact other than feet/floor is reported.
2. Joint-range sweep (head chain over its full range, legs over a walking-size range, random combinations):
   robot-robot contacts that involve a head/shield proxy are counted per geom pair. They must be empty (or explained).
3. Lie-down drops: the robot is dropped on its front, back, left and right side; which proxies reach the floor.
Writes results/post_fix/contact_check.txt and exits 1 only if the STAND pose has a self contact; clashes inside the operating envelope are reported as design findings.
"""
import os, sys
from collections import Counter
from pathlib import Path
import mujoco, numpy as np

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
m = mujoco.MjModel.from_xml_path(str(ROOT / "model" / "duck_v3_gc_scene.xml")); d = mujoco.MjData(m)
key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")
lines = []


def log(s=""):
    print(s); lines.append(s)


def gname(g):
    n = m.geom(g).name
    return n or f"{m.body(m.geom_bodyid[g]).name}/geom{g}"


PROXY = {m.geom(i).name for i in range(m.ngeom) if m.geom(i).name.endswith("_collision") and
         m.geom(i).name.split("_")[0] in ("head", "bill", "shield")}
log("proxy geoms (contype/conaffinity/body): " + ", ".join(
    f"{n}({m.geom_contype[m.geom(n).id]}/{m.geom_conaffinity[m.geom(n).id]}/{m.body(m.geom_bodyid[m.geom(n).id]).name})" for n in sorted(PROXY)))
floor = m.geom("floor").id


def contacts():
    out = []
    for c in d.contact[:d.ncon]:
        out.append((gname(c.geom1), gname(c.geom2), c.geom1 == floor or c.geom2 == floor, c.dist))
    return out


mujoco.mj_resetDataKeyframe(m, d, key); mujoco.mj_forward(m, d)
bad = 0; findings = 0
log("\n1. STAND pose contacts (floor contacts listed, anything else is a defect):")
for a, b, fl, dist in contacts():
    log(f"   {a} <-> {b}  dist {dist*1000:+.2f} mm" + ("" if fl else "   <== SELF CONTACT"))
    bad += (not fl)

rng = np.random.default_rng(1)
jn = {m.actuator(i).name: m.actuator_trnid[i, 0] for i in range(14)}
qa = {n: m.jnt_qposadr[j] for n, j in jn.items()}
rg = {n: m.jnt_range[j].copy() for n, j in jn.items()}
home = d.qpos.copy()
head = ["neck_pitch", "head_pitch", "head_yaw", "head_roll"]
legs = [n for n in jn if n not in head]
# operating envelope = what the pretrained policies / head commands use (about STAND); full = every joint limit
ENV = dict(neck_pitch=0.4, head_pitch=0.6, head_yaw=1.2, head_roll=0.44)
N = 6000


def sweep(full):
    pairs = Counter()
    for k in range(N):
        d.qpos[:] = home
        for n in head:
            d.qpos[qa[n]] = rng.uniform(*rg[n]) if full else np.clip(home[qa[n]] + rng.uniform(-ENV[n], ENV[n]), *rg[n])
        for n in legs:
            d.qpos[qa[n]] = np.clip(home[qa[n]] + rng.uniform(-0.5, 0.5) * (1.0 if full else 0.6), *rg[n])
        mujoco.mj_forward(m, d)
        for c in d.contact[:d.ncon]:
            if c.geom1 == floor or c.geom2 == floor:
                continue
            a, b = gname(c.geom1), gname(c.geom2)
            if a in PROXY or b in PROXY:
                pairs[tuple(sorted((a, b)))] += 1
    return pairs


log(f"\n2a. {N} random poses inside the operating envelope (head {ENV} about STAND, legs +-0.3 rad): self-contacts with head/shield proxies")
env = sweep(False)
if not env:
    log("   none")
for (a, b), n in env.most_common():
    log(f"   {a} <-> {b}: {n} poses ({100*n/N:.1f}%)  <== DESIGN FINDING (real clash, contact kept enabled)"); findings += 1
log(f"\n2b. INFORMATIONAL, {N} random poses over the FULL head joint limits and legs +-0.5 rad: real geometric clashes of the "
    "V3 head/shield with neck/torso/legs (kept enabled; keep head commands out of them on hardware)")
full = sweep(True)
for (a, b), n in full.most_common(12):
    log(f"   {a} <-> {b}: {n} poses ({100*n/N:.1f}%)")
log(f"   ... {len(full)} distinct pairs in total")

log("\n3. Drops (robot placed 0.25 m high in the given attitude, settled 1.5 s): geoms touching the floor")
for label, quat in (("on its front", (0.7071, 0, 0.7071, 0)), ("on its back", (0.7071, 0, -0.7071, 0)),
                    ("on its left side", (0.7071, 0.7071, 0, 0)), ("on its right side", (0.7071, -0.7071, 0, 0)), ("upside down", (0, 1, 0, 0))):
    mujoco.mj_resetDataKeyframe(m, d, key)
    d.qpos[2] = 0.25; d.qpos[3:7] = quat
    for _ in range(750):
        mujoco.mj_step(m, d)
    touch = sorted({a if b == "floor" else b for a, b, fl, _ in contacts() if fl})
    log(f"   {label:18s}: {', '.join(touch) or 'none'}")
log(f"\nSTAND-pose self contacts (defects): {bad}; operating-envelope clash pairs (design findings, not failures): {findings}")
out = ROOT / "results" / "post_fix"; out.mkdir(parents=True, exist_ok=True)
(out / "contact_check.txt").write_text("\n".join(lines) + "\n")
sys.exit(1 if bad else 0)
