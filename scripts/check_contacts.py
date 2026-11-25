r"""check_contacts.py -- S1 validation of the head/jaw/body collision proxies on model/duck_v4_gc_scene.xml.

Usage:  .venv\Scripts\python.exe scripts\check_contacts.py
1. STAND pose: any contact other than feet/floor is reported.
2. Joint-range sweep (head chain over its full range, legs over a walking-size range, random combinations):
   robot-robot contacts that involve a head/jaw/body proxy are counted per geom pair. They must be empty (or explained).
3. Lie-down drops: the robot is dropped on its front, back, left and right side; which proxies reach the floor.
Exits 1 for STAND self-contact OR a head/jaw/body clash in the sampled operating
envelope. --report-only restores the old stand-only exit status. Random sampling
does not prove full-range clearance or get-up feasibility. Historical reports are
preserved: the default output is results/hardware_readiness/contact_check.txt.
"""
import argparse, hashlib, json, os, sys, tempfile
from collections import Counter
from pathlib import Path
import mujoco, numpy as np

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("--scene", type=Path, default=ROOT / "model" / "duck_v4_gc_scene.xml")
ap.add_argument("--samples", type=int, default=6000)
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--output", type=Path, default=ROOT / "results/hardware_readiness/contact_check.txt")
ap.add_argument("--report-only", action="store_true", help="report envelope clashes without failing (diagnostics only)")
args = ap.parse_args()
if args.samples < 1:
    ap.error("--samples must be positive")
m = mujoco.MjModel.from_xml_path(str(args.scene)); d = mujoco.MjData(m)
key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")
if key < 0:
    ap.error("scene has no sim_start keyframe")
lines = []


def log(s=""):
    print(s); lines.append(s)


def gname(g):
    n = m.geom(g).name
    return n or f"{m.body(m.geom_bodyid[g]).name}/geom{g}"


PROXY = {m.geom(i).name for i in range(m.ngeom) if m.geom(i).name.endswith("_collision") and
         m.geom(i).name.split("_")[0] in ("head", "bill", "jaw", "body")}
# torso proxies are the body shell's outer blocks (make_duck.body_proxies); derive the required
# names from the files the model is built from, so the list cannot go stale when the shell changes
# (head and jaw ones come from scripts/collision_proxies.json, the same file make_duck.py reads)
params = json.loads((ROOT / "design" / "v4" / "v4_params.json").read_text())
proxy_spec = json.loads((ROOT / "scripts" / "collision_proxies.json").read_text())
required = ({p["name"] for p in proxy_spec["head"] + proxy_spec["jaw"]} |
            {f"body_{b['name']}_collision" for b in params["body"]["outer"]})
if not required <= PROXY:
    ap.error("contact gate needs the ground-contact model with all head/jaw/body collision proxies; missing " +
             ", ".join(sorted(required - PROXY)))
for name in required:
    geom = m.geom(name).id
    if m.geom_contype[geom] == 0 or m.geom_conaffinity[geom] == 0:
        ap.error(f"{name}: disabled collision mask cannot certify clearance")
log(f"scene: {args.scene.name}; sha256: {hashlib.sha256(args.scene.read_bytes()).hexdigest()}")
# MuJoCo serializes compiled numeric inputs and mesh data here, covering included
# XML/mesh changes that a scene-file hash alone would miss.
with tempfile.TemporaryDirectory() as temp:
    compiled = Path(temp) / "model.mjb"
    mujoco.mj_saveModel(m, str(compiled), None)
    log(f"compiled model sha256: {hashlib.sha256(compiled.read_bytes()).hexdigest()}")
log(f"sampling seed: {args.seed}; poses per envelope: {args.samples}")
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

rng = np.random.default_rng(args.seed)
jn = {m.actuator(i).name: m.actuator_trnid[i, 0] for i in range(14)}
qa = {n: m.jnt_qposadr[j] for n, j in jn.items()}
rg = {n: m.jnt_range[j].copy() for n, j in jn.items()}
home = d.qpos.copy()
head = ["neck_pitch", "head_pitch", "head_yaw", "head_roll"]
legs = [n for n in jn if n not in head]
# operating envelope = what the pretrained policies / head commands use (about STAND); full = every joint limit
ENV = dict(neck_pitch=0.4, head_pitch=0.6, head_yaw=1.2, head_roll=0.44)
N = args.samples


def sweep(full):
    pairs = Counter()
    for k in range(N):
        pose_pairs = set()
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
                pose_pairs.add(tuple(sorted((a, b))))
        pairs.update(pose_pairs)
    return pairs


log(f"\n2a. {N} random poses inside the operating envelope (head {ENV} about STAND, legs +-0.3 rad): self-contacts with head/jaw/body proxies")
env = sweep(False)
if not env:
    log("   none")
for (a, b), n in env.most_common():
    log(f"   {a} <-> {b}: {n} poses ({100*n/N:.1f}%)  <== OPERATING-ENVELOPE CLASH (contact kept enabled)"); findings += 1
log(f"\n2b. INFORMATIONAL, {N} random poses over the FULL head joint limits and legs +-0.5 rad: real geometric clashes of the "
    "head/jaw/body with neck/torso/legs (kept enabled; keep head commands out of them on hardware)")
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
log(f"\nSTAND-pose self contacts: {bad}; operating-envelope clash pairs: {findings}")
log("Random samples are a regression check, not a clearance certificate. Full-range findings and drops remain informational.")
failed = bool(bad or (findings and not args.report_only))
log("GATE: " + ("FAIL" if failed else "PASS (sampled poses only)") +
    (" [REPORT ONLY: envelope clashes do not fail]" if args.report_only else ""))
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text("\n".join(lines) + "\n")
sys.exit(1 if failed else 0)
