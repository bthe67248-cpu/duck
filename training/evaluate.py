r"""evaluate.py -- score walking-policy candidates on the motions.py suite plus walking under servo variations.

    python training/evaluate.py policies/velstand.onnx training/runs/walk_v4_ft/cand_*.onnx [--quick] [--out results/v4_policy/eval.md]

Per candidate: every motion of `scripts/motions.py all` (8 tests; --quick: walk, obstacles, fall_getup) and `walk` with
servo stiffness x0.8 / x1.2 and servo delay 10 / 40 ms (the Feetech servos are not measured yet, so a policy should
not depend on the model's exact numbers). Score = tests passed. Writes a Markdown table. The motions run one after
another (each is a separate process, so it is slow: about 10 minutes per candidate for the full set).
"""
import argparse, hashlib, json, platform, re, subprocess, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SUITE = ["balance", "walk", "obstacles", "fall_getup", "sitstand", "roulade", "pick", "dance"]
VARIANTS = [("kp x0.8", ["--kp-scale", "0.8"]), ("kp x1.2", ["--kp-scale", "1.2"]),
            ("delay 10 ms", ["--servo-delay", "0.01"]), ("delay 40 ms", ["--servo-delay", "0.04"])]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def provenance(policies):
    """Everything needed to tell later which code, model and policy produced a result (review ff34442 B06)."""
    git = lambda *a: subprocess.run(["git", "-C", str(ROOT), *a], capture_output=True, text=True).stdout.strip()
    import mujoco, numpy, onnxruntime
    scene = ROOT / "model" / "duck_v4_gc_scene.xml"
    m = mujoco.MjModel.from_xml_path(str(scene))
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        mjb = Path(d) / "model.mjb"; mujoco.mj_saveModel(m, str(mjb), None); model_sha = sha256(mjb)
    return dict(
        date=time.strftime("%Y-%m-%d %H:%M:%S %z"), commit=git("rev-parse", "HEAD"),
        dirty=bool(git("status", "--porcelain", "--untracked-files=no")),
        scene=str(scene.relative_to(ROOT)), compiled_model_sha256=model_sha,   # covers included XML and meshes
        robot_mass_kg=round(float(m.body_subtreemass[1]), 4),
        policies={Path(p).name: sha256(p) for p in policies},
        versions=dict(python=platform.python_version(), mujoco=mujoco.__version__, numpy=numpy.__version__,
                      onnxruntime=onnxruntime.__version__),
        motions_cmd="scripts/motions.py <motion> --walk-policy <policy> (defaults: speed 0.3 m/s, model servo delay)",
        real_time_factor="n/a: motions.py steps the simulation as fast as it can (not paced)")


def run(motion, policy, extra, tmp):
    cmd = [sys.executable, str(ROOT / "scripts" / "motions.py"), motion, "--walk-policy", str(policy), "--out", str(tmp)] + extra
    out = subprocess.run(cmd, capture_output=True, text=True).stdout.strip().splitlines()
    line = next((l for l in reversed(out) if l.startswith(motion)), "")
    st = line.split()[1] if len(line.split()) > 1 else "ERROR"
    walk = re.findall(r"travel [0-9.]+ m|lateral drift [0-9.]+|heading change [0-9]+", line)
    return st, ", ".join(walk)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("policies", nargs="+"); ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out", default=str(ROOT / "results" / "v4_policy" / "eval.md"))
    a = ap.parse_args(argv)
    tests = (["walk", "obstacles", "fall_getup"] if a.quick else SUITE)
    cols = tests + [f"walk, {v}" for v, _ in VARIANTS]
    rows, tmp = [], Path("/tmp") / "duck_eval"
    for p in a.policies:
        res, details = [], []
        for t in tests:
            st, det = run(t, p, [], tmp); res.append(st)
            if t == "walk": details.append(f"nominal: {det}")
        for v, extra in VARIANTS:
            st, det = run("walk", p, extra, tmp); res.append(st); details.append(f"{v}: {det}")
        score = sum(r == "PASS" for r in res)
        rows.append((Path(p).name, score, res, details))
        print(f"{Path(p).name}: {score}/{len(res)}  " + " ".join(res), flush=True)
    prov = provenance(a.policies)
    md = ["# Walking-policy evaluation", "", "Provenance:", "", "```json", json.dumps(prov, indent=1), "```", "",
          "| policy | score | " + " | ".join(cols) + " |", "|-" * (len(cols) + 2) + "|"]
    for name, score, res, _ in rows:
        md.append(f"| `{name}` | {score}/{len(res)} | " + " | ".join(res) + " |")
    md.append("")
    for name, _, _, det in rows:
        md.append(f"- `{name}` walk: " + "; ".join(det))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text("\n".join(md) + "\n")
    Path(a.out).with_suffix(".json").write_text(json.dumps(dict(provenance=prov, columns=cols, results=[
        dict(policy=n, score=sc, results=dict(zip(cols, res)), walk=det) for n, sc, res, det in rows]), indent=1))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
