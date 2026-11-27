r"""evaluate.py -- score walking-policy candidates on the motions.py suite plus walking under servo variations.

    python training/evaluate.py policies/velstand.onnx training/runs/walk_v4_ft/cand_*.onnx [--quick] [--out results/v4_policy/eval.md]

Per candidate: every motion of `scripts/motions.py all` (8 tests; --quick: walk, obstacles, fall_getup) and `walk` with
servo stiffness x0.8 / x1.2 and servo delay 10 / 40 ms (the Feetech servos are not measured yet, so a policy should
not depend on the model's exact numbers). Score = tests passed. Writes a Markdown table. The motions run one after
another (each is a separate process, so it is slow: about 10 minutes per candidate for the full set).
"""
import argparse, re, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SUITE = ["balance", "walk", "obstacles", "fall_getup", "sitstand", "roulade", "pick", "dance"]
VARIANTS = [("kp x0.8", ["--kp-scale", "0.8"]), ("kp x1.2", ["--kp-scale", "1.2"]),
            ("delay 10 ms", ["--servo-delay", "0.01"]), ("delay 40 ms", ["--servo-delay", "0.04"])]


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
    md = ["| policy | score | " + " | ".join(cols) + " |", "|-" * (len(cols) + 2) + "|"]
    for name, score, res, _ in rows:
        md.append(f"| `{name}` | {score}/{len(res)} | " + " | ".join(res) + " |")
    md.append("")
    for name, _, _, det in rows:
        md.append(f"- `{name}` walk: " + "; ".join(det))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text("\n".join(md) + "\n")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
