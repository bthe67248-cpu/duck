"""Tabulate closed-loop trial folders (summary.txt + launch.log) for the corner-navigation comparison.

python trial_metrics.py <runs dir> <prefix> [<prefix> ...]   -> one markdown row per <prefix>_s0..s5 folder
"""
import pathlib
import re
import sys


def row(d):
    t = (d / "summary.txt").read_text() if (d / "summary.txt").exists() else ""
    if not t:
        return None
    log = (d / "launch.log").read_text(errors="ignore") if (d / "launch.log").exists() else ""

    def g(rx):
        m = re.search(rx, t, re.M)
        return m.group(1) if m else "?"
    events = [line for line in t.splitlines() if re.match(r"\s+[0-9.]+ s  ", line)]

    def count(key):
        return sum(key in line for line in events)
    sim_wall = re.findall(r"sim (\d+) s, wall (\d+) s", log)
    beacon = g(r"beacon\s+: reached at (\d+ s)") if "reached at" in t else "not reached"
    return dict(beacon=beacon, coverage=g(r"room coverage\s+: (\d+%)"), walked=g(r"path walked\s+: ([0-9.]+) m"),
                falls=g(r"falls (\d+)"), slam=f"{g(r'mean ([0-9.]+) m')} / {g(r'max ([0-9.]+) m')}",
                guard=count("path clearance guard: turning"), arc=count("steering on a clear arc"),
                skip=count("trying another frontier"),
                manoeuvre=count("local manoeuvre") + count("no clear heading"),
                unstick=log.count("no progress for 8 s: turning"),
                sim_wall="/".join(sim_wall[-1]) if sim_wall else "?", exploration=g(r"exploration\s+: (.*)"))


def main():
    root = pathlib.Path(sys.argv[1])
    keys = ["beacon", "coverage", "walked", "falls", "slam", "guard", "arc", "skip", "manoeuvre", "unstick", "sim_wall",
            "exploration"]
    print("| run | seed | " + " | ".join(keys) + " |")
    print("|" + "-|" * (len(keys) + 2))
    for prefix in sys.argv[2:]:
        for seed in range(6):
            r = row(root / f"{prefix}_s{seed}")
            print(f"| {prefix} | {seed} | " + (" | ".join(str(r[k]) for k in keys) if r else "no summary") + " |")


if __name__ == "__main__":
    main()
