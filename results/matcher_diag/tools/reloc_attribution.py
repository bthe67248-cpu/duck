"""reloc_attribution.py DUMP.json ... -- error change caused by relocalisation vs normal matching corrections,
and heading jumps > 10 deg, from slam_replay.py --config '{"dump": "DUMP.json"}' output."""
import json, os, sys
import numpy as np
print(f"{'bag':8s} {'reloc frames':>12s} {'reloc acc':>9s} {'reloc net':>9s} {'reloc hurt':>10s} {'reloc help':>10s} | {'normal net':>10s} {'normal hurt':>11s} | hdg jumps>10deg (reloc/normal)")
for b in sys.argv[1:]:
    d = json.load(open(os.path.expanduser(b))); b = os.path.basename(b).rsplit(".", 1)[0]
    fix = np.array([r["err_after"] - r["err_before"] for r in d])
    dh = np.array([r["hdg_after"] - r["hdg_before"] for r in d])
    rel = np.array([r["reloc"] for r in d]); acc = np.array([r["accepted"] for r in d])
    app = np.array([np.hypot(*r["corr"][:2]) > 1e-4 for r in d])
    ra = rel & app; na = ~rel & app
    print(f"{b:8s} {rel.sum():12d} {(rel&acc).sum():9d} {fix[ra].sum():+9.2f} {fix[ra][fix[ra]>0].sum():+10.2f} {fix[ra][fix[ra]<0].sum():+10.2f} | {fix[na].sum():+10.2f} {fix[na][fix[na]>0].sum():+11.2f} | {(ra & (dh>10)).sum()}/{(na & (dh>10)).sum()}")
