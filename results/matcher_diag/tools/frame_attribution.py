"""frame_attribution.py DUMP.json ... -- split SLAM error growth into odometry drift and per-correction effects,
from slam_replay.py --config '{"dump": "DUMP.json"}' output; list the fastest-growing windows and worst corrections."""
import json, os, sys, collections
import numpy as np
for b in sys.argv[1:]:
    d = json.load(open(os.path.expanduser(b))); b = os.path.basename(b).rsplit(".", 1)[0]
    eb = np.array([r["err_before"] for r in d]); ea = np.array([r["err_after"] for r in d])
    t = np.array([r["t"] for r in d]); acc = np.array([r["accepted"] for r in d])
    corr = np.array([np.hypot(*r["corr"][:2]) for r in d])
    drift = np.r_[0, eb[1:] - ea[:-1]]                        # error change between frames: odometry
    fix = ea - eb                                              # error change by the frame's correction
    applied = corr > 1e-4
    help_ = fix[applied & (fix < 0)].sum(); harm = fix[applied & (fix > 0)].sum()
    print(f"== {b}: {len(d)} frames, final err {ea[-1]:.2f} m, mean {ea.mean():.2f}")
    print(f"   odometry drift between frames: +{drift[drift>0].sum():.1f} / {drift[drift<0].sum():.1f} m;  corrections: helped {help_:.1f} m, hurt +{harm:.1f} m")
    fa = applied & (fix > 0.02)
    print(f"   corrections that made error worse by >2 cm: {fa.sum()} ({100*fa.sum()/max(applied.sum(),1):.1f}% of applied), by reason:",
          dict(collections.Counter(d[i]["reason"] for i in np.where(fa)[0])))
    # fastest error growth: 20 s windows
    W = 20.0; ws = np.arange(0, t[-1], W)
    grow = []
    for w in ws:
        m = (t >= w) & (t < w + W)
        if m.sum() < 5: continue
        g = ea[m][-1] - ea[m][0]
        grow.append((g, w, m))
    grow.sort(key=lambda x: -x[0])
    for g, w, m in grow[:3]:
        idx = np.where(m)[0]
        rs = collections.Counter(d[i]["reason"] for i in idx)
        dr = drift[idx].sum(); fx = fix[idx][applied[idx]].sum()
        print(f"   window {w:4.0f}-{w+W:4.0f} s: err {ea[idx][0]:.2f} -> {ea[idx][-1]:.2f} (drift {dr:+.2f}, corrections {fx:+.2f}), hdg err {d[idx[0]]['hdg_after']:.0f}->{d[idx[-1]]['hdg_after']:.0f} deg, "
              f"accepted {acc[idx].mean()*100:.0f}%, hits {np.mean([d[i]['hits'] for i in idx]):.0f}, reasons {dict(rs.most_common(4))}")
    top = np.argsort(-np.where(applied, fix, -1))[:5]
    for i in top:
        r = d[i]
        print(f"     worst t={r['t']:6.1f} {r['reason']:12s} acc={r['accepted']} reloc={r['reloc']} corr={np.round(r['corr'],3)} err {r['err_before']:.2f}->{r['err_after']:.2f} hdg {r['hdg_before']:.0f}->{r['hdg_after']:.0f} ov={r['overlap']} std={r['std_xy']} hits={r['hits']}")
