"""build_design.py -- Duckling V3 print meshes (upper head, lower head with bill, rear shield).

Usage:  python design/build_design.py [path to original_parts]
Original (upstream) meshes are not versioned. They are looked up in: argv[1], $DUCKLING_ORIGINAL_PARTS,
design/original_parts. Needs trimesh + numpy; the rear shield additionally needs manifold3d (boolean engine).
If manifold3d is missing the shield is NOT rebuilt and the versioned design/modified/ shield STL is kept.
"""
from pathlib import Path
import os, sys, json
import trimesh as t, numpy as np
ROOT = Path(__file__).resolve().parent
_cands = [Path(sys.argv[1])] if len(sys.argv) > 1 else []
if os.environ.get('DUCKLING_ORIGINAL_PARTS'):
    _cands.append(Path(os.environ['DUCKLING_ORIGINAL_PARTS']))
_cands.append(ROOT / 'original_parts')
SRC = next((c for c in _cands if (c / 'duck_head_lower.stl').exists()), None)
if SRC is None:
    raise SystemExit('original_parts not found; tried: ' + ', '.join(map(str, _cands)))
OUT = ROOT / 'modified'
OUT.mkdir(exist_ok=True)


def box(size, pos):
    m = t.creation.box(size); m.apply_translation(pos); return m


def smooth(a):
    a = np.clip(a, 0, 1); return a * a * (3 - 2 * a)


# V3: smoothly lift and broaden original dome, no feather bumps.
# Seam (z <= 6), front sensor surround (y <= -35), and lower mounting region fixed.
upper = t.load(SRC / 'duck_head_upper.stl'); v = upper.vertices.copy(); x, y, z = v.T.copy()
roof = smooth((z - 6) / 26) * smooth((y + 35) / 18)
v[:, 2] = z + 18 * roof
v[:, 0] = -2 + (x + 2) * (1 + 0.16 * smooth((z - 8) / 12) * smooth((y + 35) / 18))
upper.vertices = v; upper.fix_normals(); upper.export(OUT / 'duck_head_upper_v3.stl')

# A2 (mouth fix, 17 Nov 2026): the earlier thin tongue-like lip (flattened ellipsoid radii [24,16,3.6] at
# [-2,-51.5,15.5], boolean-unioned) is removed. The bill is now formed by pushing the front wall of the
# ORIGINAL lower shell forward with a smooth, broad, rounded displacement field (no boolean, so the shell stays
# one closed manifold). Untouched: seam plane z=22.35, the rear/side mount region (y > -36) and the shell's
# front-top edge (z >= 21). The shell stays hollow (inner and outer wall move together). Single material.
BILL = dict(push=12.0, y0=-36.0, y1=-50.0, half_w=21.0, w_fall=13.0, z_lo=(2.0, 8.0), z_hi=(15.0, 21.0), droop=1.5)
lower = t.load(SRC / 'duck_head_lower.stl'); v = lower.vertices.copy(); x, y, z = v.T.copy()
wy = smooth((BILL['y0'] - y) / (BILL['y0'] - BILL['y1']))
wx = 1 - smooth((np.abs(x + 2) - BILL['half_w']) / BILL['w_fall'])
wz = smooth((z - BILL['z_lo'][0]) / (BILL['z_lo'][1] - BILL['z_lo'][0])) * \
     (1 - smooth((z - BILL['z_hi'][0]) / (BILL['z_hi'][1] - BILL['z_hi'][0])))
w = wy * wx * wz
v[:, 1] = y - BILL['push'] * w
v[:, 2] = z - BILL['droop'] * w * smooth((BILL['z_hi'][1] - z) / 6)   # slight downward lean: a bill, not a lip
lower.vertices = v; lower.fix_normals(); lower.export(OUT / 'duck_head_lower_v3.stl')

# U channel: solid broad rear face, two long side walls, open at BOTH y ends.
# CAD x=width, y=robot vertical after mounting, z=depth away from HAT.
SHIELD = OUT / 'hat_rear_shield_v3_open_vertical_ends.stl'
try:
    import manifold3d  # noqa: F401
    have_manifold = True
except ImportError:
    have_manifold = False
if have_manifold:
    def union(parts): return t.boolean.union(parts, engine='manifold')
    W, L, D, T = 53.34 + 1.6, 110 + 6.6, 16, 2.5
    ow, oh = W + 2 * T, D + T; r = 2.5
    parts = [box([ow - 2 * r, L, oh], [0, 0, oh / 2]), box([ow, L, oh - 2 * r], [0, 0, oh / 2])]
    for xx in [-ow / 2 + r, ow / 2 - r]:
        for zz in [r, oh - r]:
            c = t.creation.cylinder(radius=r, height=L, sections=48)
            c.apply_transform(t.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
            c.apply_translation([xx, 0, zz]); parts.append(c)
    shield = union(parts)
    shield = t.boolean.difference([shield, box([W, L + 4, D + 4], [0, 0, (D - 4) / 2])], engine='manifold')
    for xx in [-W / 2 - T / 2, W / 2 + T / 2]:
        for yy in [-30, 30]:
            shield = t.boolean.difference([shield, box([8, 5, 2], [xx, yy, 5])], engine='manifold')
    shield.export(SHIELD)
else:
    print('manifold3d not installed: keeping existing shield STL', SHIELD)

report = {}
for name in ['duck_head_upper_v3.stl', 'duck_head_lower_v3.stl', 'hat_rear_shield_v3_open_vertical_ends.stl']:
    m = t.load(OUT / name)
    report[name] = {'watertight': bool(m.is_watertight), 'positive_volume': bool(m.volume > 0),
                    'connected_components': len(m.split()), 'dimensions_mm': m.extents.round(3).tolist(),
                    'volume_mm3': round(float(m.volume), 2), 'surface_area_mm2': round(float(m.area), 1),
                    'bounds_min_mm': m.bounds[0].round(3).tolist(), 'bounds_max_mm': m.bounds[1].round(3).tolist()}
report['duck_head_lower_v3.stl']['bill_params'] = BILL
report['duck_head_lower_v3.stl']['note'] = ('A2 mouth fix: lip ellipsoid removed; bill = smooth forward displacement of the '
                                            'original lower front wall. Seam/mount region untouched.')
report['hat_rear_shield_v3_open_vertical_ends.stl']['rebuilt_this_run'] = have_manifold
(ROOT / 'mesh_checks.json').write_text(json.dumps(report, indent=2)); print(json.dumps(report, indent=2))
