r"""preflight.py -- check that everything the simulation needs is present and consistent (S8).

Usage:  .venv\Scripts\python.exe scripts\preflight.py [--root DIR] [--ros] [--strict] [--write-checksums]
Checks (exit status 1 if any ERROR):
  * every MJCF scene model/duck_v4*_scene.xml (+ the gc world) parses, its <include> files exist, every mesh file exists,
    and MuJoCo compiles it
  * generated V4 meshes + model/v4_meshes.json and the design/v4 print meshes (watertight)
  * policy ONNX files (policies/manifest.json lists them; the 4 used by motions/ROS must exist, load in onnxruntime, 61 -> 14)
  * detector ONNX (yolo/duck_yolo.onnx, loads in onnxruntime)
  * world metadata: model/world.json parses and every named obstacle exists in model/duck_v4_gc_world.xml; beacon present
  * --ros: installed package assets (ros2/duck_description urdf/meshes, ros2/duck_nav mjcf/models) exist and match model/
  * model/checksums.json (created by --write-checksums): sha256 of the policy/detector files, compared when present
Warnings (not errors): optional policies missing, checksums file absent, mass ledger still PROVISIONAL.
Root: --root, $DUCKLING_ROOT, else the repo containing this script.
"""
import argparse, hashlib, json, os, sys
from pathlib import Path
import xml.etree.ElementTree as ET

ap = argparse.ArgumentParser()
ap.add_argument("--root"); ap.add_argument("--ros", action="store_true")
ap.add_argument("--strict", action="store_true", help="warnings also fail")
ap.add_argument("--write-checksums", action="store_true")
a = ap.parse_args()
ROOT = Path(a.root or os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
errors, warns = [], []


def err(s): errors.append(s); print("ERROR  ", s)
def warn(s): warns.append(s); print("warning", s)
def ok(s): print("ok     ", s)
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def xml_files(path, seen=None):
    """the file plus everything it includes (relative to the including file)"""
    seen = seen if seen is not None else []
    seen.append(path)
    for inc in ET.parse(path).getroot().iter("include"):
        f = path.parent / inc.get("file")
        if not f.exists():
            err(f"{path.name}: include file {inc.get('file')} not found"); continue
        if f not in seen:
            xml_files(f, seen)
    return seen


# ---- XML, includes, meshes
import mujoco
model = ROOT / "model"
for scene in sorted(model.glob("duck_v4*_scene.xml")) + [model / "duck_v4_gc_world.xml"]:
    if not scene.exists():
        err(f"{scene.name} missing (run make_duck_v4.py / worldgen.py)"); continue
    files = xml_files(scene)
    n_mesh = 0
    for f in files:
        r = ET.parse(f).getroot()
        comp = r.find("compiler")
        meshdir = (comp.get("meshdir") if comp is not None else None) or ""
        for ms in r.iter("mesh"):
            if ms.get("file"):
                n_mesh += 1
                if not (f.parent / meshdir / ms.get("file")).exists():
                    err(f"{f.name}: mesh file {ms.get('file')} not found")
    try:
        mujoco.MjModel.from_xml_path(str(scene)); ok(f"{scene.name}: {len(files)} xml file(s), {n_mesh} mesh refs, compiles")
    except Exception as e:
        err(f"{scene.name}: MuJoCo compile failed: {str(e).splitlines()[0]}")

# ---- V4 meshes and design
vm = model / "v4_meshes.json"
if not vm.exists():
    err("model/v4_meshes.json missing (run scripts/prep_meshes.py)")
else:
    n_ref = 0
    for n, r in json.loads(vm.read_text()).items():
        if not (ROOT / "design" / "v4" / f"{n}.stl").exists():
            err(f"design/v4/{n}.stl missing")
        if r.get("role") == "reference":                     # placement source: bounds only, no simulation asset
            n_ref += 1; continue
        if not (model / "assets" / f"{n}.stl").exists():
            err(f"model/assets/{n}.stl missing")
        if not r.get("watertight", False):
            err(f"design/v4/{n}.stl is not watertight")
        if not r.get("asset_watertight", False):
            err(f"model/assets/{n}.stl is not watertight (or v4_meshes.json predates the check: re-run prep_meshes.py)")
    ok(f"model/v4_meshes.json + V4 asset meshes + design/v4 print meshes (watertight); {n_ref} placement reference(s)")
led = model / "mass_ledger.json"
if led.exists() and "PROVISIONAL" in json.loads(led.read_text()).get("status", ""):
    warn("mass ledger is PROVISIONAL (30% infill proxy, not slicer/scale values)")

# ---- policies and detector
import onnxruntime as ort
sums = {}
man = ROOT / "policies" / "manifest.json"
needed = ["velstand", "alpha_sitstand", "roulade", "alpha_ground_pick"]
listed = [p["file"] for p in json.loads(man.read_text())["policies"]] if man.exists() else []
if not man.exists():
    err("policies/manifest.json missing")
for f in listed:
    p = ROOT / "policies" / f
    if not p.exists():
        (err if f[:-5] in needed else warn)(f"policy {f} missing (download per README)"); continue
    sums[f"policies/{f}"] = sha(p)
    try:
        s = ort.InferenceSession(str(p), providers=["CPUExecutionProvider"])
        shp = (s.get_inputs()[0].shape, s.get_outputs()[0].shape)
        if f[:-5] in needed and not (shp[0][-1] == 61 and shp[1][-1] == 14):
            err(f"policy {f}: expected 61 -> 14, got {shp}")
    except Exception as e:
        err(f"policy {f}: cannot load: {e}")
for n in needed:
    if f"{n}.onnx" not in listed:
        err(f"policy {n}.onnx not in manifest")
ok(f"policies: {len([k for k in sums if k.startswith('policies')])} present of {len(listed)} listed")
det = ROOT / "yolo" / "duck_yolo.onnx"
if det.exists():
    sums["yolo/duck_yolo.onnx"] = sha(det)
    try:
        s = ort.InferenceSession(str(det), providers=["CPUExecutionProvider"]); ok(f"detector ONNX loads, input {s.get_inputs()[0].shape}")
    except Exception as e:
        err(f"detector ONNX cannot load: {e}")
else:
    err("yolo/duck_yolo.onnx missing (scripts/yolo_train.py)")
ck = model / "checksums.json"
if a.write_checksums:
    ck.write_text(json.dumps(sums, indent=1)); ok(f"wrote {ck.name}")
elif ck.exists():
    for k, v in json.loads(ck.read_text()).items():
        if sums.get(k) != v:
            err(f"checksum mismatch: {k}")
    ok("checksums compared")
else:
    warn("model/checksums.json absent (run with --write-checksums once the files are the ones you trust)")

# ---- world metadata
wj, ww = model / "world.json", model / "duck_v4_gc_world.xml"
if not wj.exists():
    err("model/world.json missing (scripts/worldgen.py)")
else:
    w = json.loads(wj.read_text())
    names = {g.get("name") for g in ET.parse(ww).getroot().iter("geom")} if ww.exists() else set()
    miss = [i["name"] for i in w["items"] if i["name"] not in names]
    if miss:
        err(f"world.json items not in {ww.name}: {miss}")
    if "beacon" not in names or "beacon" not in w:
        err("beacon missing from world")
    else:
        ok(f"world.json: {len(w['items'])} items all present in {ww.name}, beacon at {w['beacon']}")

# ---- installed ROS package assets
if a.ros:
    dd, dn = ROOT / "ros2" / "duck_description", ROOT / "ros2" / "duck_nav"
    urdf = dd / "urdf" / "duck_v4.urdf"
    if not urdf.exists():
        err("ros2/duck_description/urdf/duck_v4.urdf missing")
    else:
        for m in ET.parse(urdf).getroot().iter("mesh"):
            f = dd / "meshes" / m.get("filename").split("/")[-1]
            if not f.exists():
                err(f"URDF mesh {f.name} missing from duck_description/meshes")
        ok("URDF meshes present")
    for f in ("duck_v4_gc.xml", "duck_v4_gc_world.xml", "world.json"):
        for pkg in (dn, dd):
            t = pkg / "mjcf" / f
            if not t.exists():
                err(f"{t.relative_to(ROOT)} missing (scripts/sync_ros2_assets.py)")
            elif sha(t) != sha(model / f):
                err(f"{t.relative_to(ROOT)} differs from model/{f} (re-run scripts/sync_ros2_assets.py)")
    for f in needed + ["duck_yolo"]:
        if not (dn / "models" / f"{f}.onnx").exists():
            err(f"ros2/duck_nav/models/{f}.onnx missing")
    for st in (model / "assets").glob("*_v4.stl"):
        for pkg in (dn, dd):
            t = pkg / "mjcf" / "assets" / st.name
            if not t.exists() or sha(t) != sha(st):
                err(f"{t.relative_to(ROOT)} missing or stale")

print(f"\npreflight: {len(errors)} error(s), {len(warns)} warning(s)")
sys.exit(1 if errors or (a.strict and warns) else 0)
