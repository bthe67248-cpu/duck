r"""mjcf_to_urdf.py -- convert model/duck_v3.xml (MJCF) to a URDF for ROS 2 / RViz.

Usage:  .venv\Scripts\python.exe scripts\mjcf_to_urdf.py [mjcf file] [output package dir]
Writes ros2/duck_description/urdf/duck_v3.urdf and copies the STL meshes next to it.
Only what RViz and robot_state_publisher need: links, inertias, visual meshes, hinge joints.
Collision / self-collision geoms (S1 proxies) are not exported; they are not visuals.

Materials (A1): every MJCF <material> becomes a named top-level URDF <material> (so black_pla is ONE definition
that every printed link references). URDF has no specular/shininess; those stay MuJoCo-only.
Roots come from this file's location; override with $DUCKLING_ROOT.
"""
import math, os, shutil, sys
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
SRC = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "model" / "duck_v3.xml"
PKG = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "ros2" / "duck_description"
SKIP_CLASSES = {"collision", "self_collision_only"}


def rpy(quat):
    w, x, y, z = (float(v) for v in quat.split())
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return f"{r:.6f} {p:.6f} {yw:.6f}"


def origin(parent, el):
    ET.SubElement(parent, "origin", xyz=el.get("pos", "0 0 0"), rpy=rpy(el.get("quat", "1 0 0 0")))


def main():
    mj = ET.parse(SRC).getroot()
    asset = mj.find("asset")
    mesh_file = {Path(m.get("file")).stem if not m.get("name") else m.get("name"): m.get("file")
                 for m in asset.findall("mesh")}
    rgba = {m.get("name"): m.get("rgba", "0.8 0.8 0.8 1") for m in asset.findall("material")}
    (PKG / "urdf").mkdir(parents=True, exist_ok=True)
    (PKG / "meshes").mkdir(parents=True, exist_ok=True)

    robot = ET.Element("robot", name="duck_v3")
    mats = {}                                   # material name -> rgba, written once at the top
    used = set()

    def material(vis, name, color):
        if name not in mats:
            mats[name] = color
        ET.SubElement(vis, "material", name=name)

    def add_body(body, parent_name):
        name = body.get("name")
        link = ET.SubElement(robot, "link", name=name)
        inert = body.find("inertial")
        if inert is not None:
            i = ET.SubElement(link, "inertial")
            ET.SubElement(i, "origin", xyz=inert.get("pos", "0 0 0"), rpy="0 0 0")
            ET.SubElement(i, "mass", value=inert.get("mass"))
            if inert.get("fullinertia"):
                xx, yy, zz, xy, xz, yz = inert.get("fullinertia").split()
            else:
                xx, yy, zz = inert.get("diaginertia").split(); xy = xz = yz = "0"
            ET.SubElement(i, "inertia", ixx=xx, iyy=yy, izz=zz, ixy=xy, ixz=xz, iyz=yz)
        for k, g in enumerate(body.findall("geom")):
            if g.get("class") in SKIP_CLASSES or g.get("group") == "3":
                continue
            vis = ET.SubElement(link, "visual")
            origin(vis, g)
            geo = ET.SubElement(vis, "geometry")
            if g.get("type") == "mesh":
                f = mesh_file[g.get("mesh")]
                used.add(f)
                ET.SubElement(geo, "mesh", filename=f"package://duck_description/meshes/{f}")
                mname = g.get("material")
                material(vis, mname if mname in rgba else f"{name}_m{k}", g.get("rgba") or rgba.get(mname, "0.8 0.8 0.8 1"))
            else:                                             # primitives
                sx, sy, sz = (2 * float(v) for v in g.get("size").split())
                ET.SubElement(geo, "box", size=f"{sx} {sy} {sz}")
                material(vis, f"{name}_m{k}", g.get("rgba", "0.1 0.1 0.1 1"))
        if parent_name is not None:
            j = body.find("joint")
            jn = j.get("name") if j is not None else f"{parent_name}_to_{name}"
            joint = ET.SubElement(robot, "joint", name=jn, type="revolute" if j is not None else "fixed")
            ET.SubElement(joint, "parent", link=parent_name)
            ET.SubElement(joint, "child", link=name)
            origin(joint, body)
            if j is not None:
                ET.SubElement(joint, "axis", xyz=j.get("axis", "0 0 1"))
                lo, hi = j.get("range", "-3.14 3.14").split()
                ET.SubElement(joint, "limit", lower=lo, upper=hi, effort="0.96", velocity="6.0")
        for child in body.findall("body"):
            add_body(child, name)

    add_body(mj.find("worldbody").find("body"), None)
    for k, (mname, color) in enumerate(mats.items()):         # global definitions first, as URDF recommends
        mat = ET.Element("material", name=mname)
        ET.SubElement(mat, "color", rgba=color)
        robot.insert(k, mat)
    # keep the material definitions' full colour on each reference too (some RViz versions ignore name-only refs)
    for vis in robot.iter("visual"):
        ref = vis.find("material")
        ET.SubElement(ref, "color", rgba=mats[ref.get("name")])
    ET.indent(robot, "  ")
    ET.ElementTree(robot).write(PKG / "urdf" / "duck_v3.urdf", xml_declaration=True, encoding="utf-8")
    for f in used:
        shutil.copy2(ROOT / "model" / "assets" / f, PKG / "meshes" / f)
    for stale in sorted(set(f.name for f in (PKG / "meshes").glob("*.stl")) - used):   # meshes only the old model used
        (PKG / "meshes" / stale).unlink(); print("removed stale mesh", stale)
    n_rev = sum(1 for j in robot.findall("joint") if j.get("type") == "revolute")
    print(f"wrote {PKG / 'urdf' / 'duck_v3.urdf'}: {len(robot.findall('link'))} links, "
          f"{n_rev} revolute joints, {len(used)} meshes, {len(mats)} materials")


if __name__ == "__main__":
    main()
