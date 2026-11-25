r"""worldgen.py -- build MuJoCo worlds (walls, boxes, floor clutter, beacon) around the V3 duck.

Usage:  .venv\Scripts\python.exe scripts\worldgen.py      -> writes model\duck_v4_gc_world.xml (the ROS 2 demo world)
        ... scripts\worldgen.py --layout b                -> model\duck_v4_gc_world_b.xml + world_b.json (second layout)
Also imported by yolo_dataset.py, which asks for random worlds.
"""
import random
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "model" / "duck_v4_gc_scene.xml"
GEOM = '<geom name="{n}" type="box" size="{sx:.3f} {sy:.3f} {sz:.3f}" pos="{x:.3f} {y:.3f} {sz:.3f}" euler="0 0 {yaw:.3f}" rgba="{c}"/>\n'


def _wrap(body):
    txt = SCENE.read_text()
    txt = txt.replace("</worldbody>", body + "</worldbody>", 1)
    return txt.replace("</asset>", '<material name="beacon_mat" rgba="1 0 1 1" emission="0.8"/>\n</asset>', 1)


def beacon(x, y):
    return f'<geom name="beacon" type="cylinder" size="0.04 0.14" pos="{x:.3f} {y:.3f} 0.14" material="beacon_mat"/>\n'


def demo_world():
    """The ROS 2 demo: a 5 x 4 m room, three partitions, five boxes, two step-over planks, a beacon.
    Returns (xml text, list of (name, x, y, sx, sy, sz, yaw, rgba)) -- the list feeds the RViz markers."""
    items = [
        ("w_south", 1.9, -2.0, 2.6, 0.03, 0.15, 0, "0.75 0.75 0.7 1"), ("w_north", 1.9, 2.0, 2.6, 0.03, 0.15, 0, "0.75 0.75 0.7 1"),
        ("w_west", -0.7, 0.0, 0.03, 2.0, 0.15, 0, "0.7 0.75 0.75 1"), ("w_east", 4.5, 0.0, 0.03, 2.0, 0.15, 0, "0.7 0.75 0.75 1"),
        ("p1", 0.9, 0.7, 0.03, 1.3, 0.15, 0, "0.3 0.6 0.4 1"),         # from the north wall, gap in the south
        ("p2", 2.4, -0.7, 0.03, 1.3, 0.15, 0, "0.3 0.4 0.7 1"),        # from the south wall, gap in the north
        ("p3", 3.5, 0.3, 0.5, 0.03, 0.15, 0, "0.6 0.4 0.3 1"),         # short wall shielding the beacon
        ("box1", 0.1, -1.2, 0.10, 0.10, 0.06, 0.3, "0.9 0.6 0.1 1"), ("box2", 1.6, 1.2, 0.12, 0.08, 0.05, 0.0, "0.2 0.7 0.8 1"),
        ("box3", 1.7, -0.3, 0.07, 0.07, 0.07, 0.6, "0.8 0.2 0.2 1"), ("box4", 3.2, -1.2, 0.10, 0.12, 0.08, 0.2, "0.8 0.8 0.2 1"),
        ("box5", 3.0, 1.4, 0.09, 0.09, 0.04, 0.0, "0.3 0.8 0.3 1"),
        ("plank1", 1.65, -1.25, 0.02, 0.30, 0.004, 0, "0.55 0.35 0.2 1"),   # 8 mm high: the duck steps over these
        ("plank2", 2.9, 0.9, 0.02, 0.30, 0.004, 1.2, "0.55 0.35 0.2 1"),
    ]
    body = "".join(GEOM.format(n=n, x=x, y=y, sx=sx, sy=sy, sz=sz, yaw=yaw, c=c) for n, x, y, sx, sy, sz, yaw, c in items)
    body += beacon(4.0, 1.3)
    return _wrap(body), items, (4.0, 1.3)


def layout_b():
    """Second test layout in the same 5 x 4 m room (same `room` interior): one long featureless partition parallel to
    the north wall (a 3 m corridor where a wall constrains only one direction), a short wall from the south wall
    shielding the beacon in the south-east corner, five boxes. The duck starts at the origin facing +x as in the demo.
    Launch with mjcf:=<share>/mjcf/duck_v4_gc_world_b.xml world_json:=<share>/mjcf/world_b.json beacon_x:=3.95
    beacon_y:=-1.55."""
    items = [
        ("w_south", 1.9, -2.0, 2.6, 0.03, 0.15, 0, "0.75 0.75 0.7 1"), ("w_north", 1.9, 2.0, 2.6, 0.03, 0.15, 0, "0.75 0.75 0.7 1"),
        ("w_west", -0.7, 0.0, 0.03, 2.0, 0.15, 0, "0.7 0.75 0.75 1"), ("w_east", 4.5, 0.0, 0.03, 2.0, 0.15, 0, "0.7 0.75 0.75 1"),
        ("q1", 1.5, 0.65, 1.5, 0.03, 0.15, 0, "0.3 0.6 0.4 1"),         # long partition, x 0..3, gaps at both ends
        ("q2", 3.3, -1.45, 0.03, 0.55, 0.15, 0, "0.3 0.4 0.7 1"),       # from the south wall, shields the beacon
        ("box1", 1.0, -1.0, 0.10, 0.10, 0.06, 0.3, "0.9 0.6 0.1 1"), ("box2", 2.2, -1.5, 0.12, 0.08, 0.05, 0.0, "0.2 0.7 0.8 1"),
        ("box3", 2.0, 1.35, 0.07, 0.07, 0.07, 0.6, "0.8 0.2 0.2 1"), ("box4", 0.2, 1.4, 0.10, 0.12, 0.08, 0.2, "0.8 0.8 0.2 1"),
        ("box5", 4.0, 0.9, 0.09, 0.09, 0.04, 0.0, "0.3 0.8 0.3 1"),
    ]
    body = "".join(GEOM.format(n=n, x=x, y=y, sx=sx, sy=sy, sz=sz, yaw=yaw, c=c) for n, x, y, sx, sy, sz, yaw, c in items)
    body += beacon(3.95, -1.55)
    return _wrap(body), items, (3.95, -1.55)


def random_world(rng: random.Random):
    """Random room for YOLO training images: random wall colours, 4-7 boxes of random size/colour
    (including pink and purple distractors), planks, one beacon. Returns xml text."""
    def col(lo=0.15, hi=0.95):
        while True:
            r, g, b = (rng.uniform(lo, hi) for _ in range(3))
            if not (r > 0.75 and b > 0.75 and g < 0.45):                # never beacon-magenta
                return f"{r:.2f} {g:.2f} {b:.2f} 1"
    hx, hy = rng.uniform(1.2, 2.5), rng.uniform(1.0, 2.0)
    body = ""
    for n, x, y, sx, sy in (("ws", 0, -hy, hx, 0.03), ("wn", 0, hy, hx, 0.03), ("ww", -hx, 0, 0.03, hy), ("we", hx, 0, 0.03, hy)):
        body += GEOM.format(n=n, x=x, y=y, sx=sx, sy=sy, sz=rng.uniform(0.1, 0.25), yaw=0, c=col(0.3, 0.9))
    for i in range(rng.randint(0, 2)):
        body += GEOM.format(n=f"part{i}", x=rng.uniform(-hx, hx) * 0.6, y=rng.uniform(-hy, hy) * 0.6, sx=0.03,
                            sy=rng.uniform(0.3, 0.9), sz=0.15, yaw=rng.uniform(0, 3.14), c=col())
    for i in range(rng.randint(4, 7)):
        c = col() if rng.random() > 0.3 else rng.choice(["0.95 0.45 0.75 1", "0.55 0.2 0.8 1", "0.9 0.2 0.5 1"])
        body += GEOM.format(n=f"box{i}", x=rng.uniform(-hx, hx) * 0.85, y=rng.uniform(-hy, hy) * 0.85,
                            sx=rng.uniform(0.04, 0.14), sy=rng.uniform(0.04, 0.14), sz=rng.uniform(0.025, 0.10),
                            yaw=rng.uniform(0, 3.14), c=c)
    for i in range(rng.randint(0, 2)):
        body += GEOM.format(n=f"plank{i}", x=rng.uniform(-hx, hx) * 0.7, y=rng.uniform(-hy, hy) * 0.7, sx=0.02,
                            sy=rng.uniform(0.15, 0.4), sz=0.004, yaw=rng.uniform(0, 3.14), c=col())
    if rng.random() < 0.85:
        body += beacon(rng.uniform(-hx, hx) * 0.85, rng.uniform(-hy, hy) * 0.85)
    return _wrap(body), (hx, hy)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", choices=("demo", "b"), default="demo")
    b = ap.parse_args().layout == "b"
    xml, items, goal = layout_b() if b else demo_world()
    out = ROOT / "model" / ("duck_v4_gc_world_b.xml" if b else "duck_v4_gc_world.xml")
    out.write_text(xml)
    import json
    js = [dict(name=n, x=x, y=y, sx=sx, sy=sy, sz=sz, yaw=yaw, rgba=[float(v) for v in c.split()]) for n, x, y, sx, sy, sz, yaw, c in items]
    js.append(dict(name="beacon", x=goal[0], y=goal[1], sx=0.04, sy=0.04, sz=0.14, yaw=0, rgba=[1, 0, 1, 1]))
    (ROOT / "model" / ("world_b.json" if b else "world.json")).write_text(json.dumps(dict(items=js, beacon=goal), indent=1))
    print("wrote", out, f"({len(items)} obstacles, beacon at {goal})")
