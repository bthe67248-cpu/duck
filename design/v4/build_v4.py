"""Duckling V4 parts (Blender). Writes design/v4/*.stl in millimetres.

    E:lenderlender.exe --background --python design/v4/build_v4.py [-- body|head|head_upper|head_lower]

Parts (all OUR geometry; upstream meshes are not used or written). Dimensions: v4_params.json.
  body_shell_v4.stl     torso enclosure replacing the upstream left/right shells, the rear shield and the battery
                        holder (power_support). Rear board bay (bulkhead with UNO Q standoffs, lid bosses), battery
                        compartment over a connector well (panel XT60, cover bosses), A010 chest pocket with rear posts,
                        trunk mounts (rear flange to the trunk plate, side bosses to the hip-yaw XL330s), cable windows.
                        Torso frame: +x forward, +y left, +z up; origin = trunk_base.
  body_lid_v4.stl       removable lid of the board bay (4 x M2), U-slot for the USB-C cable.
  keel_cover_v4.stl     removable cover of the connector well (4 x M2) with the rails the LiPo stands on.
  pcb_stack_v4.stl, a010_v4.stl, battery_v4.stl, xt60_v4.stl   placeholders (boxes) of the bought parts.
  head_upper_v4.stl     from the source head_upper_v4_whole.stl with the M12 lens opening.
  head_lower_v4.stl, jaw_v4.stl  from the source head_lower_v4_whole.stl: clearance cuts around the neck mechanism
                        (head_clearance.py), then the jaw split with arc faces about the hinge and a pushrod lever.
  mouth_servo_v4.stl    XL330 placeholder in the dome (output axis parallel to the jaw hinge).
Head mesh frame: x = width, -y = front, z = up; the upper part keeps its own frame (z shifted by the seam height).
Still to do on the real parts: hinge pin and bosses for the jaw, the pushrod/horn hardware, screws joining the head
shells, mouth-servo bosses in the dome, the neck cable pass-through.
"""
import json, math, os
import bpy, bmesh
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
import sys
P = json.load(open(os.path.join(HERE, "v4_params.json")))
ONLY = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "all"


def clear():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def box(name, lo, hi, bevel=0.0, seg=3):
    me = bpy.data.meshes.new(name); bm = bmesh.new(); bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        for i in range(3):
            v.co[i] = lo[i] + (v.co[i] + 0.5) * (hi[i] - lo[i])
    bm.to_mesh(me); bm.free()
    ob = bpy.data.objects.new(name, me); bpy.context.collection.objects.link(ob)
    if bevel > 0:
        md = ob.modifiers.new("bev", "BEVEL"); md.width = bevel; md.segments = seg; md.limit_method = "NONE"
        bpy.context.view_layer.objects.active = ob; bpy.ops.object.modifier_apply(modifier=md.name)
    return ob


def boolean(target, tool, op, keep=False):
    if os.environ.get("V4_DEBUG"):
        print(f"bool {op} {target.name} ({len(target.data.polygons)}) <- {tool.name} ({len(tool.data.polygons)})", flush=True)
    md = target.modifiers.new("b", "BOOLEAN"); md.operation = op; md.object = tool; md.solver = "EXACT"
    bpy.context.view_layer.objects.active = target; bpy.ops.object.modifier_apply(modifier=md.name)
    if not keep:
        bpy.data.objects.remove(tool, do_unlink=True)
    return target


def drop_slivers(ob, min_vol=1.0):
    """Remove zero-volume islands that exact booleans leave where cut faces meet (they break watertightness)."""
    bm = bmesh.new(); bm.from_mesh(ob.data)
    # triangulate first: the STL writer triangulates n-gons anyway, and a degenerate n-gon can fold into a back-to-back
    # triangle pair that is only visible (and removable) as its own island once it is triangles
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-3)
    bmesh.ops.dissolve_degenerate(bm, edges=bm.edges, dist=1e-3)
    bmesh.ops.triangulate(bm, faces=bm.faces)              # dissolving can merge faces back into n-gons
    bm.faces.ensure_lookup_table()
    seen, kill = set(), []
    for f in bm.faces:
        if f.index in seen:
            continue
        island, stack = [], [f]
        seen.add(f.index)
        while stack:
            g = stack.pop(); island.append(g)
            for e in g.edges:
                if len(e.link_faces) != 2:          # non-manifold edges join a sliver to the part: do not walk them
                    continue
                for h in e.link_faces:
                    if h.index not in seen:
                        seen.add(h.index); stack.append(h)
        vol = abs(sum(g.calc_area() * g.normal.dot(g.calc_center_median()) for g in island) / 3.0)
        if vol < min_vol:
            kill += island
    if kill:
        bmesh.ops.delete(bm, geom=list(set(kill)), context="FACES")
    bm.to_mesh(ob.data); ob.data.update()
    bm.free()


def check_closed(ob, fname):
    """Every edge must join exactly two faces (closed, manifold): the masses and the simulation rely on it."""
    bm = bmesh.new(); bm.from_mesh(ob.data)
    bad_e = [e for e in bm.edges if len(e.link_faces) != 2]
    bad = len(bad_e)
    if bad and os.environ.get("V4_DEBUG"):
        pts = sorted({tuple(round(c, 1) for c in v.co) for e in bad_e for v in e.verts})
        print(f"{fname}: non-manifold edges near", pts[:: max(1, len(pts) // 25)], flush=True)
    bm.free()
    if bad:
        raise RuntimeError(f"{fname}: {bad} edge(s) not shared by exactly two faces; the part is not a closed solid")


def export(ob, fname):
    drop_slivers(ob)
    check_closed(ob, fname)
    bpy.ops.object.select_all(action="DESELECT"); ob.select_set(True); bpy.context.view_layer.objects.active = ob
    bpy.ops.wm.stl_export(filepath=os.path.join(HERE, fname), export_selected_objects=True, ascii_format=False)
    bm = bmesh.new(); bm.from_mesh(ob.data); vol = bm.calc_volume(signed=False); bm.free()
    print(f"V4PART {fname} volume {vol / 1000:.2f} cm3")


def union_boxes(name, specs):
    obs = [box(f"{name}{i}", s["lo"], s["hi"], s.get("bevel", 0)) for i, s in enumerate(specs)]
    base = obs[0]
    for o in obs[1:]:
        boolean(base, o, "UNION")
    base.name = name
    return base


def load(path, name):
    bpy.ops.wm.stl_import(filepath=path)
    ob = bpy.context.selected_objects[0]; ob.name = name; return ob


def mesh_object(name, verts, faces):
    me = bpy.data.meshes.new(name); me.from_pydata([tuple(v) for v in verts], [], faces); me.update()
    bm = bmesh.new(); bm.from_mesh(me); bmesh.ops.recalc_face_normals(bm, faces=bm.faces); bm.to_mesh(me); bm.free()
    ob = bpy.data.objects.new(name, me); bpy.context.collection.objects.link(ob)
    return ob


def prism_x(name, pts_yz, x0, x1):
    """Prism along x from a closed (y, z) polygon."""
    n = len(pts_yz)
    V = [(x0, y, z) for y, z in pts_yz] + [(x1, y, z) for y, z in pts_yz]
    F = [list(range(n))[::-1], list(range(n, 2 * n))] + [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    return mesh_object(name, V, F)


def lathe_z(name, profile, cx, cy, seg=72):
    """Solid of revolution about the vertical axis through (cx, cy) from [(z, r), ...] (z increasing)."""
    V, F = [], []
    for z, r in profile:
        V += [(cx + r * math.cos(2 * math.pi * k / seg), cy + r * math.sin(2 * math.pi * k / seg), z) for k in range(seg)]
    for i in range(len(profile) - 1):
        F += [[i * seg + k, i * seg + (k + 1) % seg, (i + 1) * seg + (k + 1) % seg, (i + 1) * seg + k] for k in range(seg)]
    F += [list(range(seg))[::-1], list(range((len(profile) - 1) * seg, len(profile) * seg))]
    return mesh_object(name, V, F)


def cylinder_y(name, cx, cz, r, y0, y1, seg=64):
    pts = [(cx + r * math.cos(2 * math.pi * k / seg), cz + r * math.sin(2 * math.pi * k / seg)) for k in range(seg)]
    V = [(x, y0, z) for x, z in pts] + [(x, y1, z) for x, z in pts]
    F = [list(range(seg)), list(range(seg, 2 * seg))[::-1]] + [[k, (k + 1) % seg, seg + (k + 1) % seg, seg + k] for k in range(seg)]
    return mesh_object(name, V, F)


def cylinder_x(name, cy, cz, r, x0, x1, seg=32):
    pts = [(cy + r * math.cos(2 * math.pi * k / seg), cz + r * math.sin(2 * math.pi * k / seg)) for k in range(seg)]
    return prism_x(name, pts, x0, x1)


def rotate_about_y(ob, x0, z0, ang):
    """Rotate an object's mesh by ang (rad) about the axis along y through (x0, z0)."""
    c, s = math.cos(ang), math.sin(ang)
    for v in ob.data.vertices:
        dx, dz = v.co.x - x0, v.co.z - z0
        v.co.x, v.co.z = x0 + c * dx + s * dz, z0 - s * dx + c * dz
    ob.data.update()


def subtract_swept(target, make, x0, z0, angles):
    for a in angles:
        tool = make(); rotate_about_y(tool, x0, z0, a); boolean(target, tool, "DIFFERENCE")

def cyl_axis(name, axis, c, r, span, seg=32):
    """Cylinder along axis x|y|z with centre c (the other two coordinates in xyz order) and span along the axis."""
    pts = [(c[0] + r * math.cos(2 * math.pi * k / seg), c[1] + r * math.sin(2 * math.pi * k / seg)) for k in range(seg)]
    i = "xyz".index(axis); others = [k for k in range(3) if k != i]
    V = []
    for a in span:
        for u, v in pts:
            p = [0.0, 0.0, 0.0]; p[i] = a; p[others[0]] = u; p[others[1]] = v; V.append(tuple(p))
    F = [list(range(seg)), list(range(seg, 2 * seg))] + [[k, (k + 1) % seg, seg + (k + 1) % seg, seg + k] for k in range(seg)]
    return mesh_object(name, V, F)


def rotate_about_x(ob, y0, z0, ang):
    """Rotate an object's mesh by ang (rad) about the axis along x through (y0, z0); +ang turns +y towards +z."""
    c, s = math.cos(ang), math.sin(ang)
    for v in ob.data.vertices:
        dy, dz = v.co.y - y0, v.co.z - z0
        v.co.y, v.co.z = y0 + c * dy - s * dz, z0 + s * dy + c * dz
    ob.data.update()


def hull_of(ob, name, inset=0.6):
    """Convex hull of an object, moved `inset` mm inwards, as a new closed object. Used to keep added features inside a
    part's outer skin: the inset makes them end inside the wall instead of on the skin (coincident faces break the
    boolean's manifold result)."""
    bm = bmesh.new(); bm.from_mesh(ob.data)
    res = bmesh.ops.convex_hull(bm, input=bm.verts)
    bmesh.ops.delete(bm, geom=[g for g in res["geom_interior"] + res["geom_unused"] if isinstance(g, bmesh.types.BMVert)],
                     context="VERTS")
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces); bm.normal_update()
    moves = [(v, v.normal.copy()) for v in bm.verts]
    for v, n in moves:
        v.co -= n * inset
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    h = bpy.data.objects.new(name, me); bpy.context.collection.objects.link(h)
    return h


def local_hull(ob, name, lo, hi, inset=0.6):
    """Inset convex hull of the part of `ob` inside the box lo..hi: follows a curved skin much more closely than the
    hull of the whole part (used to keep the hinge bosses and the pushrod lever inside the jaw's skin)."""
    piece = ob.copy(); piece.data = ob.data.copy(); bpy.context.collection.objects.link(piece)
    boolean(piece, box(name + "_win", lo, hi), "INTERSECT")
    h = hull_of(piece, name, inset)
    bpy.data.objects.remove(piece, do_unlink=True)
    return h


def hull_2d(pts):
    """Convex hull (counter-clockwise) of 2-D points, monotone chain."""
    pts = sorted(set(pts))
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo_, hi_ = [], []
    for p in pts:
        while len(lo_) >= 2 and cross(lo_[-2], lo_[-1], p) <= 0: lo_.pop()
        lo_.append(p)
    for p in reversed(pts):
        while len(hi_) >= 2 and cross(hi_[-2], hi_[-1], p) <= 0: hi_.pop()
        hi_.append(p)
    return lo_[:-1] + hi_[:-1]


def ear_profile(Hg, k, grow=0.0):
    """(y, z) outline of hinge ear k (0 = left, 1 = right): convex hull of the ring about the hinge axis and the web's
    underside; `grow` enlarges it (clearance)."""
    J = P["head"]["jaw"]; hy, hz = J["hinge_y"], J["hinge_z"]
    r = Hg["ring_r"] + grow
    w = Hg["web"][k]
    pts = [(hy + r * math.cos(2 * math.pi * i / 48), hz + r * math.sin(2 * math.pi * i / 48)) for i in range(48)]
    # the plain ear ends 0.3 mm inside the web (no coincident faces in the union); the clearance outline covers it all
    d = 0.3 if grow == 0 else -grow
    pts += [(w["lo"][1] + d, w["lo"][2] + d), (w["hi"][1] - d, w["lo"][2] + d),
            (w["lo"][1] + d, w["hi"][2] - max(d, 0)), (w["hi"][1] - d, w["hi"][2] - max(d, 0))]
    return hull_2d(pts)


def ear_tool(name, Hg, k):
    x0, x1 = Hg["ear_x"][k]
    return prism_x(name, ear_profile(Hg, k), x0, x1)


def ear_sweep_tool(name, Hg, k):
    """Clearance slot the jaw needs for ear k: the ear grown by `gap`, turned (in the jaw's frame) over the jaw's whole
    travel, as one prism (2-D convex hull of the turned outlines: one boolean instead of one per angle, and a little
    conservative). The jaw sees the ear turn the other way as it opens."""
    J = P["head"]["jaw"]; hy, hz = J["hinge_y"], J["hinge_z"]
    prof, g = ear_profile(Hg, k, Hg["gap"]), Hg["gap"]
    a0, a1 = Hg["sweep_deg"]
    pts = []
    for i in range(25):
        a = -math.radians(a0 + (a1 - a0) * i / 24); c, s_ = math.cos(a), math.sin(a)
        pts += [(round(hy + c * (y - hy) - s_ * (z - hz), 4), round(hz + s_ * (y - hy) + c * (z - hz), 4)) for y, z in prof]
    x0, x1 = Hg["ear_x"][k]
    return prism_x(name, hull_2d(pts), x0 - g, x1 + g)


def prim(spec):
    """A box (lo, hi, bevel) or a cylinder ({"cyl": {axis, c, r, span}}) from v4_params.json."""
    if "cyl" in spec:
        c = spec["cyl"]; return cyl_axis(spec["name"], c["axis"], c["c"], c["r"], c["span"])
    return box(spec["name"], spec["lo"], spec["hi"], spec.get("bevel", 0))


def build_from(name, parts, cuts=()):
    base = prim(parts[0]); base.name = name
    for s in parts[1:]:
        boolean(base, prim(s), "UNION")
    for s in cuts:
        boolean(base, prim(s), "DIFFERENCE")
    return base


# ------------------------------------------------------------------ body
if ONLY in ("all", "body"):
    clear()
    B = P["body"]
    outer = union_boxes("body_shell_v4", B["outer"])
    inner = union_boxes("inner", B["inner"])
    boolean(outer, inner, "DIFFERENCE")
    for s in B["features"]:
        boolean(outer, prim(s), "UNION")
    for s in B["cuts"]:
        boolean(outer, prim(s), "DIFFERENCE")
    export(outer, "body_shell_v4.stl")
    clear(); export(build_from("body_lid_v4", B["lid"]["parts"], B["lid"]["cuts"]), "body_lid_v4.stl")
    clear(); export(build_from("keel_cover_v4", B["cover"]["parts"], B["cover"]["cuts"]), "keel_cover_v4.stl")
    for key, fname in (("pcb_stack", "pcb_stack_v4.stl"), ("a010", "a010_v4.stl"), ("battery", "battery_v4.stl"), ("xt60", "xt60_v4.stl")):
        clear(); export(box(key, P[key]["lo"], P[key]["hi"], 0.5), fname)

# ------------------------------------------------------------------ head
# Sources (committed): head_upper_v4_whole.stl and head_lower_v4_whole.stl. They were made once from the V3 head shells
# (now deleted) by widening x about x = cx by `widen` and pushing the rear (y > rear_y0) back by up to `rear_push`
# mm with a smoothstep (parameters kept in v4_params.json "head" as the record). Rebuilt here: the lens opening, the
# clearance cuts around the neck mechanism (head_clearance.py measures them), the jaw split and the mouth servo.
H = P["head"]
if ONLY in ("all", "head", "head_upper"):
    clear()
    upper = load(os.path.join(HERE, "head_upper_v4_whole.stl"), "head_upper_v4")
    # the upper part's STL keeps its own frame (z shifted by the seam height): cutters are given in the lower-head frame
    zs = H["upper_zshift"]
    L = H["lens_opening"]
    (lx, lz), lr = L["centre_xz"], L["radius"]
    boolean(upper, cylinder_y("lens", lx, lz - zs, lr, *L["y"]), "DIFFERENCE")
    boolean(upper, box("lensnotch", [lx - lr, L["y"][0], L["notch_to_z"] - zs], [lx + lr, L["y"][1], lz - zs]), "DIFFERENCE")
    # mouth-servo cradle hanging from the dome, trimmed to the inside of the shell's convex hull
    MC = H["mouth_cradle"]
    sh = lambda s: ([s["lo"][0], s["lo"][1], s["lo"][2] - zs], [s["hi"][0], s["hi"][1], s["hi"][2] - zs])

    def dome_hits(x, y):
        """(inner, outer) z of the dome above (x, y), upper-mesh frame, from two upward rays."""
        o = Vector((x, y, 50.0 - zs)); out = []
        for _ in range(2):
            hit, loc, _n, _i = upper.ray_cast(o, Vector((0, 0, 1)))
            if not hit:
                raise RuntimeError(f"cradle: no dome above ({x}, {y})")
            out.append(loc.z); o = loc + Vector((0, 0, 1e-3))
        return out

    plates = []
    for k, pl in enumerate(MC["plates"]):
        (x0, y0, z0), (x1, y1, _z1) = sh(pl)
        tops = []
        for y in (y0, y1):
            hits = [dome_hits(x0 + (x1 - x0) * i / 12, y) for i in range(13)]
            top = min(h[1] for h in hits) - MC["top_inset"]
            if top < max(h[0] for h in hits) + MC["min_anchor"]:
                raise RuntimeError(f"cradle plate {k}: the dome wall is too thin/steep here to anchor the plate")
            tops.append(top)
        plates.append(prism_x(f"plate{k}", [(y0, z0), (y1, z0), (y1, tops[1]), (y0, tops[0])], x0, x1))
    cradle = plates[0]
    for pl in plates[1:]:
        boolean(cradle, pl, "UNION")
    for s in MC["lips"] + [MC["stop"]]:
        boolean(cradle, box("cr", *sh(s)), "UNION")
    for s in [MC["cable_window"]] + MC["tie_slots"]:
        boolean(cradle, box("crcut", *sh(s)), "DIFFERENCE")
    boolean(upper, cradle, "UNION")
    # head-shell screws: fill each upper boss bore and re-drill a round pilot (the V4 widening made the rear ones oval)
    FS = H["fasteners"]
    for s in FS["screws"]:
        (sx, sy), top = s["xy"], s["upper_top_z"] - zs
        boolean(upper, cyl_axis("fill", "z", (sx, sy), FS["fill_r"], (s["upper_fill_from_z"] - zs, top + 1.0)), "UNION")
        boolean(upper, cyl_axis("pilot", "z", (sx, sy), FS["pilot_d"] / 2, (-1.0, top)), "DIFFERENCE")
    export(upper, "head_upper_v4.stl")

if ONLY in ("all", "head", "head_lower"):
    clear()
    lo = load(os.path.join(HERE, "head_lower_v4_whole.stl"), "head_lower_v4")
    CL = H["clearance"]
    rx, rz = CL["roll_xz"]
    rolls = [CL["roll_range_rad"][0] + (CL["roll_range_rad"][1] - CL["roll_range_rad"][0]) * k / 8 for k in range(9)]
    prof = CL["revolve_profile"]
    prof = [(prof[0][0] - 15.0, prof[0][1])] + [tuple(p) for p in prof] + [(prof[-1][0] + 0.01, 0.01)]
    subtract_swept(lo, lambda: lathe_z("yawsweep", prof, *CL["yaw_xy"]), rx, rz, rolls)
    for b in CL["boxes"]:
        if b["roll_sweep"]:
            subtract_swept(lo, lambda: box("rollsweep", b["lo"], b["hi"]), rx, rz, rolls)
        else:
            boolean(lo, box("fixed", b["lo"], b["hi"]), "DIFFERENCE")

    # jaw split: lower beak in front of the hinge plane + a quarter-disc heel about the hinge axis; the lower shell loses
    # that region grown by the gap and the heel's sweep above the hinge (mating faces are arcs about the hinge)
    J = H["jaw"]
    hy, hz, R, g = J["hinge_y"], J["hinge_z"], J["heel_r"], J["gap"]
    arc = lambda r, a0, a1, n=24: [(hy + r * math.cos(math.radians(a0 + (a1 - a0) * k / n)), hz + r * math.sin(math.radians(a0 + (a1 - a0) * k / n))) for k in range(n + 1)]
    jaw_pts = [(J["front_y"], hz - R), (hy, hz - R)] + arc(R, -90, 0)[1:] + [(hy, hz), (hy, J["top_z"]), (J["front_y"], J["top_z"])]
    cut_pts = [(J["front_y"] - 1, hz - R - g - 6), (hy, hz - R - g - 6)] + arc(R + g, -90, J["sweep_deg"])[0:] + \
              [(hy + g, hz + (R + g) * math.sin(math.radians(J["sweep_deg"]))), (hy + g, J["top_z"] + g), (J["front_y"] - 1, J["top_z"] + g)]
    jaw = lo.copy(); jaw.data = lo.data.copy(); bpy.context.collection.objects.link(jaw); jaw.name = "jaw_v4"
    boolean(jaw, prism_x("jawcut", jaw_pts, *J["x"]), "INTERSECT")
    # lever plate on the heel for the mouth pushrod (pin hole on the plate), in the pushrod's plane
    lv = J["lever"]
    # the plate's lower edge follows the bill's inner bottom surface (0.3 mm into the wall, never through it): the
    # bottom is concave behind the hinge, so the plain quarter disc used to bulge out under the bill
    xm = 0.5 * (lv["x"][0] + lv["x"][1])
    prof = [(hy, hz)]
    for pt_y, pt_z in arc(R - 0.5, -90, 0, 40):
        floor = None
        for xs_ in (lv["x"][0] + 0.2, xm, lv["x"][1] - 0.2):
            hit, loc, _n, _i = jaw.ray_cast(Vector((xs_, pt_y, hz)), Vector((0, 0, -1)))
            if hit:
                floor = loc.z - 0.3 if floor is None else max(floor, loc.z - 0.3)
        prof.append((pt_y, min(hz - 0.5, max(pt_z, floor)) if floor is not None else hz - 0.5))
    boolean(jaw, prism_x("lever", prof, *lv["x"]), "UNION")
    boolean(jaw, cylinder_x("leverpin", lv["pin_yz"][0], lv["pin_yz"][1], lv["pin_d"] / 2, lv["x"][0] - 1, lv["x"][1] + 1), "DIFFERENCE")
    boolean(lo, prism_x("shellcut", cut_pts, J["x"][0] - 1, J["x"][1] + 1), "DIFFERENCE")

    # jaw hinge: pin bosses on the jaw's side walls (kept inside the jaw's outer skin), ears + seam webs on the shell
    Hg, FS = H["hinge"], H["fasteners"]
    for k in range(2):
        x0, x1 = Hg["jaw_boss_x"][k]
        r_ = Hg["jaw_boss_r"]
        jh = local_hull(jaw, "jaw_hull", [x0 - 3, hy - r_ - 3, hz - r_ - 3], [x1 + 3, hy + r_ + 3, hz + r_ + 3])
        b = cyl_axis("jawboss", "x", (hy, hz), r_, (x0, x1))
        boolean(b, jh, "INTERSECT")
        boolean(jaw, b, "UNION")
    for k in range(2):
        boolean(lo, ear_tool("ear", Hg, k), "UNION")
        boolean(lo, box("web", Hg["web"][k]["lo"], Hg["web"][k]["hi"]), "UNION")
        boolean(jaw, ear_sweep_tool("earsweep", Hg, k), "DIFFERENCE")
    for x0, x1 in Hg["jaw_boss_x"]:
        boolean(jaw, cyl_axis("pinclear", "x", (hy, hz), Hg["pin_clear_d"] / 2, (x0 - 2, x1 + 1)), "DIFFERENCE")
    for x0, x1 in Hg["ear_x"]:
        boolean(lo, cyl_axis("pinpilot", "x", (hy, hz), Hg["pin_pilot_d"] / 2, (x0 - 1, x1 + 1)), "DIFFERENCE")
    # head-shell screws: clean clearance holes through the lower bosses (and the webs that wrap the front ones)
    for s in FS["screws"]:
        if "lower_fill_z" in s:
            boolean(lo, cyl_axis("fill", "z", tuple(s["xy"]), FS["lower_fill_r"], tuple(s["lower_fill_z"])), "UNION")
        boolean(lo, cyl_axis("clear", "z", tuple(s["xy"]), FS["clear_d"] / 2, (-1.0, FS["seam_z"] + 1.0)), "DIFFERENCE")
        if "counterbore_d" in s:                          # screw head recessed into the web, clear of the jaw heel
            boolean(lo, cyl_axis("cbore", "z", tuple(s["xy"]), s["counterbore_d"] / 2, (-1.0, s["counterbore_top_z"])), "DIFFERENCE")
    export(lo, "head_lower_v4.stl"); export(jaw, "jaw_v4.stl")
if ONLY in ("all", "head"):
    clear(); export(box("mouth_servo", H["mouth_servo"]["lo"], H["mouth_servo"]["hi"], 1.0), "mouth_servo_v4.stl")

print("V4_DONE")
