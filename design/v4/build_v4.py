"""Duckling V4 concept parts (Blender). Writes design/v4/*.stl in millimetres.

    E:\blender\blender.exe --background --python design/v4/build_v4.py

Parts (all OUR geometry; upstream meshes are not used or written):
  body_shell_v4.stl     one-piece torso enclosure replacing the upstream left/right shells and the old rear shield.
                        Encloses the battery, the resized 70 x 53.34 mm UNO Q + HAT stack (vertical, behind the
                        battery) and the Sipeed A010 in a chest pocket with a front window. Open bottom over the hips,
                        notch on top for the neck. Torso frame: +x forward, +y left, +z up; origin = trunk_base.
  pcb_stack_v4.stl      placeholder for UNO Q + HAT (70 tall x 53.34 across x 16 deep).
  a010_v4.stl           placeholder for the A010 (40.7 x 23.25 x 10.5).
  head_lower_v4.stl, jaw_v4.stl  split from the committed source head_lower_v4_whole.stl: the lower half of the
                        beak becomes the jaw, hinged at the bill root (axis along x).
  mouth_servo_v4.stl    XL330-size placeholder under the forehead.
  (head_upper_v4.stl and head_lower_v4_whole.stl are sources: head 10 % wider and 16 mm longer at the back than the
  old V3 head, so the rear servo and speaker sit inside.) Head mesh frame: x = width, -y = front, z = up; the upper
  part keeps its own frame (z shifted by the 22.35 mm seam height).
CONCEPT GEOMETRY: no screw bosses, ribs, print splits or wire routing yet; positions come from the simulation model
(trunk frame at the standing pose). Check every clearance on the real parts.
"""
import json, math, os
import bpy, bmesh
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(os.path.join(HERE, "v4_params.json")))


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
    md = target.modifiers.new("b", "BOOLEAN"); md.operation = op; md.object = tool; md.solver = "EXACT"
    bpy.context.view_layer.objects.active = target; bpy.ops.object.modifier_apply(modifier=md.name)
    if not keep:
        bpy.data.objects.remove(tool, do_unlink=True)
    return target


def export(ob, fname):
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


# ------------------------------------------------------------------ body shell
clear()
B = P["body"]
outer = union_boxes("body_shell_v4", B["outer"])
inner = union_boxes("inner", B["inner"])
boolean(outer, inner, "DIFFERENCE")
for c in B["cuts"]:
    boolean(outer, box("cut", c["lo"], c["hi"], c.get("bevel", 0)), "DIFFERENCE")
export(outer, "body_shell_v4.stl")
for key, fname in (("pcb_stack", "pcb_stack_v4.stl"), ("a010", "a010_v4.stl"), ("battery", "battery_v4.stl"), ("xt60", "xt60_v4.stl")):
    clear(); export(box(key, P[key]["lo"], P[key]["hi"], 0.5), fname)

# ------------------------------------------------------------------ head
# Sources (committed): head_upper_v4.stl and head_lower_v4_whole.stl. They were made once from the V3 head shells
# (now deleted) by widening x about x = cx by `widen` and pushing the rear (y > rear_y0) back by up to `rear_push`
# mm with a smoothstep (parameters kept in v4_params.json "head" as the record). Only the jaw split is rebuilt here.
H = P["head"]


def load(path, name):
    bpy.ops.wm.stl_import(filepath=path)
    ob = bpy.context.selected_objects[0]; ob.name = name; return ob


clear()
lo = load(os.path.join(HERE, "head_lower_v4_whole.stl"), "head_lower_v4")
# jaw = lower front (bill lower half + chin) in front of the hinge; a small gap is removed between the two parts
J = H["jaw"]
jaw_tool = box("jawcut", J["lo"], J["hi"])
jaw = lo.copy(); jaw.data = lo.data.copy(); bpy.context.collection.objects.link(jaw); jaw.name = "jaw_v4"
boolean(jaw, jaw_tool, "INTERSECT", keep=True)
gap = box("gap", [J["lo"][0] - 1, J["lo"][1] - 1, J["lo"][2] - 1], [J["hi"][0] + 1, J["hi"][1] + J["gap"], J["hi"][2] + J["gap"]])
boolean(lo, gap, "DIFFERENCE")
bpy.data.objects.remove(jaw_tool, do_unlink=True)
export(lo, "head_lower_v4.stl"); export(jaw, "jaw_v4.stl")
clear(); export(box("mouth_servo", H["mouth_servo"]["lo"], H["mouth_servo"]["hi"], 1.0), "mouth_servo_v4.stl")
print("V4_DONE")
