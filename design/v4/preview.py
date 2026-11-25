"""V4 preview: robot in its start pose (mouth open) -> design/v4/preview/duck_v4.glb, then a .blend + renders.

    python design/v4/preview.py                      # export the .glb (project venv; reads model/duck_v4_gc_world.xml)
    E:\\blender\\blender.exe --background --python design/v4/preview.py -- blend     # .blend + 4 renders
    E:\\blender\\blender.exe design/v4/preview/duck_v4.blend                        # open it

preview/ is git-ignored: the robot geometry includes upstream (Pollen Robotics) meshes.
The body shell is see-through in the .blend so the battery (blue), boards (green), XT60 (yellow) and A010 show.
"""
import os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "preview")
os.makedirs(OUT, exist_ok=True)


def export_glb():
    import numpy as np, mujoco, trimesh
    repo = os.path.abspath(os.path.join(HERE, "..", ".."))
    m = mujoco.MjModel.from_xml_path(os.path.join(repo, "model/duck_v4_gc_world.xml")); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("sim_start").id)
    j = m.joint("mouth").id; d.qpos[m.jnt_qposadr[j]] = m.jnt_range[j][1] * 0.8
    mujoco.mj_forward(m, d)
    scene = trimesh.Scene()
    for g in range(m.ngeom):
        if m.geom_group[g] != 2 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        F = m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
        W = V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        rgba = m.mat_rgba[m.geom_matid[g]] if m.geom_matid[g] >= 0 else m.geom_rgba[g]
        mesh = trimesh.Trimesh(W, F, process=False)
        mesh.visual = trimesh.visual.ColorVisuals(mesh, face_colors=np.tile((np.array(rgba) * 255).astype(np.uint8), (len(F), 1)))
        nm = f"{m.body(m.geom_bodyid[g]).name}__{m.mesh(mid).name}__{g}"
        scene.add_geometry(mesh, node_name=nm, geom_name=nm)
    s = m.site("tof_body").id; R = d.site_xmat[s].reshape(3, 3); a = d.site_xpos[s]
    fx, fy, r = np.tan(np.radians(35)), np.tan(np.radians(30)), 0.25
    P = np.array([a] + [a + r * R @ np.array([1, u * fx, v * fy]) for u, v in ((1, 1), (-1, 1), (-1, -1), (1, -1))])
    fov = trimesh.Trimesh(P, [[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1], [1, 3, 2], [1, 4, 3]], process=False)
    fov.visual = trimesh.visual.ColorVisuals(fov, face_colors=np.tile([50, 150, 255, 40], (6, 1)))
    scene.add_geometry(fov, node_name="a010_fov_70x60", geom_name="a010_fov_70x60")
    scene.apply_transform(trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0]))   # z-up -> glTF y-up
    scene.export(os.path.join(OUT, "duck_v4.glb")); print("wrote", os.path.join(OUT, "duck_v4.glb"))


def make_blend():
    import bpy
    from mathutils import Vector
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=os.path.join(OUT, "duck_v4.glb"))
    for ob in [o for o in bpy.data.objects if o.type == "MESH"]:
        me = ob.data; col = [0.5, 0.5, 0.5, 1.0]
        ca = me.color_attributes.active_color if me.color_attributes else None
        if ca is not None and len(ca.data):
            col = list(ca.data[0].color)
        see_through = "body_shell_v4" in ob.name or "a010_fov" in ob.name
        if "body_shell_v4" in ob.name:
            col = [0.03, 0.03, 0.03, 0.35]
        if "a010_fov" in ob.name:
            col = [0.2, 0.6, 1.0, 0.15]
        mat = bpy.data.materials.new(ob.name + "_mat"); mat.diffuse_color = col; mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            bsdf.inputs["Base Color"].default_value = col; bsdf.inputs["Alpha"].default_value = col[3]
        if see_through and hasattr(mat, "surface_render_method"):
            mat.surface_render_method = "BLENDED"
        me.materials.clear(); me.materials.append(mat)
        ob.name = ob.name.split("__")[1] if "__" in ob.name else ob.name
    sc = bpy.context.scene
    sc.unit_settings.system = "METRIC"; sc.unit_settings.length_unit = "MILLIMETERS"
    sc.render.engine = "BLENDER_WORKBENCH"; sc.display.shading.color_type = "MATERIAL"; sc.display.shading.light = "STUDIO"
    sc.render.resolution_x, sc.render.resolution_y = 900, 760
    cam = bpy.data.objects.new("preview_camera", bpy.data.cameras.new("preview_camera")); sc.collection.objects.link(cam)
    sc.camera = cam; cam.data.lens = 50; cam.data.clip_start = 0.001
    target = Vector((0.0, 0.0, 0.15))
    for name, pos in (("front", (0.42, -0.34, 0.26)), ("side", (0.0, -0.55, 0.16)), ("rear", (-0.42, -0.32, 0.25)),
                      ("head", (0.25, -0.18, 0.30))):
        cam.location = Vector(pos)
        tgt = Vector((0.04, 0.0, 0.25)) if name == "head" else target
        cam.rotation_euler = (tgt - cam.location).to_track_quat("-Z", "Y").to_euler()
        sc.render.filepath = os.path.join(OUT, f"render_{name}.png"); bpy.ops.render.render(write_still=True)
    cam.location = Vector((0.42, -0.34, 0.26)); cam.rotation_euler = (target - cam.location).to_track_quat("-Z", "Y").to_euler()
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, "duck_v4.blend")); print("BLEND_OK")


if "blend" in sys.argv:
    make_blend()
else:
    export_glb()
