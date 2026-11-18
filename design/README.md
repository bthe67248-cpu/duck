# Duckling V3 — prototype redesign and MuJoCo starter

> **Revised 17 Nov 2026 (V3 handoff).** The thin tongue-like lip on `duck_head_lower_v3.stl` was replaced by a short rounded bill (`build_design.py`); the old part is kept as `modified/duck_head_lower_v3_pre_mouthfix.stl`, and `actual_mesh_preview_pre_mouthfix.png` shows the old version. Current renders: `results/geometry_v3/`. `original_parts/` (upstream meshes) is not included in this repo.

This revision supersedes V2. It uses a fuller, taller duckling head with no crown bumps, plus a solid-backed rear shield open at its physical upper and lower ends. It is a prototype geometry revision, not a physically fitted or locomotion-calibrated design.

## What to print / replace

|New file in modified/|Replaces|Change|
|-|-|-|
|duck\_head\_upper\_v3.stl|duck\_head\_upper.stl|Dome smoothly lifted and broadened; no feather bumps|
|duck\_head\_lower\_v3.stl|duck\_head\_lower.stl|Wider soft rounded bill below the sensor opening|
|hat\_rear\_shield\_v3\_open\_vertical\_ends.stl|electronics\_cover.stl, conceptually|Solid rear face and two side walls; open vertical ends; NOT a screw-compatible replacement|

All other supplied STLs are retained unchanged under original\_parts/. This includes alternative head/enclosure variants: do not print every file. Keep the black-PLA assembly's structural parts. Prefixes 2x and 4x indicate intended quantities, not necessarily multiple objects within a mesh.

## Rear shield orientation and fit

The broad rear face is FILLED. There is no top end cap and no bottom end cap. The inward-facing side remains open toward the HAT. In the STL, X is the board width, Y is its 110 mm length (vertical on the robot), and Z is the depth away from the board. Rotate it on assembly; STL Z is not the robot's vertical axis.

Based on the corrected HAT dimensions of 110 x 53.34 mm and 12 mm component height:

* Clear width: 54.94 mm, giving 0.8 mm allowance at each board edge.
* Length: 116.6 mm; centered board leaves 3.3 mm at each open end.
* Clear depth to the solid rear face: 16 mm.
* Back and side thickness: nominally 2.5 mm.
* Overall envelope: 59.94 x 116.6 x 18.5 mm.
* Four 5 x 2 mm strap slots in the side walls; no guessed PCB screw pattern.

Keep the HAT on its own standoffs. The shield does not support or electrically insulate the board by itself. If the specified 12 mm excludes PCB thickness, headers or underside parts, measure the total stack before printing. Route straps only to structural chassis anchors and clear of components. The original cover is shorter; this shield is a new part, not a verified drop-in mounting solution. Connector cutouts and a screw-mounted adapter remain dependent on board/mounting geometry. Check leg sweep, cables and balance before fixing its location.

## Head geometry and checks

Upper dome maximum height is now 50 mm, versus 32 mm originally. Original seam and low mounting region (z <= 6 mm) stay fixed. The front sensor-surround region at y <= -35 mm stays fixed; the rear/upper dome is smoothly deformed. Features elsewhere in the deformed region must be checked against hardware. No claim is made that every internal feature stays unchanged. The lower shell retains its original geometry with the new bill unioned onto its front.

The new lower-shell front-to-back extent is about 117.36 mm. Bill is integral and single-material, not a separately attached TPU part. The upper-shell shape change increases plastic volume and head mass; confirm neck servo capacity and balance using the actual printed weight.

All three generated STLs passed watertightness, positive-volume and single-connected-component checks. Results are in mesh\_checks.json. These checks do not certify self-intersection-free deformation, physical fit, sensor field of view, print strength or collisions through the complete joint range.

Import STLs in millimeters without automatic resizing. Inspect support needs around the bill and dome in your slicer's layer preview. The shield can lie on its solid rear face for printing. Test shield fit first. The supplied original mouth\_buttom\_tpu.stl is not watertight; it is unchanged and unused by the new head.

actual\_mesh\_preview.png is a direct render of the edited meshes. The cylinder beneath the head is only a simulation display fixture. The whole-robot image shown in chat is an AI-generated concept view with illustrative mounting, not an exact CAD assembly render.

## Quick start: Windows / MuJoCo

1. Extract this ZIP into a normal folder, for example C:\\AI\\duckling\_v3. Do not run files while still inside the ZIP.
2. Open PowerShell in the extracted folder. Install Python if `py --version` is unavailable.
3. Create an environment and install MuJoCo:

```powershell
py -m venv .venv
.\\\\.venv\\\\Scripts\\\\python.exe -m pip install mujoco numpy
```

4. Open the supplied moving-head demo:

```powershell
.\\\\.venv\\\\Scripts\\\\python.exe simulation\\\\run\\\_demo.py
```

5. To open the model in MuJoCo's viewer without the scripted motion loop:

```powershell
.\\\\.venv\\\\Scripts\\\\python.exe -m mujoco.viewer --mjcf=simulation/head\\\_demo.xml
```

6. To check physics without a window:

```powershell
.\\\\.venv\\\\Scripts\\\\python.exe simulation\\\\run\\\_demo.py --headless
```

The supplied demo was tested using MuJoCo 4.0.1 for 5,000 steps at 0.002 seconds per step (10 seconds), with finite joint coordinates. It animates two illustrative head joints. Pivot positions, masses, inertias, torque limits and gains are explicitly placeholders, not Fantech servo specifications. The frame is displayed beside the head, not attached to the demo's moving body. Visual meshes have collision disabled. This is NOT a walking model.

## How to import the complete robot correctly

STL contains triangles, not joints, servo axes or assembly relationships. These uploads are individually positioned around their local origins; loading them all at 0,0,0 does not assemble the robot. A photograph cannot provide accurate joint transforms.

1. Assemble the parts in your CAD program using real servo CAD and the black-PLA reference assembly. Replace the two head shells with the V3 meshes. Place the new frame relative to the actual board. Keep a neutral pose and record each servo shaft center, axis direction and parent link.
2. Group parts that move rigidly together. A mesh is a visual part; a MuJoCo body is a rigid link. Several printed parts and a servo case can belong to one link. Do not assign a joint to each STL. Servo case and output horn belong on opposite sides of the relevant joint.
3. Define an MJCF body tree rooted at the torso. Add a freejoint to the torso for walking. Add nested bodies for each actual leg degree of freedom, then neck/head joints. Confirm the physical joint order and servo count; do not infer it solely from filenames such as hip\_yaw2rol.
4. Use meters for all body positions and collision shapes. For these millimeter meshes use `scale="0.001 0.001 0.001"`. Never scale an already-meter mesh again.
5. Export an OBJ for each visual mesh. The supplied head STLs exceeded MuJoCo's STL triangle limit; OBJ equivalents are already included in simulation/meshes/. OBJ avoids that STL decoder limit but does not reduce render cost. Keep high-resolution print meshes separate from simplified simulation meshes.
6. Set each child body's position to its pivot relative to the parent body. Its joint axis is expressed in that body's frame. Place or rotate the mesh relative to the same pivot. Keep CAD export coordinates consistent. Use a known dimension (for example the 110 mm HAT length) to verify scale.
7. Add simple collision shapes (boxes, capsules and cylinders) to each link. A concave shield or shell must not be treated as one convex solid for contact. Use multiple boxes for the shield back and side walls. Keep detailed visual meshes at contype=0 and conaffinity=0. Use simple sole boxes for foot contact; represent TPU grip through measured contact parameters, not just the visual mesh.
8. Set each link's mass, center of mass and inertia from assembled CAD/material estimates or measurements. Include servo motors, battery, HAT, UNO Q, screws and cables. A shell's enclosed volume is not a reliable printed-infill mass estimate.
9. Add actuators for the real active joints. Specify measured angle limits, torque limits, damping and appropriate gains. XML angle="degree" applies to angle specifications such as joint ranges; hinge actuator targets in runtime control use radians. The supplied script demonstrates this conversion convention.
10. Start with the torso fixed and sweep one joint slowly at a time. Verify pivot, direction, mesh placement, limits and collisions. Then enable the torso freejoint, gravity and ground contacts. First test standing support, then balance control, then walking. Importing a model does not create a walking controller.

Illustrative link fragment — values must be replaced with measured robot data:

```xml
<asset>
  <mesh name="thigh\\\_visual" file="thigh.obj" scale="0.001 0.001 0.001"/>
</asset>
<!-- Inside the parent body: -->
<body name="thigh" pos="0 0 -0.03">
  <joint name="hip\\\_pitch" type="hinge" axis="1 0 0" range="-45 45" damping="0.03"/>
  <inertial pos="0 0 -0.025" mass="0.08" diaginertia="0.00004 0.00004 0.00001"/>
  <geom type="mesh" mesh="thigh\\\_visual" contype="0" conaffinity="0"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.05" size="0.009"/>
</body>
```

This fragment is an example, not dimensions for the supplied thigh shell. If the mesh was exported in its print-bed orientation, it needs an additional transform to line up with the body frame.

## Editing / regenerating these prototypes

build\_design.py reads original\_parts/ and regenerates modified/. It requires Python, trimesh, numpy and manifold3d:

```powershell
.\\\\.venv\\\\Scripts\\\\python.exe -m pip install trimesh numpy manifold3d
.\\\\.venv\\\\Scripts\\\\python.exe build\\\_design.py
```

After changing mesh geometry, regenerate simulation OBJ files with:

```powershell
.\\\\.venv\\\\Scripts\\\\python.exe simulation\\\\convert\\\_meshes.py
```

## Official references

* Python install and viewer: https://mujoco.readthedocs.io/en/stable/python.html
* Body hierarchy and modeling: https://mujoco.readthedocs.io/en/stable/modeling.html
* MJCF meshes, joints, inertials and actuators: https://mujoco.readthedocs.io/en/stable/XMLreference.html

The original design's licensing and attribution remain applicable to its meshes. No new license for the source XGO design is asserted here.

