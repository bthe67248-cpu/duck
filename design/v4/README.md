# Duckling V4: A010 depth sensor, boards and LiPo inside the body, opening mouth

24–25 November 2026. **Simulation and concept CAD only.** The body now has its assembly split, board standoffs,
mounting bosses, battery retention and connector/cable envelopes, and the head clears the neck mechanism and the camera
(see "Fit fixes and assembly geometry"); hinge, pushrod and head-shell fasteners are still to do, and every clearance
must be checked on the real parts. V4 is now the only robot model: the V3 model, its rear shield, the V3 head
STLs and the V3 generator have been deleted (they are in git history).

## What changed against V3

| Area | V3 (deleted) | V4 |
|-|-|-|
| Body | upstream left/right shells + rear shield (clashed with the head in 129/600 sampled poses) | body shell + rear-bay lid + keel cover (`body_shell_v4`, `body_lid_v4`, `keel_cover_v4`: 79.5 cm³, 98.6 g at the default print settings), replacing the upstream battery holder too |
| Boards | old Pi Zero + HAT in the head; 110 mm HAT behind the rear shield | UNO Q + resized 70 × 53.34 mm HAT stack in a rear bay behind the leg sweep |
| Depth sensor | VL53L5CX 8×8 on the head | Sipeed MaixSense A010 in a chest pocket with a front window, level, 0.13 m above the floor |
| Battery | NP-F970-style pack visual | 2S LiPo about 800–1000 mAh (60 × 28 × 15 mm placeholder) in a compartment over a connector well, panel-mount XT60 |
| Head | rear servo stuck out about 7 mm; eye plate/ring (`face_part`, `noenoeil`) poked through the sides | 10 % wider, 16 mm longer at the back; eye plate and old head boards removed |
| Mouth | fixed bill | lower half of the beak is a `jaw` on a `mouth` hinge (0–30°), position actuator `mouth` (actuator 16, after the push motors), XL330 in the dome driving the jaw through a pushrod |
| Limits | upstream | neck pitch capped at 0.70 rad (was 1.047): beyond that, with the head tipped back, the back of the head reaches the board bay |

Mass (`model/mass_ledger.json`, estimates): robot 0.760 (upstream) → 0.819 kg. Torso 278 g (body shell, lid and cover
99 g, boards 45 g, 2S LiPo 55 g), head 191 g with the jaw (budget 189 g). See "Fit fixes and assembly geometry" and
"Mechanical validation and dynamics" below for the battery, head budget and servo delay.

## Fit fixes and assembly geometry (second review, 25 November 2026)

A second review found that the fit check now works, but the geometry it checks still clashed and the body had no way
to be assembled: no split, standoffs, bosses, battery retention or room for the connectors. Both are addressed here.
The CAD is still concept geometry (boxes and booleans driven by `v4_params.json`), now with the parts needed to put the
duck together; "Still open" below lists what the real parts need before printing.

### Static fit: before → after (`fit_check.json`, standing, mouth closed, penetration > 0.8 mm counts)

| Pair | before (mm) | after (mm) | Fix |
|-|-|-|-|
| upper head shell / camera lens | 6.7 | 0 | lens opening Ø19 (17 mm lens + 1 mm), open down to the seam (`head.lens_opening`) |
| mouth servo / M12 lens holder | 6.1 | 0 | mouth servo moved into the dome (below) |
| lower head shell / head-roll servo (`yaw_roll_motion` XL330) | 5.1 | 0 | clearance box swept over the head-roll range |
| lower head shell / neck-pitch bearing | 3.7 | 0 | clearance solid of revolution about the yaw axis, swept over roll |
| lower head shell / neck-pitch bracket | 2.5 | 0 | same |
| lower head shell / head XL330s (front, rear) | 2.3, 1.8 | 0 | pockets (servo box + 1.2 mm) |
| upper shell, lower shell, motor support / mouth servo | 2.2, 1.0, 1.6 | 0 | mouth servo moved |
| lower head shell / roll bearing | 1.1 | 0 | roll-swept clearance box |
| 2S LiPo / upstream power support | 1.1 | – | power support (the old battery holder) removed; its 13 g is subtracted |
| lower head shell / motor support | 0.9 | 0.9 | **explained**: the upstream motor support sits on the shell's ledge (a 0.3 mm² corner, CAD contact face; upstream parts show up to 1.1 mm the same way) |
| keep-out envelopes (USB, XT60, balance plug, mouth-servo cable) | 2.0–5.8 | 0 | new connector layout and cable routes (below) |

No other static pair is deeper than 0.8 mm (the next is the mouth-servo cable envelope against the servo, 0.6 mm:
the plugs sit on it); the new body parts, boards, LiPo, XT60 and A010 do not touch.

### Head

- **Lens opening.** `build_v4.py` now cuts the upper shell from the committed source `head_upper_v4_whole.stl`
  (renamed from `head_upper_v4.stl`; the cut shell is the new `head_upper_v4.stl`). `make_duck.py` places the head from
  the uncut source, so the cut does not move it.
- **Mouth servo.** A voxel search of the head cavity (all parts, 1 mm margin) found no place for an XL330 (34 × 20 ×
  26 mm) under the forehead: the M12 holder, the motor support, the eye bosses of the upper shell and the lower shell's
  floor slab fill it. The only free volume is the dome above the motor support, so the servo now lies flat there
  (x −13…13, y −22…12, z 40…60 in the lower-head frame), output axis along x, parallel to the jaw hinge. A pushrod in
  the plane x = 17 runs from the horn (front end, +x face) down to a 3 mm lever plate on the jaw heel (pin at y −40,
  z 8, 7 mm behind and below the hinge): pulling the rod up opens the jaw. Keep-out envelopes `mouth_pushrod`
  (oriented box along the rod) and `mouth_servo_cable` (plugs at the servo's rear end) are checked and clear.
- **Clearance around the neck mechanism** (`head_clearance.py`, writes `head.clearance`): measures, from the upstream
  parts in the generated model, the solid of revolution swept by everything on the neck-pitch body (and the neck body over
  head pitch 0.35 ± 0.6 rad) about the head-yaw axis, plus boxes for the head-roll servo and roll bearings and the two
  XL330s fixed in the head, each + 1.2 mm. `build_v4.py` subtracts the revolved profile and the roll boxes at 9 angles
  over the head-roll range (±0.436 rad) and the fixed boxes as they are, so `build_v4.py` itself still needs only
  `v4_params.json`. The floor opening around the neck grows and the inner ledges at the neck go; with the new jaw split
  the lower shell goes from 48.1 to 38.2 cm³. The neck-pitch bracket still touched the shell at full roll (1.1 mm, 7/300 poses) until
  the profile included the bracket's parts below the shell that roll up into it.
- **Jaw.** The jaw/shell split is no longer a plane at the hinge: the jaw is the beak in front of the hinge plane plus a
  quarter-disc heel (radius 10.5 mm) behind and below the hinge axis, and the lower shell loses that region grown by
  the 2 mm gap plus the heel's sweep to 36° above the back of the hinge. The mating faces are arcs about the hinge, so the
  gap is the same at every opening. Before: 0.48 mm rub at 15°, 1.0 mm at 30° (619/3000 poses). After: 0 mm at 0–30°
  (checked every 5°, and in the pose envelope with the mouth sampled over its range). The jaw now includes the cheek
  pieces inside the arc, so it is 5.9 g (was 4.4 g).

### Body: assembly split, standoffs, bosses, battery retention, connectors

Three prints instead of one closed shell (all in `v4_params.json` `body`; fasteners 12 × M2 + heat-set inserts,
4 g in the ledger):

| Part | What it does |
|-|-|
| `body_shell_v4` (68.9 cm³) | main shell. Board bay with a fixed **bulkhead** (x −60…−58) carrying four **UNO Q standoffs** (Ø7, 6 mm, M2.5 heat-set; UNO hole pattern, UNO side forward, HAT to the rear) and two **lid bosses** in the top corners; **battery compartment** walls up to the roof (with lightening windows) over a **connector well** with four **cover bosses**; **A010 rear posts** (the sensor is clamped between them and the window lip); **trunk mounts**: a flange under the rear-centre of the trunk plate (2 × M2 up into heat-set inserts pressed into the plate) and two bosses from the side walls to the hip-yaw XL330s' outer faces (2 × M2 into the servo cases); a cable window in the bulkhead |
| `body_lid_v4` (7.4 cm³) | rear lid of the board bay, 1.6 mm with vents: two hooks drop into slots in the bay floor, 2 × M2 at the top; locating lip; U-slot for the USB cable |
| `keel_cover_v4` (3.2 cm³) | bottom of the connector well, 4 × M2; two rails the LiPo stands on, the leads pass between them |

Assembly: boards onto the standoffs through the open rear, then the lid; LiPo into its compartment from below with its
XT60 male plugged into the panel-mount female in the well's rear wall, then the cover; the shell drops over the trunk
(the neck through the notch) and bolts to the trunk plate and the hip-yaw servos. To swap or charge the pack, remove the
cover (4 screws): the XT60 and the balance plug lie on it and the pack slides down.

Layout changes against the 24 November body:
- Board stack: 6 mm standoffs in front of it and 3 mm behind it for the HAT's solder tails; the stack moved 1.5 mm back
  and 7 mm down (x −82…−66, z −40…30) so that the bay roof is level with the rest of the body (outer z 41, was 42.5) and
  still clears a right-angle USB-C plug. Bay: x −85…−58 plus the 1.6 mm lid (−86.8), z −44…41 (was −35…42.5).
- USB-C: the UNO form factor puts it on a short edge, the top of the upright stack; a straight plug needed 25 mm above
  it. Now a **right-angle plug** (8 mm) with the cable out through the lid's U-slot (`usb_c_ra`, `usb_cable`).
- Servo bus: two right-angle JST-EH headers on the HAT's top edge (left and right leg chains); the cables drop between
  the stack and the bulkhead, pass the bulkhead window, run along the roof beside the battery compartment and down behind
  each hip-yaw XL330 (`servo_bus_l_*`, `servo_bus_r_*`, four boxes per side).
- LiPo: 1.5 mm clearance per side (was 0.5 / 1.0 mm) for ±1 mm pack tolerance and swelling (foam pads), held by the
  compartment walls, the roof (2 mm, pad) and the cover's rails; leads out of the bottom.
- XT60: a panel-mount female in the connector well's rear wall (8.2 × 15.8 mm cutout), the pack's male plugged into it
  inside the well; the leads loop forward under the keel wall with a 7 mm bend (`lipo_leads_drop`, `lipo_leads_loop`);
  the robot-side lead rises behind the well to the bulkhead window (`xt60_rear`, `power_lead`). Before: an XT60 that
  could not be unplugged through a 5 × 12 mm hole and a 9.5 mm placeholder for a 16 mm part. `xt60_v4` is now the mated
  pair inside the well (18 mm, 8 g).
- Balance plug: lies on the cover in front of the lead loop (`balance_plug`), reachable when the cover is off.
- A010 cable: from the sensor's rear along the centre line between the hip-yaw servos, across behind the right one,
  back beside the battery compartment and up to the bulkhead window (`a010_cable_1…4`).
- The keel cover is 2.5 mm narrower than the well on each side: in one sampled pose (hip roll inwards) the right shank
  reached its edge by 1.5 mm.

All 20 torso envelopes and 2 head envelopes are clear of every part at rest. They are estimates: check them against the
real connectors and cables.

### Pose envelope and recorded motions: before → after

"Clash" = a pair deeper than 0.8 mm that is clear at rest (no pair clashes at rest any more, so this is every clash).
Before: the 24 November run (old geometry, old recordings). After: `fit_check.json` from this branch, with the motions
re-recorded on the new model (`motions.py all --record`, 20 ms servo delay, every 50 Hz frame); `*_oldtraj` = the old
roulade and fall recordings replayed on the new geometry.

| Check | before | after | Remaining contacts |
|-|-|-|-|
| Pose envelope (3000 random poses) | 3000 poses with a clash (800 in pairs clear at rest) | **77** | only foot against foot or shank (random hip yaw/roll crossing the legs, up to 3.3 mm, as before); no head, jaw, body or envelope pair |
| balance, walk, pick, dance, sit/stand | none | none | sit/stand: 1 frame of foot proxies on the body front (no mesh clash) |
| obstacles | 7 frames (shank through the XT60 plug envelope, 4.1 mm) | **0** | the plug is now inside the well |
| roulade | 11 frames, 1.2 mm | 27 frames, **1.2 mm** (old recording: 18, 1.2 mm) | **explained**: the jaw meets the chest while the head is tucked in the roll; the collision proxies touch in the same frames (19), i.e. the head rests on the body in simulation. A 1.5 mm relief on the chest's top-front edge would remove it |
| fall + get-up | 668 frames, 7.0 mm | 20 / 1372 frames, **2.2 mm** (old recording: 26 / 889, 7.0 mm) | **explained**: feet and a shank strike the head shells, body shell and lid while the duck is on the ground (the proxies collide in 25 frames); these are impacts, not fit errors. The parts must survive them. The 7.0 mm on the old recording is the same foot hitting the head XL330 |

The 628 "head over the board bay" frames of the old fall run are gone: the straight USB-C plug envelope above the bay
no longer exists.

**Effect on the motions** (passes out of 3, servo stiffness × 0.95 / 1.0 / 1.05, 20 ms delay; main = the model before
this branch, same runs):

| Model | balance | walk | obstacles | fall + get-up | sit/stand | roulade | pick | dance |
|-|-|-|-|-|-|-|-|-|
| main | 3 | 0 | 1 | 2 | 3 | 3 | 3 | 3 |
| this branch | 3 | 0 | 1 | 1 | 3 | 3 | 3 | 3 |

Only the fall recovery changes (fails at × 1.0 now). It was already marginal (1/3 in the 24 November table, 2/3 on main
since the 100 g battery fix), and the torso is now 21 g heavier with its COM 3 mm lower. Pick touches the chest proxy
with a hip in 20–32 control steps (main: 0–18), with no mesh clash.

### Mass ledger (`model/mass_ledger.json`): before → after

| | before | after |
|-|-|-|
| Printed body (shell + lid + cover) | 71.0 g (shell only) | 98.6 g (85.5 + 9.2 + 3.9) |
| Removed upstream power support | – | −13.0 g |
| Fasteners (body), XT60 pair | –, 6 g | 4 g, 8 g |
| **Torso link** | 257.6 g, COM (−30.9, 0, 5.8) mm | **278.2 g**, COM (−32.5, 0, 2.4) mm |
| Torso Ixx / Iyy / Izz | 298 / 544 / 451 kg·mm² | 355 / 617 / 488 kg·mm² |
| Head shells + jaw | 113.0 g (62.3 + 46.3 + 4.4) | 107.1 g (61.1 + 40.1 + 5.9) |
| **Head link + jaw** | 197.1 g | **191.1 g** |
| **Robot** | 0.804 kg | **0.819 kg** |

The body is 28 g heavier because it now has a bulkhead, standoffs, bosses, compartment walls, a connector well, a lid
and a cover (at the default 1.2 mm walls / 30 % infill the 2 mm walls print almost solid); removing the old battery
holder takes back 13 g. The torso COM moves 1.6 mm back and 3.4 mm down (the bay and well are lower).

**Head against its budget.** Budget 188.8 g (the upstream head link the policies were trained with). V4 head 185.2 g
link + 5.9 g jaw = **191.1 g, 2.3 g (1.2 %) over** (was 197.1 g, 4.4 % over): shells + jaw 107.1 g (the lower shell is
6.2 g lighter after the clearance cuts and the jaw split; the jaw gained 1.5 g) plus the 18 g mouth servo. In the pessimistic case (upstream
parts at 1.08 g/cm³, see above) it is about 207 g, 10 % over. With the mouth servo in the dome the head COM moves 3.8 mm
(head body frame: +2.9, −1.6, +1.9 mm) and the head inertia drops 2–5 % (620 / 425 / 365 kg·mm², was 652 / 445 / 371). The
`print._proposed_head` settings (0.8 mm walls, 15 % infill on the shells and jaw) bring shells + jaw to 76.6 g and the
head to about 161 g (177 g pessimistic), under budget either way; they are still not applied (check the dome's
stiffness on a print first).

### Still open (not modelled)

- Jaw hinge pin, knuckles and bushing; horn, pushrod and clevis hardware; mouth-servo bosses in the dome.
- Screws or clips joining the upper and lower head shells.
- Neck cable pass-through for the head servos' bus, the mouth servo and the head camera (slack for yaw ±170°, pitch and
  roll).
- Trunk mounts: the two holes and inserts in the upstream trunk plate (x −15.5, y ±2.5) and the XL330 case holes the
  side bosses use must be checked on the real parts; the side bosses assume tapped M2 holes in the hip-yaw servos'
  outer faces.
- The UNO Q's bottom-connector height (the 6 mm standoffs assume ≤ 5 mm) and the real positions of its USB-C and the
  HAT headers.
- Panel-mount XT60 screws; foam pads for the LiPo; strain relief at the lid's USB slot.
- Collision proxies (`scripts/collision_proxies.json`) for the head and jaw are unchanged; the body's follow the outer
  blocks (a new box for the connector well).

## Mechanical validation and dynamics (review fixes, 24 November 2026)

A review found that the fit check missed deep penetration, the LiPo kept the upstream battery's mass, the servo delay
was recorded but not simulated, and the realistic profile published exact IMU readings and sensor poses. All four are
fixed; the fit check now reports real interferences that the old check could not see.

### Fit check (`check_v4_fit.py`, penetration > 0.8 mm)

**Method.** The old check looked for a surface sample of the other part within 2.5 mm and projected onto its normal,
one direction only, so a point 3 mm inside a part reported 0 mm, and a thin wall pushed through a part was invisible.
Now every visual mesh is a closed solid (`solid.py`): containment from signed vertical ray crossings (a generalised
winding number, robust to small gaps), depth = distance to the surface with no cut-off, and every pair is checked in
both directions (A's samples in B and B's in A). Tested against analytic solids (`tests/test_fit_solid.py`: sphere
boundary within 0.3 mm, depth within 0.25 mm, a 3 mm-deep point reports 3.0 mm where the old method gave 0, a 2 mm
wall through a block reports its 19 mm reach where the old one-way check saw 1 mm). The deepest results were confirmed
on the full-resolution STLs (the simulation meshes are decimated to 20k faces): same or deeper. The same pairs run
over the random pose envelope and over recorded motions (`scripts/motions.py --record DIR`, then `--traj DIR/*.npz`),
next to the MuJoCo collision proxies' self-contacts. The upstream Microduck head, checked the same way, has no pair
deeper than 1.1 mm (CAD contact faces), so 0.8 mm is close to the noise floor for upstream parts.

**Static fit (standing, mouth closed): pairs deeper than 0.8 mm.** Before = the old method's number for the same
pair, where the old check covered it at all (it only checked parts inside their own body's shells).

| Pair | new (mm) | old method (mm) | Cause |
|-|-|-|-|
| upper head shell / camera lens | **6.7** | 1.4 | the lens opening is smaller than the lens after the 10 % widening |
| mouth servo / M12 lens holder | **6.1** | 2.5 (band limit) | the mouth servo placeholder sits on the camera holder |
| lower head shell / head-roll servo (`yaw_roll_motion` XL330) | **5.1** | not checked | V4 reshaping (upstream shell: < 0.3 mm) |
| lower head shell / neck-pitch bearing | **3.7** | not checked | V4 reshaping (upstream: none) |
| lower head shell / neck-pitch bracket | **2.5** | not checked | V4 reshaping (upstream: none) |
| lower head shell / head XL330 (in the head) | 2.3 | 2.3 | mounting face, as upstream (≤ 1.1 mm there) |
| upper head shell / mouth servo | 2.2 | 2.2 | mouth servo placeholder |
| motor support / mouth servo | 1.6 | 1.5 | mouth servo placeholder |
| 2S LiPo / upstream power support (old battery holder) | 1.1 | 1.0 | the holder is still in the model; trim or delete it |
| lower head shell / roll bearing, mouth servo, motor support | 0.9–1.1 | 0.9–1.1 | contact faces |
| keep-out envelopes | 2.0–5.8 | – | see "Connector envelopes" below |

The old method also reported 1.4 mm for the closed jaw against the lower shell; the new one finds no overlap there.
The table in the earlier V4 write-up ("lens 1.3 mm, servos 1.4/2.1, mouth servo 1.0 mm") under-reported every pair.

**Pose envelope (3000 random poses, 1697 part pairs, seed 1).** Every pose clashes, because the four head/neck
overlaps above already exist at rest. Clashes in pairs that are clear at rest (caused by motion): **800 / 3000 poses**.

| Pair | poses (new) | poses (old method) | worst (mm) |
|-|-|-|-|
| jaw / lower head shell | 619 | 377 | 1.0 |
| lower head shell / neck servo (`neck` XL330) | 106 | 100 | 2.7 |
| left foot / right foot (random hip yaw/roll ±0.3 rad crossing the feet) | 76 | 74 | 3.3 |
| `usb_c_top` envelope / head shells (head tipped back over the board bay) | 60 | 48 | 3.2 |
| any moving part / body shell, boards, LiPo, XT60, A010, power support | 0 | 0 | – |

The jaw rubs the lower shell from about 15° opening: 0.48 mm at 15°, 0.72 at 20°, 1.0 mm at 30° (confirmed on the
full-resolution meshes). Move the hinge axis back/down or chamfer the jaw's rear edge, or limit the mouth to ~12°.

**Recorded motions** (`motions.py all --record`, final model with the LiPo and 20 ms servo delay, every 50 Hz frame).
"Frames" counts clashes in pairs that are clear at rest; proxy = MuJoCo collision-proxy self-contacts > 0.8 mm.

| Motion | frames | clashes caused by motion | proxy contacts |
|-|-|-|-|
| balance, walk, pick, dance | 550, 500, 315, 1100 | none | none |
| sit/stand | 475 | none | 2 frames, right foot / body front |
| roulade | 375 | 11: lower head shell 1.2 mm and jaw 1.1 mm into the body shell (head tucked); head over the `usb_c_top` space | 17: head/jaw proxies on the body proxies |
| fall + get-up | 889 | 668: right foot up to **7.0 mm** into the head XL330 and the head shells (the foot swings into the head on the ground), left foot 2.2 mm into the body shell, lower head shell 2.8 mm into the neck servo; the head lying back over the `usb_c_top` space (628 frames) | 22: feet on the head and board-bay proxies |
| obstacles | 1601 | 7: the right shank sweeps through the `xt60_mating_plug` envelope (4.1 mm), so a plug hanging below the keel would be kicked | none |

The walking gaits stay clear. Contact only happens in falls and rolls, where parts really do strike each other. The
simulator lets meshes pass through because only coarse proxies collide, so in a fall the real foot hits the head
shell and the head XL330: those parts need to survive that impact. The roll presses the head into the chest, so the
body shell's chest corner needs about 1.5 mm relief or the head stops on it. Proxy contacts and mesh clashes agree
on where these happen.

### Missing hardware detail (24 November; most of it is addressed in "Fit fixes and assembly geometry" above)

Fasteners and bosses
- The body shell is one closed solid: no split, lid or cover, so it cannot be printed around the boards or opened. It
  needs a parting line (e.g. a removable rear-bay lid and keel cover) with M2 heat-set insert bosses (about 3.5 mm
  diameter, 4–5 mm deep), and a mounting interface to the trunk frame (the upstream shells' screw points are gone).
- UNO Q + HAT: no standoffs. The stack has 2.0 mm behind it (rear wall), 1.3 mm at the sides, 2.0 / 1.5 mm below /
  above. That is less than the through-hole solder tails of a HAT, and there is no room for side bosses; the UNO
  mounting holes (3.2 mm) need standoffs from the bay wall, i.e. at least 4–5 mm more bay depth.
- 2S LiPo: no strap, cradle or clip, and 0.5 mm clearance per side in x (1.0 mm in y) in the keel; real packs vary by
  ±1 mm and swell. Give it 1.5–2 mm per side and a strap slot.
- A010: no bosses or clips in the chest pocket (6.3 mm free behind it, 1.65 mm at the sides).
- Mouth servo: no bosses for the XL330's M2 case holes; it also intersects the M12 lens holder (6.1 mm), the upper
  shell (2.2 mm) and the motor support (1.6 mm), see the static table.
- Jaw: no hinge pin, bushing or bosses, and the servo-to-jaw link is not modelled.
- Head shells: no screws or bosses joining the upper and lower shells.
- XT60: no retention; the pack-side connector floats in the keel.

Connector envelopes (keep-out boxes in `v4_params.json` `envelopes`, checked like parts; ESTIMATES)
- `usb_c_top` (straight USB-C on the UNO Q's short edge, which is the top of the upright stack): 3.2 mm into the shell
  roof. With the board upright there is no room for a plug above it.
- `usb_c_rear` (a rear-facing plug through `usb_access`): clear. The cut only works with a right-angle receptacle or a
  short right-angle adapter; the UNO form factor's connector points up.
- `xt60_mating_plug` (female XT60 + lead pushed on from below): 2.0 mm into the keel floor. The `xt60_access` opening is
  5 × 12 mm; an XT60 housing is 8.2 × 15.8 mm, so the pack cannot be unplugged through it.
- `balance_plug` (2S JST-XH beside the XT60): 2.9 mm into the keel walls; there is nowhere to stow it, and it must be
  reachable for charging.
- `a010_usb` (straight USB-C at the A010's rear face): clear at rest (touches the A010, 0.5 mm).
- `mouth_servo_cable` (JST-EH plug + bend at the servo's end): 5.8 mm into the upper shell, 4.4 mm into the lower.

Cable bends and routes (no envelope yet)
- LiPo leads: 1.0 mm between the pack's bottom and the XT60 placeholder, which is also only 9.5 mm long (a real XT60
  half is about 16 mm). 14 AWG silicone leads need roughly a 10 mm bend radius, so the keel needs 15–20 mm more depth
  or the leads leave the pack sideways.
- Neck pass-through: the head servos' bus, the mouth servo and the head camera's cable must run from the torso through
  the neck with slack for head yaw ±1.2 rad, pitch and roll; not modelled.
- Leg servo bus from the HAT to both hips past the battery keel; A010 USB from the chest pocket to the rear bay
  (about 120 mm past the pack); neither routed.

### Mass: battery and head budget

**2S LiPo (`make_duck.py`).** The upstream torso inertial includes its battery (`np_f970`); before, V4 kept that mass
and only assumed the 2S pack weighed the same. Its mass is not published, but the part is a real 2S battery: the mesh
is 38.6 × 20.6 × 70.8 mm, the size of a Sony NP-F550/F570 Li-ion camcorder pack (the NP-F970 in its name is 51 mm
thick), and retail listings for that size give about 100 g. A mass fit agrees (the upstream neck, shank and hip links
imply an XL330 of 17.9 g and printed parts at 1.08 g/cm³, which leaves 123 g of the 199 g torso; fits to the torso
COM give 103–125 g). `UPSTREAM_BATTERY_G = 100` (until the pack is weighed; it was a 115 g estimate) is now subtracted at the
battery's pose (uniform density over its mesh) and the LiPo is added as a 55 g uniform box at its V4 location
(`v4_params.json` `battery`, COM (−37.5, 0, 5.0) mm in the torso frame; principal inertia 4.6, 17.5, 20.1 kg·mm²).

| | before | after |
|-|-|-|
| Torso link | 302.6 g, COM (−31.4, 0, 2.7) mm | 257.6 g, COM (−30.9, 0, 5.8) mm |
| Torso inertia Ixx / Iyy / Izz | 232 / 489 / 431 kg·mm² | 298 / 544 / 451 kg·mm² |
| Robot | 0.849 kg, COM height 139.0 mm | 0.804 kg, COM height 141.1 mm |

The inertia rises although the torso is lighter: the 60 mm-tall pack sits higher and further from the torso COM than the
flat upstream battery it replaces. Weigh the real pack and the torso link (`config/robot_measurements.example.json`).

**Head (`head_mass.py`).** Budget: the upstream head link the policies were trained with and the neck servos carry,
188.8 g. The V4 head is 192.7 g (shells 108.6 g + mouth servo 18 g on the upstream remainder) plus the 4.4 g jaw =
**197.1 g, 8.3 g (4.4 %) over**. That figure subtracts the replaced upstream parts at solid PLA density (122.6 g); if the
upstream CAD used the 1.08 g/cm³ its other links imply, 16 g too much was subtracted and the head is about 213 g (13 %
over). The shells are solid meshes about 3 mm thick on average (upper 2.9, lower 3.5 mm), so at 1.2 mm perimeters per
face they are almost all wall: infill barely matters.

| Shells + jaw (g), wall per face × infill | 10 % | 15 % | 20 % | 30 % |
|-|-|-|-|-|
| 0.8 mm | 76.7 | 80.0 | 83.2 | 89.8 |
| 1.0 mm | 91.8 | 94.2 | 96.6 | 101.5 |
| 1.2 mm (model default) | 106.6 | 108.2 | 109.8 | **113.0** |

Proposal (recorded as `print._proposed_head` in `v4_params.json`, not applied): 0.8 mm walls (two 0.4 mm perimeters) and
15 % infill on both head shells and the jaw, −33 g, head about 164 g (180 g in the pessimistic case), under budget with
margin. 1.0 mm / 15 % (−19 g, 178 g) if the dome proves too soft in a drop. Thinning the shell geometry itself to 2.0 mm
(a CAD offset) would save 26 g at the current print settings. A 9 g-class micro servo for the jaw instead of the 18 g
XL330 saves about 9 g more. `print` settings per part are now read from `v4_params.json` by `make_duck.py`.

### Servo delay

`make_duck.py` now writes the servo command delay into every model as `<custom><numeric name="servo_delay_s">`: the
calibration's measured `servo.delay_s`, else an ESTIMATE of 20 ms (the upstream policies are trained with 3–6 physics
steps, 15–30 ms, of actuator lag). `scripts/motions.py` and `world_sim` apply it with `duck_nav/actuation.py`, a pure
transport delay at physics-step resolution (the servos act on the target issued 4 physics steps earlier). It is not
MuJoCo's actuator `delay` attribute because that needs MuJoCo ≥ 3.4 and CI pins 3.3.6; where it exists the response is
identical (`test_actuation.py`). Overrides: `motions.py --servo-delay S`, launch `servo_delay:=S`. The ledger keeps
`servo_delay_s` as the measured value (null until measured; MATLAB reads it) and adds `servo_delay_sim_s`.

**Effect on the motions** (`scripts/motions.py all`, ground-contact model; passes out of 3 runs per row, servo stiffness
×0.95 / 1.0 / 1.05, because single deterministic runs of the marginal motions flip with tiny changes):

| Model, servo delay | balance | walk | obstacles | fall + get-up | sit/stand | roulade | pick | dance | walk speed (m/s) |
|-|-|-|-|-|-|-|-|-|-|
| before: old mass, no delay | 3 | 0 | 0 | 0 | 0 | 3 | 3 | 3 | 0.077–0.095 |
| new mass (LiPo), 0 ms | 3 | 2 | 0 | 2 | 2 | 3 | 3 | 3 | 0.085–0.103 |
| new mass, 10 ms | 3 | 0 | 0 | 2 | 3 | 3 | 3 | 3 | 0.045–0.082 |
| **new mass, 20 ms (default)** | 3 | 0 | 2 | 1 | 3 | 3 | 3 | 3 | 0.063–0.090 |
| new mass, 30 ms | 3 | 0 | 2 | 2 | 3 | 2 | 3 | 3 | 0.036–0.098 |
| new mass, 40 ms | 3 | 1 | 2 | 1 | 3 | 3 | 3 | 3 | 0.100–0.109 |

No delay up to 40 ms breaks standing, sit/stand, pick or dance. The lighter LiPo makes sit/stand and the fall recovery
work where the old mass failed (0/3 → 2–3/3). Walk (0.8 m in 8 s at a 0.3 m/s command), the plank course and the
fall recovery are marginal in every configuration and change between neighbouring stiffness values as much as between
delays, so these rows show robustness, not a ranking. The walk criterion failed before this change too.

### Realistic sensor profile (`world_sim`, `duck_nav/sensor_errors.py`)

New parameters (defaults are ESTIMATES for a BMI088/ICM-42688-class IMU and XL330 servos). They are **off by default**
(`sensor_errors:=false`) until paired trials have measured their effect; `sensor_errors:=true` switches them on and
`sensors:=ideal` never has them:

| Error | Parameters (default) | Where it shows |
|-|-|-|
| IMU noise | `imu_gyro_noise` 0.002 rad/s, `imu_accel_noise` 0.02 m/s² per 50 Hz sample | `/imu` |
| IMU bias | `imu_gyro_bias` 0.005 rad/s and `imu_accel_bias` 0.1 m/s² (turn-on, σ per axis), random walks `imu_gyro_bias_walk` 1e-5 rad/s/√s, `imu_accel_bias_walk` 1e-4 m/s²/√s | `/imu`; the accelerometer bias tilts the attitude estimate by bias / g (0.1 m/s² ≈ 0.6°) |
| IMU misalignment | `imu_misalign_deg` 0.5 (σ per axis) | `/imu` axes; roll/pitch part tilts the attitude estimate |
| Attitude noise | `attitude_noise_deg` 0.1 | the tilt used for the sensor poses and the `base_level` TF |
| Encoders | `enc_offset_deg` 0.5 (zero offset σ), `enc_noise_deg` 0.05, `enc_counts_per_rev` 4096, `enc_vel_noise` 0.05 rad/s | `/joint_states`; the head camera pose (forward kinematics with the measured angles) |
| Extrinsics | `tof_extrinsic_deg` 1.0 / `tof_extrinsic_mm` 2.0, `cam_extrinsic_deg` 1.0 / `cam_extrinsic_mm` 2.0 (σ per axis) | the simulated A010 and camera look from a mount perturbed once per run; `/tof/pose_level` and `/camera/pose_level` report the CAD mount |
| Timing | `imu_stamp_jitter` 0.5 ms, `cam_stamp_jitter` 3 ms, `tof_stamp_jitter` 2 ms (σ, clamped so a stamp is never later than its publication time, kept increasing), `tof_latency_jitter` 10 ms (uniform extra delay, frames stay in order) | message stamps |

Published sensor poses are now computed the way the robot would: nominal mounts, measured joint angles, true heading
and the IMU-derived tilt (`level_pose`, which reproduces the old world-frame formula exactly when the errors are zero,
`test_sensor_errors.py`). The walking policy still sees exact gyro, gravity and joints unless
`policy_sensor_errors:=true`. Start-up log line: the drawn biases, offsets and extrinsic errors.

`score_node` now pairs each SLAM estimate with the nearest ground-truth sample within `gt_match_tol` (10 ms) instead
of an identical stamp: with jittered ToF stamps the first smoke run scored 0 samples.

**Closed-loop smoke runs** (300 s, seed 0, realistic profile, 20 ms servo delay; these check that it runs, they do not
compare):

| Run | sensor errors | sim / wall | SLAM mean / max error | walked | falls |
|-|-|-|-|-|-|
| `mech_noerr_s0` | off | 300 / 300 s | 0.16 / 0.36 m | 6.5 m | 0 |
| `mech_err2_s0` | on (defaults, stamps not yet capped) | 300 / 357 s (**not real time**: CPU shared with fit-check jobs) | 0.32 / 0.64 m | 9.3 m | 0 |
| `mech_clamp_s0` | on (defaults, stamps capped at publication) | 300 / 300 s | 0.26 / 0.29 m | 6.3 m | 0 |

Uncapped stamp jitter put about half of the `/imu` stamps (and so the SLAM poses) after `/clock`, and nav_node stopped
with "stale pose" (23 and 36 times in the two uncapped runs, 2 without errors, 0 on main). With stamps capped at
the publication time, `mech_clamp_s0` logged 0.

These smoke runs predate the camera-pose fix below (they ran with `/camera/pose_level` at the origin); they show
that the stack runs, not how well. `gyro_calib_s` (off by default in the launch file) would remove most of the
0.005 rad/s gyro turn-on bias.

**Paired 600 s closed-loop trials** (seeds 0–5, realistic profile, launch defaults, three runs at a time on separate ROS
domains, every run sim = wall throughout):

| Version | beacon | SLAM mean / max error | walked (mean, median) | coverage | falls | "slam weak" / "exploring" events |
|-|-|-|-|-|-|-|
| (a) main `c1cf128` | 4/6 | 0.35 / 1.21 m | 27.5 m, 32.1 m | 82 % | 1 | 187 / 85 |
| (b) this branch before the camera fix, errors off | 0/6 | 0.26 / 0.54 m | 14.2 m, 14.6 m | 61 % | 0 | 402 / 22 |
| (c) same, `sensor_errors:=true` | 0/6 | 0.34 / 1.01 m | 15.4 m, 15.0 m | 63 % | 1 | 344 / 29 |
| (f) camera fixed, 115 g upstream-battery estimate, errors off | 2/6 | 0.25 / 0.44 m | 10.4 m, 8.8 m | 51 % | 0 | 442 / 21 |
| (g) same, `sensor_errors:=true` | 1/6 | 0.55 / 1.13 m | 14.8 m, 14.2 m | 64 % | 0 | 368 / 28 |
| **(h) this branch: camera fixed, 100 g NP-F570 battery, errors off (default)** | **4/6** | 0.29 / 1.07 m | 27.9 m, 26.4 m | 87 % | 1 | 220 / 32 |

Isolation runs on seeds 0, 2, 3 (before the camera fix):

| Version | beacon | SLAM mean / max | walked | "slam weak" / "exploring" |
|-|-|-|-|-|
| (a) main, same seeds | 3/3 | 0.26 / 1.09 m | 34.3 m | 198 / 30 |
| (d) this branch, `servo_delay:=0` | 0/3 | 0.28 / 0.47 m | 8.3 m | 445 / 16 |
| (e) this branch's code with main's robot model (`mjcf:=`), no delay | 0/3 | 0.34 / 1.09 m | 20.6 m | 174 / 52 |

Per seed (beacon time or "no" / SLAM mean error / walked):

| seed | (a) main | (f) 115 g, errors off | (g) 115 g, errors on | (h) 100 g, errors off |
|-|-|-|-|-|
| 0 | 184 s / 0.22 m / 38.0 m | no / 0.31 m / 10.3 m | no / 0.34 m / 15.3 m | no / 0.62 m / 22.2 m |
| 1 | no / 0.24 m / 17.6 m (1 fall) | no / 0.15 m / 6.0 m | 77 s / 0.08 m / 13.2 m | 147 s / 0.16 m / 41.6 m (stopped at 570 s, 97 % mapped) |
| 2 | 308 s / 0.26 m / 30.2 m | no / 0.17 m / 7.3 m | no / 0.99 m / 14.1 m | no / 0.24 m / 18.1 m (1 fall) |
| 3 | 194 s / 0.29 m / 34.8 m | 130 s / 0.25 m / 15.3 m | no / 1.26 m / 14.4 m | 96 s / 0.36 m / 26.6 m |
| 4 | 216 s / 0.84 m / 33.9 m | no / 0.28 m / 6.4 m | no / 0.47 m / 24.4 m | 228 s / 0.20 m / 33.0 m (stopped at 450 s, 96 % mapped) |
| 5 | no / 0.28 m / 10.7 m | 209 s / 0.34 m / 17.0 m | no / 0.13 m / 7.6 m | 95 s / 0.17 m / 26.1 m |

What the trials found:

1. **Bug, fixed (4c99e77):** world_sim computed the published sensor poses from its own kinematics copy with
   `mj_kinematics`, which does not compute camera poses, so `/camera/pose_level` sat at the floor origin with sensor
   errors on or off: the beacon was never reached in (b), (c), (d), (e). `robot_kinematics()` adds `mj_camlight`
   (`test_robot_kinematics_includes_cameras`). The sensor-error models also drew from world_sim's main generator,
   shifting the ToF and camera noise against main for the same seed; they now have their own.
2. **The servo delay is not the cause:** (d) with no delay is no better than (b).
3. **The trunk mass is:** with main's robot model (e) the "slam weak" rate is main's (174 vs 198) and the duck walks
   twice as far as (b)/(d). The model's only change is the trunk inertial. With the upstream battery taken as a 115 g
   estimate the trunk was 60 g lighter than main's (0.303 → 0.243 kg) and (f) navigated far worse than main. The
   upstream battery part is a real 2S pack (NP-F550/F570 size, about 100 g in retail listings; "Mass: battery and head
   budget" above); with 100 g the trunk is 45 g lighter (0.258 kg) and (h) matches main: beacon 4/6, 27.9 m walked,
   87 % coverage. Two (h) runs ended early because the explorer ran out of frontiers with 96–97 % of the room mapped.
   An earlier open-loop check (`motions.py walk` at one speed) suggested the lighter trunk walks less straight; a sweep
   over 0.25–0.5 m/s does not separate main, 115 g and 100 g (mean heading change 49°, 55°, 68° in 8 s, single runs
   ranging 19–122°), so that check is too noisy to rank them.
4. **Sensor errors** (g vs f, 115 g model): SLAM mean error 0.25 → 0.55 m (two seeds near or above 1 m mean); distance
   and coverage are no worse. They stay off by default; (g) has not been repeated with the 100 g model.

So with the camera fix and the 100 g battery this branch navigates as well as main. The closed loop is sensitive to
the trunk mass (15 g separates (f) from (h)), so weigh the upstream battery or the assembled V4 and set
`UPSTREAM_BATTERY_G` / `battery.mass_g` from the scale before trusting either; if the real trunk is much lighter than
main's, the walking policy should be fine-tuned on the measured mass.

## SLAM for a body-fixed depth sensor (`ros2/duck_nav`)

The head-mounted ToF mapped by sweeping with the head. A sensor fixed to the body cannot, so:

- **Start-up survey** (`survey_turn`, default 6.6 rad): the duck turns in place a full circle before exploring, and SLAM
  keeps mapping every frame until it has turned that far (`bootstrap_turn`; otherwise a partly new view is rarely
  accepted and never mapped).
- **Confirmed relocalisation** (`reloc_confirm`, default on): a relocalisation jump is held until a match at least
  0.5 s later proposes the same correction (±6 cm / ±4°). Window back at ±0.8 m / ±25° (the narrow ±0.4 m / ±10°
  could not reach heading errors after turns).
- **Look-around** (`weak_look_after`, default 3 s): while localisation is weak or the sensor is *blind* (frames arrive
  but nothing is in range, e.g. closer than the A010's 0.2 m minimum), turn 1.5 s / hold 2 s, never drive forward.
- ToF simulation is batched (`mj_multiRay`), so 100 × 100 zones run in real time.

**First attempt (branch `a010-sim`): better, not solved.** Closed loop, 600 s, realistic profile (all runs held real time):

| Run | Zones | SLAM mean / max error | Beacon | Coverage | Walked | Falls |
|-|-|-|-|-|-|-|
| `v4d_s0` | 25×25 | 0.74 / 1.43 m | no | 50 % | 10.1 m | 1 |
| `v4d_s1` | 25×25 | 0.37 / 1.17 m | no (seen, estimate 0.41 m off) | 70 % | 22.1 m | 1 |
| `v4d100_s0` | 100×100 | 2.69 / 4.72 m | no | 96 % | 19.6 m | 0 |
| `v4c_*` (before the blind fix) | 25 / 100 | 0.22–1.55 m | no | 62–68 % | 8–19 m | 0 |
| `v4_s0_old_keyframe_bug` (no fix) | 25×25 | 1.79 m | no | 58 % | 5.9 m | 0 |
| `a010_head_s0` (V3 body, A010 on the head) | 25×25 | 0.08 m until a fall at 86 s | 46 s | 98 % | 14.6 m | 1 |

The robot no longer freezes, maps much more of the room, and the survey map is good (error under 0.2 m for the first
20–40 s). It then drifts while matches are still *accepted* (0.2 → 0.4–1.5 m within 25–60 s), so matching locks onto a
slightly wrong map; and it spends most of the run "weak". The head-mounted A010 run localised far better. The body
mount needs more SLAM work (likely: keyframe map updates only from confirmed-good poses, a wider normal search while
turning, or a short head-sweep substitute such as slow body yaw oscillation) or the A010 goes on the head. One run per
case; run-to-run noise is large.

### Fixing the drift (branch `v4-body-slam`)

**Diagnosis.** `tools/slam_replay.py` now replays V4 bags (`--profile v4`: 25 × 25 zones, 70 × 60°, 2.5 m; `--zones`,
`--fov`, `--fov-v`, `--max-range` override it) and writes a per-frame CSV (`--dump`: truth and estimate at acquisition,
match reason, overlap, match std, accepted / mapped / relocalising / bootstrap flags). On `v4d_s0`, `v4d_s1` and `v4c_s0`
it showed three separate faults:

1. **Map and pose ratchet together while turning.** In the survey turn the forward command is zero, yet the estimate
   slid 0.1 m sideways in 1 s through a run of *accepted* matches of about 5 mm each, all in the same direction. Zones
   that land just past the mapped end of a wall score as badly as zones in known free space, so the matcher shifts the
   pose to pull them onto the mapped end; the frame is then mapped at that pose, and the next frame repeats it. Every
   frame was mapped (`map_move` = `map_turn` = 0), so the error went straight into the map.
2. **Unconfirmed heading jumps while relocalising.** `reloc_confirm` held *accepted* relocalisation matches but not
   *partial* ones (one direction + heading). With the wide relocalisation prior those swung the heading by 10–30° per
   frame; once the error passed the ±25° window every match hit the search boundary and the robot stayed lost.
3. **Relocalisation that never ends.** Even at 0.2 m error, 85 % of frames were "relocalising": confirmation also held
   corrections of a few cm, so every other frame was `reloc_unconfirmed`, the accepted streak reset, health never
   returned to "ok", and the search stayed wide. In closed loop that is the "weak most of the run" behaviour.

**Changes** (`FrameLocalizer` / `DuckSLAM`; launch `body_slam:=true`, the default, sets them for `sensors:=realistic`;
the node defaults keep the head-sweep behaviour, and `sensors:=ideal` is unchanged):

| Parameter | V4 value | Effect |
|-|-|-|
| `map_move`, `map_turn` | 0.1 m, 0.17 rad | keyframe mapping after bootstrap instead of every frame |
| `unknown_ll` | -1.125 (1.5 σ) | never-observed cells score above known free space (fault 1) |
| `reloc_partial` | false | partial matches are not applied while relocalising (fault 2) |
| `confirm_free_xy`, `confirm_free_th` | 0.08 m, 0.07 rad | smaller relocalisation corrections need no confirmation (fault 3) |

Also new, off by default: `map_max_std` (map only tightly matched frames), `map_accepted_only`, and in the navigator
`yaw_sweep_rate` / `yaw_sweep_period` (body yaw oscillation while walking).

**Replay** (open loop on the recorded path; mean error in m, "ok" = share of time the motion gate would allow driving):

| Policy | `v4d_s0` | `v4d_s1` | `v4c_s0` |
|-|-|-|-|
| recorded live run | 0.74 | 0.37 | 1.33 |
| `a010-sim` policy | 0.34 (ok 8 %) | 0.58 (9 %) | 1.69 (9 %) |
| + `reloc_partial` false | 0.30 | 3.56 | 0.48 |
| + keyframes 0.1 m / 10° | 0.24 | 0.38 | 0.37 |
| + `unknown_ll` -1.125 | 0.25 | 0.24 | 0.15 |
| + `confirm_free` 0.08 m / 4° (**the V4 defaults**) | 0.25 (ok 46 %) | 0.23 (60 %) | 0.16 (82 %) |
| defaults with `map_max_std` 0.02 instead of keyframes | 0.65 | 1.75 | 1.49 |
| `unknown_ll` -0.5 | 0.14 | 0.44 | 0.25 |
| relocalisation window 1.0 m / 40° | 0.24 | 0.55 | 0.27 |
| match gain 1.0 | 0.23 | 0.57 | 0.33 |

Mapping only frames with a tight match covariance (the "confirmed-good poses" idea) was worse: the map stops growing,
so the duck relocalises more. A wider relocalisation window or full match gain did not help.

**Closed loop**, paired on seeds 0–5: `a010-sim` (`v4sbase_*`) against this branch (`v4snew_*`), and this branch with
`yaw_sweep_rate:=0.4` (`v4syaw_*`). 600 s, realistic profile, 25 × 25 zones, launch defaults, one run per cell; every run
held real time (`sim N s, wall N s` equal). A run ends early when exploration stalls with the beacon already reached.

| Seed | `a010-sim`: SLAM mean / max, beacon, coverage, walked | this branch | + yaw sweep 0.4 rad/s |
|-|-|-|-|
| 0 | 0.46 / 0.89 m, no, 70 %, 18.9 m | 0.22 / 0.70 m, **495 s**, 74 %, 36.6 m | 0.17 / 0.57 m, **120 s**, 96 %, 29.1 m |
| 1 | 1.08 / 2.05 m, no, 46 %, 17.5 m | 0.72 / 2.12 m, **253 s**, 82 %, 30.5 m (1 fall) | 0.46 / 0.96 m, **211 s**, 77 %, 37.9 m |
| 2 | 1.13 / 2.39 m, no, 54 %, 18.3 m | 0.45 / 1.81 m, **223 s**, 94 %, 44.6 m | 0.27 / 0.51 m, **193 s**, 73 %, 27.2 m |
| 3 | 2.04 / 3.19 m, no, 62 %, 11.1 m (1 fall) | 0.29 / 0.60 m, **269 s**, 96 %, 32.1 m | 0.32 / 0.76 m, **167 s**, 84 %, 36.6 m |
| 4 | 0.67 / 1.45 m, no, 43 %, 16.3 m | 0.17 / 0.38 m, **153 s**, 94 %, 38.2 m | 0.28 / 0.67 m, **278 s**, 94 %, 32.3 m |
| 5 | 0.25 / 1.23 m, no, 68 %, 12.8 m (1 fall) | 0.52 / 2.13 m, **273 s**, 96 %, 35.4 m | 0.52 / 0.92 m, **230 s**, 81 %, 36.9 m |
| **mean** | **0.94 m, beacon 0/6, 57 %, 15.8 m** | **0.40 m, beacon 6/6 (278 s), 89 %, 36.2 m** | **0.34 m, beacon 6/6 (200 s), 84 %, 33.3 m** |

Beacon estimate error when the run ended: `a010-sim` 0.47 / 2.60 m on the two seeds that ranged it at all; this branch
0.25–0.85 m (median 0.31 m); with yaw sweep 0.14–1.00 m (median 0.44 m).

**Result: the body-mounted A010 now works for this task in simulation.** The duck reached the beacon on 6/6 seeds (0/6
before), covered 89 % of the room instead of 57 %, and walked twice as far; mean SLAM error fell from 0.94 to 0.40 m. It is
still worse than the head-mounted A010 (0.08 m), and late drift remains: three seeds end with 1.4–2.1 m error after
the beacon was reached. The yaw sweep cut the worst-case error on those seeds (max under 1 m on all six, against 2.1 m)
and reached the beacon sooner, at slightly lower coverage. On some seeds it was worse (seeds 3 and 4 by mean error),
and with one run per seed the difference is within the noise, so it stays opt-in until a repeat confirms it (the repeat below did not). Frame cost
is unchanged (mean 39–54 ms per 25 × 25 frame on a desktop CPU).

**Corners (later change, `results/corner_nav/`).** The clearance guard used to fire 24–162 times per run at corners,
because pure pursuit cut inside the margin-hugging shortest path. The planner now keeps off walls where there is room,
the lookahead and speed shrink at corners, and a clear arc is searched before stopping. Guard events fell to 1–12 per
run, and the duck reached the beacon on 6/6 seeds against 4/6 on the same `main`, with 93 % coverage against 80 %.
Exploration still does not complete.

Ideas not adopted: a wider normal search while turning was not needed once the ratchet was fixed (the relocalisation
window variant above did nothing). The 0.13 m sensor height is not a problem for `scan_from_points`: at 60° vertical
FOV the lowest zones reach the floor at about 0.23 m, floor returns stay below the 3 cm obstacle threshold with the
simulated 1 % range noise, and the residual error in the dumps was in translation along walls, not in floor/obstacle
classification. On the real A010, range noise and multipath near the floor can be larger; check the floor threshold there.

**100 × 100 vs 25 × 25:** SLAM frame cost on a desktop CPU 16–52 ms (25×25) vs 36–91 ms (100×100) per frame, about 2× more.
On the UNO Q (A53, roughly 5–10× slower than a desktop CPU) 25×25 at about 5–10 Hz is plausible, 100×100 only at 1–3 Hz.
Accuracy was not better at 100×100 in these runs. Binning to 25×25 (or 50×50) is the sensible default.

### Late drift and the yaw sweep (branch `v4-slam-drift`)

**Diagnosis.** Replaying the three runs that ended with 1.4–2.1 m error (`v4snew_s1`, `_s2`, `_s5`; the replay reproduces
0.65 / 0.37 / 0.48 m mean) and reading the per-frame dumps, every big step in the error is an *aliased* match:

- `_s5`, 155 s: facing a wall, the duck relocalises (±0.8 m, weak prior) and takes a 0.6 m jump *along* the wall. The jump
  is "confirmed", because the next relocalisation match sees the same alias in an overlapping sweep window. The frames
  are then mapped at the wrong pose and the error is permanent.
- `_s2`, 236–243 s: normal tracking with 9–11 hits per frame; accepted and partial matches walk the pose 1.1 m in 5 s.
- `_s1`, 126–200 s: standing with 3–5 hits per frame, accepted matches turn the heading by 14–50°.

The match covariance is computed only over ±4 cells (±0.1 m) around the optimum, so a second, separate peak 0.6 m away
does not widen it, and the match looks confident.

**Second-peak test** (`DuckSLAM.match_correlative`, `peak_margin`): after the branch and bound, a second exact branch
and bound finds the best score among candidates that differ from the optimum by more than 0.125 m in a *constrained*
direction (both axes for a full match, the across-wall axis for a partial one, so the expected ridge along a wall does not
count) or by more than 5° in heading. `match_gap` = best score − that score (log-likelihood units, 40 effective points).
In the replays the harmful corrections (error worse by > 5 cm, 15–30 per run) sit almost all at gaps below 1; at 2 and
above they are rare. New `FrameLocalizer` options use it (all default **off**; launch arguments of the same names):

| Option | Effect |
|-|-|
| `peak_margin` | reject any match whose gap is below this |
| `reloc_peak_margin` | the same, only while relocalising (wide window, weak prior) |
| `map_min_gap` | after bootstrap, map only accepted matches with at least this gap |
| `partial_tracks` | a partial (along-wall) match restarts the relocalisation timer |

**Replay** (12 recorded runs: `v4snew_s0`–`5` without and `v4syaw_s0`–`5` with yaw sweep; mean error, runs whose error ever
exceeded 1 m, share of time the motion gate would allow driving). Replays are chaotic (one rejected frame changed one run
from 0.65 to 1.17 m), so only many runs mean anything:

| Policy | Mean error | Runs > 1 m | Motion gate ok |
|-|-|-|-|
| V4 defaults | 0.52 m | 5 / 12 | 58 % |
| `reloc_peak_margin` 1 | **0.40 m** | **4 / 12** | 57 % |
| `map_min_gap` 1.5 + `reloc_peak_margin` 1 | 0.47 m | 6 / 12 | 53 % |
| `map_min_gap` 2 + `reloc_peak_margin` 1 | 0.73 m | 9 / 12 | 49 % |
| `map_min_gap` 2 + `reloc_peak_margin` 1.5 | 0.69 m | 9 / 12 | 49 % |
| `peak_margin` 1 (6 runs without sweep) | 0.34 m | 1 / 6 | 54 % |
| `map_min_gap` 3 + `reloc_peak_margin` 1 + `partial_tracks` (6 runs) | 2.49 m | 6 / 6 | 25 % |

`peak_margin` everywhere lowers the replay error but holds the duck still (closed loop below). Gating the map
(`map_min_gap`) starves it: it stops growing where new views are naturally ambiguous, the duck gets lost or stops
exploring, worst with the sweep. `partial_tracks` was inconsistent (0.42 m together with `reloc_peak_margin`, but one run
lost at 1.95 m with the map gate). Only `reloc_peak_margin` alone helped without costing driving time.

**Closed loop.** 600 s, realistic profile, 25 × 25 zones, seeds 0–5, **two runs per seed and cell** (12 runs per cell);
every run held real time (`sim N s, wall N s` equal in its `launch.log`). Main's runs are `v4snew_*` / `v4syaw_*`
(the `v4-body-slam` runs above) plus `dsw_boff_*` / `dsw_bon_*`; `reloc_peak_margin` runs are `dc2offa_s0`–`2`,
`dc2offr_s3`–`5`, `dc2offb_*` and `dc2on{a,b}_*` (`dc2offa_s3`–`5` lost their bags and were replaced by the reruns
`dc2offr_*`, a rule fixed before those ran; the lost runs had 0.16–0.47 m mean error and reached the beacon 3/3).
Metrics from `result.json` and `tools/trial_metrics.py` on the bags: error near walls = true body centre within 0.25 m
of an obstacle; SLAM ok = share of the time `/slam/state` was "ok" after it first was; recovery = length of the spells
in which it was not. Exploration completion is not tabulated: no run reported it complete within 600 s.
Every run, with all of these values: `results/v4_slam_drift/closed_loop_runs.csv`.

| Policy | Runs | SLAM mean (median) / worst max | Near walls / open | Time > 0.5 m | SLAM ok | Recovery mean / max | Beacon (mean time) | Coverage | Falls |
|-|-|-|-|-|-|-|-|-|-|
| main, sweep off | 12 | 0.42 (0.40) / 2.88 m | 0.44 / 0.37 m | 22 % | 60 % | 2.6 / 69 s | **12/12** (258 s) | 89 % | 2 |
| main, sweep 0.4 rad/s | 12 | 0.38 (0.30) / 3.13 m | 0.34 / 0.36 m | 23 % | 56 % | 3.7 / 355 s | 11/12 (241 s) | 84 % | 3 |
| `reloc_peak_margin` 1, sweep off | 12 | 0.43 (0.31) / 3.09 m | 0.38 / 0.47 m | 18 % | 52 % | 5.0 / 398 s | 9/12 (244 s) | 86 % | 4 |
| `reloc_peak_margin` 1, sweep 0.4 rad/s | 12 | 0.39 (0.27) / **1.92 m** | 0.40 / 0.38 m | 17 % | 52 % | 4.8 / 249 s | 11/12 (220 s) | 84 % | 2 |
| `peak_margin` 1 + `map_min_gap` 2 + `partial_tracks` | 3 | 0.29 (0.18) / 1.44 m | 0.31 / 0.21 m | 21 % | 23 % | 9.3 / 257 s | 1/3 | 63 % | 1 |
| `map_min_gap` 2 + `reloc_peak_margin` 1 | 6 | 0.27 (0.27) / 1.14 m | 0.25 / 0.27 m | 12 % | 42 % | 6.6 / 302 s | 4/6 | 75 % | 2 |

Two-sided Mann–Whitney over the 12 runs per cell, per-run mean error: sweep on vs off p = 0.34 (main) and 0.71
(`reloc_peak_margin`); `reloc_peak_margin` vs main p = 0.51 (sweep off) and 0.75 (sweep on). Per-run *maximum* error:
only `reloc_peak_margin` + sweep vs main without sweep reaches p = 0.03 (one of ten comparisons, so weak evidence).

**Second layout** (`scripts/worldgen.py --layout b`: same room, one 3 m featureless partition parallel to the north wall,
beacon behind a short wall in the south-east corner; one run per seed, seeds 0–2):

| Policy | SLAM mean / worst max | Time > 0.5 m | SLAM ok | Beacon | Coverage |
|-|-|-|-|-|-|
| main, sweep off | 0.39 / 0.89 m | 10 % | 37 % | 0/3 | 51 % |
| main, sweep 0.4 rad/s | 0.51 / 1.49 m | 49 % | 52 % | 1/3 | 69 % |
| `reloc_peak_margin` 1, sweep off | 0.50 / 1.45 m | 36 % | 53 % | 2/3 | 70 % |
| `reloc_peak_margin` 1, sweep 0.4 rad/s | 0.46 / 2.32 m | 32 % | 17 % | 0/3 | 46 % |

This layout is much harder for the navigator (the duck tends to stay north of the partition; 3 of 12 runs reach the
beacon), and the map shows the partition twice, about 15 cm apart: drift along that wall written into the map. With three
runs per cell it decides nothing about the options; the sweep did not help here either.

**Result.**

- **Yaw sweep: stays off by default.** With two runs per seed it makes no measurable difference with either SLAM policy
  (mean 0.38 against 0.42 m with main's, worst case 3.1 against 2.9 m, beacon 11/12 against 12/12, slightly less SLAM
  "ok" time), and none in the second layout. The gain seen with one run per seed (table above) was noise. An adaptive
  sweep (on only when the scan geometry is weak) was therefore not built: the fixed sweep gave no gain for it to keep.
- **`reloc_peak_margin`: stays opt-in (default off).** In replay it removes the confirmed aliased relocalisation jumps
  (0.52 → 0.40 m over 12 bags), but closed loop it is no better than main: same mean error, lower median and less time
  above 0.5 m, but longer "weak" spells (rejected relocalisation matches; 5.0 against 2.6 s), the motion gate open less
  (52 against 60 %), and 9/12 beacons against 12/12. Combined with the sweep, its worst case was the lowest of all
  (1.9 m), which is the one hint worth a further repeat. Enable with `reloc_peak_margin:=1.0`.
- `peak_margin` everywhere, `map_min_gap` and `partial_tracks` stay off: closed loop they stall the duck (SLAM "ok"
  23 %) or starve the map (4/6 beacons). The launch defaults are unchanged from main.
- Frame cost of the peak test (desktop CPU, synthetic 200-point scan): 21.5 → 29.1 ms per normal match, 49.5 → 63.7 ms
  per relocalisation match; with `reloc_peak_margin` only relocalising frames pay it.

**What is left.** Late drift is not solved. Two failure kinds remain in the runs above. (1) Translation along a
featureless wall *while tracking*, where the along-wall direction is weakly constrained and accepted matches walk the
pose (`v4snew_s2` 236 s: 1.1 m in 5 s with 9–11 hits; the doubled partition in layout b). (2) Jumps while the duck stands
closer to an obstacle than the A010's 0.2 m minimum range, so a handful of zones decide the match (`dc2offb_s5`: 0.17 →
1.7 m within 20 s of a weak spell 9–16 cm from a partition end; `v4snew_s1` 126 s with 3–5 hits per frame). Candidates
for the next step: a minimum number of hits in the *newest frame* (not only in the sweep window) before a match may move
the pose; bounding along-wall corrections by the measured motion; keeping raw keyframes so a region can be rebuilt after a
relocalisation (loop closure stays off). None of them is in this branch.

## Files

- `v4_params.json`: all dimensions (mm). Body, boards, A010, battery and XT60 in the torso frame; head parts in the
  lower-head mesh frame. The `head` block also records how the head shells were made from the V3 head.
- `build_v4.py` (Blender): builds the body shell, lid, keel cover and placeholders, cuts the lens opening and the neck
  clearances into the head shells and splits the jaw: `E:\blender\blender.exe --background --python
  design/v4/build_v4.py [-- body|head]`. Sources: `head_upper_v4_whole.stl`, `head_lower_v4_whole.stl`.
- `head_clearance.py`: measures the clearance cutters around the neck mechanism from the generated model and writes
  them to `v4_params.json` (`head.clearance`); rerun it, then `build_v4.py`, if the upstream neck/head parts change.
- Model: `scripts/prep_meshes.py`, then `scripts/make_duck.py` (`--base robot_groundcontact.xml --tag gc`,
  `--torso-scale 1.1 --tag mplus`, `--torso-scale 0.9 --tag mminus`), `scripts/worldgen.py` (and `--layout b`),
  `scripts/sync_ros2_assets.py`, `scripts/mjcf_to_urdf.py`.
- `check_v4_fit.py`: the fit check (writes `fit_check.json`): static fit, pose envelope and recorded motions.
  `python design/v4/check_v4_fit.py --samples 3000 --traj "DIR/*.npz"` after
  `python scripts/motions.py all --record DIR`. `solid.py`: the containment / depth geometry it uses.
- `head_mass.py`: head mass against its budget and the print-setting options.
- `v4_params.json` also holds the keep-out `envelopes` (connectors, cable bends) and the `print` settings.
- `preview.py`: `preview/duck_v4.glb`, `.blend` and renders (`preview/` is git-ignored: upstream meshes).
- `sim_runs/`: `summary.txt`, `result.json`, `map.png`, `launch.log` per run (`v4sbase_*`, `v4snew_*`, `v4syaw_*`: the
  paired trials above).

The simulator launch defaults are this V4 configuration (`ros2 launch duck_nav duck_nav.launch.py sensors:=realistic`);
`tof_zones:=100` for the full A010 array. The old head-mounted VL53L5CX is still available through launch arguments.
