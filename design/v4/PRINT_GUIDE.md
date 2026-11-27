# Duckling V4: printing and assembling the printed parts

This covers the six V4 parts this repository designs. The legs, hips, neck, trunk plate and everything else
come from upstream Pollen Robotics Microduck CAD ([pollen-robotics/microduck](https://github.com/pollen-robotics/microduck)).
Print those from upstream with its own instructions.

Before you print, regenerate and check everything (Part A of the main README), then:

```bash
python design/v4/export_print.py          # checks every part and writes design/v4/print/*.stl + print_report.json
```

It exits with status 1 if any part is not a closed solid, does not fit a 220 × 220 × 250 mm bed (change this
with `--bed X Y Z`), or has its 5th-percentile wall below 0.6 mm.

## 1. Parts and checks (current geometry)

| File in `design/v4/print/` | What it is | Size (mm, as oriented) | Volume | Walls (5th pct / median) | Check |
|-|-|-|-|-|-|
| `head_upper_v4.stl` | dome + upper bill, M12 lens opening, mouth-servo cradle, 4 pilot holes | 98 × 50 × 122 | 61.1 cm³ | 1.49 / 3.11 | ok |
| `head_lower_v4.stl` | lower shell: neck clearance, jaw hinge ears, 4 clearance holes | 95 × 22 × 121 | 38.9 cm³ | 1.11 / 3.0 | ok |
| `jaw_v4.stl` | jaw (lower bill), hinge bosses, pushrod lever | 76 × 17 × 31 | 4.8 cm³ | 0.64 / 1.96 | **warn**: thin areas |
| `body_shell_v4.stl` | body: board bay, battery compartment, connector well, A010 pocket, trunk mounts | 136 × 69 × 89 | 68.9 cm³ | 1.53 / 2.0 | ok |
| `body_lid_v4.stl` | rear lid of the board bay | 85 × 60 × 5 | 7.5 cm³ | 1.5 / 1.8 | ok |
| `keel_cover_v4.stl` | bottom cover of the connector well, LiPo rails | 33 × 30 × 20 | 3.2 cm³ | 2.0 / 2.0 | ok |

All six are closed solids. The jaw's thin areas are along the top lip of the bill and the back of the heel. They
come from the upstream shell (the original jaw had them too: 5.9 % of the surface under 0.8 mm, 6.2 % now).
Turn on variable-width walls (Arachne: PrusaSlicer 2.5+, Cura 5+, OrcaSlicer, Bambu Studio) and look at those
areas in the slicer preview.

## 2. Slicer settings

These are the settings the simulated masses assume (`v4_params.json` → `print`). If you change them, weigh the
parts and put the weights in the calibration file (step 5).

| Setting | Value |
|-|-|
| Material | black PLA (PETG works; it is heavier: re-weigh) |
| Nozzle / layer | 0.4 mm / 0.16–0.2 mm (0.12 mm for the jaw if your slicer struggles with its lip) |
| Walls | 3 perimeters (1.2 mm) |
| Top / bottom | 4 / 4 layers |
| Infill | 30 % gyroid |
| Wall generator | Arachne (variable width) |
| Supports | tree supports, on the build plate only where possible; see the orientation notes |

## 3. Orientation

`export_print.py` writes each STL already oriented: of the six axis-aligned orientations, it picks the one with
the least area overhanging more than 45°. These are suggestions, so check them in your slicer:

- **Head upper:** exported standing on its back (bill up). The usual alternative is open side down, dome up. That
  needs support inside the dome but gives the outside of the dome a cleaner finish. Either works.
- **Head lower:** exported standing on its back. Open side up (bowl on the plate) also works, with supports under
  the curved bottom.
- **Jaw:** on its side. The hinge bosses are then circles in the layer plane, so the pin holes come out round.
- **Body shell:** upside down (roof on the plate). The board bay, battery compartment and well open upwards.
- **Lid** flat; **keel cover** as modelled (rails up).

After printing, clear every hole with a drill bit of the listed size. Printed holes come out a little small.

## 4. Hardware

| Qty | Part | Where |
|-|-|-|
| 4 | M2 × 12 pan-head self-tapping screw + M2 washer | head shells, rear bosses (from below, into Ø1.75 pilots in the upper shell) |
| 2 | M2 × 10 pan-head self-tapping screw + M2 washer | head shells, front bosses (head on the hinge web) |
| 2 | M2 × 10 pan-head screw (or a Ø2 mm steel pin, glued in the ear) | jaw hinge pins, one per side: through the jaw boss (Ø2.3 clearance, the jaw turns on the shank) into the shell ear (Ø1.75 pilot) |
| 1 | 2 mm rod + 2 clevises/Z-bends, about 35 mm | mouth pushrod: servo horn → jaw lever pin (Ø2 hole in the jaw lever plate) |
| 1 | 2.5 mm cable tie | mouth servo strap, through the two slots in the cradle plates |
| 4 | M2.5 heat-set insert + M2.5 × 6 screw | UNO Q standoffs in the body bulkhead |
| 12 | M2 heat-set insert + M2 × 6 screw | body: lid (2), keel cover (4), trunk plate (2), hip-yaw XL330 bosses (2), spare (2) |
| 1 | panel-mount XT60 female + 2 screws | connector well rear wall |
| — | foam pads | LiPo, on the compartment walls and roof |

## 5. Assembly order

1. **Head.** Put the mouth servo into the cradle in the dome from below/+x (horn on the +x face, cable plugs to
   the rear through the plate window) and strap it with the cable tie. Fit the pushrod to the horn.
2. Put the jaw into the lower shell, line up the hinge bosses with the ears, and drive one M2 × 10 through each
   boss into its ear. **Check that the jaw swings 0–30° freely** before going on. Clip the pushrod to the jaw lever.
3. Route the head servo bus, the mouth servo cable and the camera cable down through the neck opening, with enough
   slack for head yaw ±170° and full pitch/roll. Mount the head on the neck.
4. Close the shells: the upper onto the lower, 2 × M2 × 10 (front, through the hinge webs) and 2 × M2 × 12
   (rear) from below with washers. Do not over-tighten the self-tappers in PLA.
5. **Body.** Press the heat-set inserts in, then put the UNO Q + HAT stack on the standoffs through the open rear
   and close the lid (2 hooks + 2 × M2). Put the LiPo into its compartment from below, plug its XT60 into the panel
   female, and close the keel cover (4 × M2). Drop the shell over the trunk (neck through the notch) and bolt it to
   the trunk plate (2 × M2) and to the hip-yaw servos (2 × M2).
6. **Weigh** every printed part and every complete link and put the numbers in a copy of
   `config/robot_measurements.example.json`. Then rebuild the model and re-run the checks and the policy
   training (main README, "After printing").

## 6. What the checks cover, and what they do not

- `design/v4/check_v4_fit.py`: mesh-against-mesh fit of every part at rest, over 3000 sampled poses and over
  recorded motions. Result for this geometry: no static clash except the known 0.9 mm contact face of the upstream
  motor support. The screw-head envelopes sit flush under the bosses. The new hinge ears, webs, bosses and cradle
  are clear of the neck mechanism, the camera and the jaw over its whole travel. The only pose-envelope clashes
  left are the upstream legs touching each other in extreme crossed-leg samples.
- `scripts/check_contacts.py`: the simulation's collision proxies (convex hulls of the head shells and jaw, and
  chamfered blocks for the body). Gate: no proxy contact deeper than 0.8 mm in 6000 sampled operating poses.
  PASS. Four grazing contacts under 0.5 mm remain at the corners of that envelope.
- **Not checked:** the real servo cases, horn and clevis sizes, cable diameters, the UNO Q's bottom-side
  connector height (the standoffs assume ≤ 5 mm), and the screw lengths against the real boss depths. These are
  estimates. Do a test fit of the head first: it is the cheapest print and has the most new mechanism.
