"""head_mass.py -- V4 head mass against its budget, and what print settings would save (no model build needed).

    python design/v4/head_mass.py          (reads model/mass_ledger.json and model/v4_meshes.json)

Budget: the upstream Microduck head link (188.8 g in the ledger), which the walking policies were trained with and
the neck servos carry. The V4 head = that link with the upstream shells, mouth, face plate and head boards replaced
by the V4 shells and the mouth servo, plus the separate jaw body. Printed masses use make_duck.py's wall-proxy
formula: V_wall = min(V, A * wall); m = (V_wall + infill * (V - V_wall)) * 0.00124 g/mm3. The shells are solid
meshes about 3 mm thick on average (2 V / A), so with 1.2 mm perimeters on both faces they are almost all wall:
fewer perimeters save mass, infill barely matters, thinner shell geometry saves the most.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RHO = 0.00124
SHELLS = ("head_upper_v4", "head_lower_v4", "jaw_v4")


def mass_g(v, a, wall, infill):
    vw = min(v, a * wall)
    return (vw + infill * (v - vw)) * RHO


def thinned(v, a, t_new):
    """Volume of the shell offset to a mean thickness t_new (area kept): V' = A / 2 * t_new, never more than V."""
    return min(v, a / 2 * t_new)


def main():
    led = json.loads((ROOT / "model/mass_ledger.json").read_text())["parts"]
    mesh = json.loads((ROOT / "model/v4_meshes.json").read_text())
    head = led["_head_link"]
    budget = head["upstream_mass_g"]
    jaw = led["jaw_v4"]["mass_g"]
    total = head["new_mass_g"] + jaw
    shells = sum(led[n]["mass_g"] for n in SHELLS)
    print(f"head budget (upstream head link): {budget:.1f} g")
    print(f"V4 head link {head['new_mass_g']:.1f} g + jaw {jaw:.1f} g = {total:.1f} g "
          f"({total - budget:+.1f} g, {100 * (total / budget - 1):+.1f} %)")
    print(f"  shells + jaw {shells:.1f} g, mouth servo {led['mouth_servo_v4']['mass_g']:.1f} g, "
          f"upstream parts they replace {sum(head['removed_upstream_parts_g_ESTIMATE'].values()):.1f} g (solid-PLA estimate)")
    for n in SHELLS:
        v, a = mesh[n]["volume_mm3"], mesh[n]["area_mm2"]
        print(f"  {n}: {v / 1000:.1f} cm3, {a / 100:.0f} cm2, mean thickness {2 * v / a:.2f} mm, {led[n]['mass_g']:.1f} g")
    print("\nshells + jaw (g) by print setting (wall mm per face x infill):")
    infills = (0.10, 0.15, 0.20, 0.30)
    print("  wall  " + "  ".join(f"{i:>5.0%}" for i in infills))
    for wall in (0.8, 1.0, 1.2):
        row = [sum(mass_g(mesh[n]["volume_mm3"], mesh[n]["area_mm2"], wall, i) for n in SHELLS) for i in infills]
        print(f"  {wall:4.1f}  " + "  ".join(f"{g:5.1f}" for g in row))
    print("\nshells + jaw (g) with the shell geometry thinned to a mean thickness t (needs a CAD offset):")
    for t in (2.5, 2.0, 1.6):
        for wall, infill in ((1.2, 0.30), (0.8, 0.15)):
            g = sum(mass_g(thinned(mesh[n]["volume_mm3"], mesh[n]["area_mm2"], t), mesh[n]["area_mm2"], wall, infill)
                    for n in SHELLS)
            print(f"  t {t:.1f} mm, wall {wall} mm, infill {infill:.0%}: {g:5.1f} g ({g - shells:+.1f} g)")


if __name__ == "__main__":
    main()
