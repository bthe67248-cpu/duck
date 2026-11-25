# Feetech HD1910M servo data (for the actuator model and policy retraining)

Kept for later: these numbers go into the simulated actuator model before the walking policy is retrained or
fine-tuned for V4. **Not yet verified.** They come from a summary of the vendor data, not the official Feetech
datasheet, and nothing here has been measured on the duck's own servos. Confirm them against the datasheet, then
measure on the bench (below).

## Part

- Feetech HD1910M (model code HD-1910-C001): TTL serial bus servo intended for small bipeds such as the Microduck.
- Metal gear train, 1:320 reduction. Gear backlash ≤ 0.5°.
- 12-bit magnetic encoder: 360° / 4096 = 0.088° per step.
- Half-duplex TTL serial up to 1 Mbps; position updates up to 1 kHz (1 ms minimum command period).
- Coreless DC motor; PID gains are configurable, or pure PD mode (the mode an RL policy and its sim model should use).
- No fixed step-response time or latency is published: both depend on the gains and the bus setup. Distributors
  provide step-response and friction data on request (e.g. for BAM-style actuator identification).

## Torque and speed

Torque constant K_T = 7.5 kg·cm/A = 0.735 N·m/A.

| Voltage | No-load speed | Rated torque | Rated current | Stall torque | Stall current |
|-|-|-|-|-|-|
| 4.8 V | 0.137 s/60° (72 RPM, 7.5 rad/s) | 2.2 kg·cm (0.22 N·m) | 500 mA | 9.0 kg·cm (0.88 N·m) | 1.2 A |
| 6.0 V | 0.109 s/60° (92 RPM, 9.6 rad/s) | 3.0 kg·cm (0.29 N·m) | 690 mA | 12.0 kg·cm (1.18 N·m) | 1.6 A |
| 7.4 V | 0.088 s/60° (113 RPM, 11.8 rad/s) | 3.7 kg·cm (0.36 N·m) | 900 mA | 15.0 kg·cm (1.47 N·m) | 2.0 A |

Idle current 20 mA; no-load running current 160 mA (4.8 V) to 210 mA (7.4 V). The table is internally consistent:
stall torque / stall current ≈ 0.73–0.74 N·m/A in every row, and the s/60° figures match the RPM figures.

## For the simulation

Linear torque–speed limit (coreless motor):

    tau_max(omega) = tau_stall * (1 - |omega| / omega_no_load)

| Supply | tau_stall | omega_no_load | Equivalent back-EMF damping tau_stall / omega_no_load |
|-|-|-|-|
| 4.8 V | 0.88 N·m | 7.54 rad/s | 0.117 N·m·s/rad |
| 6.0 V | 1.18 N·m | 9.63 rad/s | 0.122 N·m·s/rad |
| 7.4 V | 1.47 N·m | 11.8 rad/s | 0.124 N·m·s/rad |

- The V4 duck runs from a 2S LiPo: 7.4 V nominal, 8.4 V full, about 6.6 V near empty. Use the 7.4 V row as the
  default and randomise between the 6.0 V and 7.4 V rows during training. Above 7.4 V (full pack) is an
  extrapolation: check the servo's rated maximum voltage before running it from a full 2S pack without regulation.
- Backlash ≤ 0.5° and encoder quantisation 0.088° can be modelled as a dead band and position noise.
- For comparison, the upstream Microduck policies were trained for Dynamixel XL330 servos (about 0.5 N·m stall at
  5 V), so the HD1910M is roughly 2.5–3× stronger at stall. The pretrained policies' action scaling and PD gains will
  not transfer unchanged.

## Still to measure on the bench (before retraining)

1. Step response in PD mode at the gains the policy will use: rise time, overshoot, settling time.
2. End-to-end latency: command sent → motion starts, including the bus at the real baud rate and servo count.
3. Backlash and friction (static and viscous), loaded and unloaded.
4. Torque–speed check at the real supply voltage, at least the stall torque and the no-load speed.
5. Weigh the printed V4 parts and the servos for the mass ledger.

## Training hardware note

The upstream training code (`microduck_rl`, mjlab) runs physics on MuJoCo Warp, which needs an NVIDIA GPU (CUDA). It
does not run on an AMD GPU as is; use a rented NVIDIA GPU, or port the environment to a backend that supports the
available GPU.
