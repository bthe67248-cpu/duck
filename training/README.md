# Training a walking policy for the Duckling V4 (CPU, no GPU needed)

The pretrained Microduck policies were trained for the upstream robot. On the V4 duck (heavier head, body shell,
LiPo, jaw), the pretrained `velstand` fails the walk, plank and backward get-up tests (README Part D,
`results/v4_policy/`). This folder retrains it on the V4 model.

| File | What it does |
|-|-|
| `duck_env.py` | MuJoCo environment on `model/duck_v4_gc_scene.xml` (the ground-contact model with the head, jaw and body collision proxies). Same policy contract as `scripts/motions.py` and `world_sim`: 50 Hz, 61-number observation, 14 joint targets = STAND + action |
| `train_ppo.py` | PPO on the CPU with one worker process per core. Writes a resumable checkpoint, a CSV log and `policy.onnx` (normalizer baked in, input `obs` [1, 61] → `actions` [1, 14]) |

## Why it is built this way

- **Same contract as the deployed policies.** The output ONNX drops into `motions.py --walk-policy`, into
  `world_sim`, and into the upstream runtime's hot-swap slot, with no adapter.
- **Warm start.** `--init-onnx policies/velstand.onnx` copies the pretrained network and its normalizer (verified
  to reproduce velstand to 4e-4 rad), and the normalizer then stays fixed (`--freeze-normalizer` is implied), so the
  pretrained actor's input scaling cannot drift under it. A fresh critic is trained first, for `--critic-warmup`
  iterations with the actor frozen and the learning rate held. Without that, the first actor updates are driven by
  a random critic, and in a test run they took the fall rate from 2 % to 75 % within ten iterations.
- **Same actuation as deployment.** The servo delay is the same transport delay as
  `duck_nav.actuation.ServoDelay` (checked step for step), physics 0.005 s × 4 per action, and the actions are not
  filtered.
- **Unknown servos.** The Feetech servos have not been bench-tested yet, so the domain randomization around the
  model's servo numbers is wide: stiffness ±25 %, torque limit ±20 %, command delay ×0.5–2 of the model's
  `servo_delay_s` (20 ms estimate: 10–40 ms), joint friction and damping ×0.5–2, armature ±20 %. (The position
  actuators have no velocity gain, so servo damping is randomized through joint damping.) It is re-applied from
  stored defaults at every reset, so it never accumulates. Once `scripts/make_duck.py` has been run with a
  measured calibration file, the stiffness, torque and delay ranges centre on the measured values. Narrow them with
  `--cfg '{"kp": [0.9, 1.1], "delay_scale": [0.8, 1.25]}'`. In PowerShell, write the JSON as
  `--cfg '{\"kp\": [0.9, 1.1]}'`, or put it in a file and pass `--cfg (Get-Content cfg.json -Raw)`.
- **Also randomized:** every body's mass ±10 %, trunk and head centre of mass ±8 mm, foot friction 0.5–1.2, IMU
  mounting tilt up to 5°, joint encoder bias ±0.015 rad, observation noise (upstream values) and velocity pushes
  every 3–6 s (ramped from 0.1 to 0.3 m/s over training).
- **Fall recovery.** 15 % of episodes start lying in a random attitude. In those, a fall does not end the episode
  until the duck has stood for 1 s. Getting up pays by potential (progress in up-axis and height), so lying still
  earns nothing. All task rewards are gated on being upright: lying still with a zero command would otherwise
  collect the velocity and pose rewards.
- **Rewards** follow upstream's velocity recipe (velocity tracking, upright, leg pose with tighter limits when
  standing, foot air time and clearance, slip, smoothness, joint-limit proximity). Two are added: a self-collision
  penalty for legs touching the body or head proxies (the V4 shell is what made the pretrained walk fail), and a
  head-pose term on a 1 s average (upstream lesson: price the head's DC error, not its unavoidable oscillation).
  The smoothness weight ramps from 0.1 to 0.5 over the first half of training. Upstream introduces it only after
  the gait exists. Episodes start at random points of their time limit so time-outs do not all land together.
- **Commands** cover what deployment sends (nav_node sweeps the head ±0.9 rad and turns at 1.2 rad/s; the dance
  uses head 0.8, turn 1.5, body roll 0.15 and pitch 0.12). Twist: 20 % exactly zero (the idle state must be
  trained explicitly), 10 % turn-in-place, otherwise vx −0.2…0.4 m/s, vy ±0.15 m/s, yaw rate ±1.5 rad/s, with small
  commands snapped to zero. Head pose ±0.8 rad in 70 % of command periods. Body pose: roll ±0.15 and pitch ±0.12
  (tracked: the upright term aims at the commanded tilt) and height −15…0 mm (not tracked) in 40 %; x, y and yaw
  stay tiny.
- **Tracking widths** are tighter than upstream (velocity σ² 0.02, yaw rate 0.05, given with `--cfg`). With the
  upstream widths, a 0.2 rad/s yaw-rate error still scored 85 %, and the policy drifted 90° over the 8 s walk test.
- **Joint limits:** proximity penalty within 8 % of each joint's range, weight 20. `motions.py balance` requires
  0.02 rad of margin, and a weaker penalty let one joint sit on its limit.
- **Falls** use the same proxies and 3-step persistence as `motions.py`'s fall monitor.

## Run it

```bash
python -m pip install -r requirements-test.txt torch onnx     # once (torch CPU is enough)
# fine-tune from the pretrained policy (recommended):
python training/train_ppo.py --out training/runs/walk_v4_ft --init-onnx policies/velstand.onnx \
    --critic-warmup 40 --lr 3e-5 --lr-max 1e-4 --lr-min 1e-5 --target-kl 0.005 \
    --iters 2500 --workers 4 --envs-per-worker 48 --steps 48 --cfg '{"s_ang": 0.05, "s_lin": 0.02}'
# export any checkpoint to ONNX (to evaluate it while training runs):
python training/export_checkpoint.py training/runs/walk_v4_ft/last.pt --out candidate.onnx
# from scratch (needs far more steps; give it a big machine):
python training/train_ppo.py --out training/runs/walk_v4 --iters 20000 --workers 16
# resume either one:
python training/train_ppo.py --out training/runs/walk_v4_ft --resume training/runs/walk_v4_ft/last.pt ...
```

The log (`log.csv`, and every 10th iteration on screen) shows: iteration, env steps, mean step reward, mean
episode length (s), fall rate per episode, learning rate, action noise std, KL, steps/s, and every reward term
(each penalty must stay ≤ 0). On 4 cores it runs at about 3 000–4 000 steps/s, or 12M steps an hour. More cores
scale it almost linearly (`--workers`).

`last.pt` is saved every 25 iterations. Choose a policy by evaluating it, not by the training reward: the reward
moves with the curriculum.

## Evaluate and install

```bash
python scripts/motions.py all --walk-policy training/runs/walk_v4_ft/policy.onnx --out results/v4_policy/candidate
```

Only if it passes at least what velstand passes, copy it to `policies/duck_v4_walk.onnx` and use it in
`world_sim` (`ros2 launch duck_nav duck_nav.launch.py walk_policy:=duck_v4_walk.onnx`, see the main README).

## Before it goes on the real duck

1. Weigh the parts and links, bench-test one servo (TODO.md steps 2–3) and rebuild the model with the
   calibration file.
2. Fine-tune again from the current policy (`--init-onnx policies/duck_v4_walk.onnx`) on the measured model.
3. First runs on the robot: on a harness or held by hand, zero command first, then 0.1 m/s.
4. Replay the same `/cmd_vel` in `world_sim` and on the robot. If they differ, fix the model before the gains.
