# Floor optical flow odometry: scale fix and net-displacement scoring

Script: `ros2/duck_nav/tools/floor_flow.py` (offline, bags only). Raw numbers: `scores.json` (new default tracker `rect`),
`scores_image.json` (old camera-image tracker, `--method image`, same scoring). All 12 bags, full 600 s, fallen periods skipped.

## 1. The ~4 % under-scale

Confirmed / ruled out (hdg_s3 unless noted; "slope" = regression of flow on true per-pair motion, 1.00 is ideal):

| candidate | evidence | verdict |
|---|---|---|
| pixel-centre convention (+0.5) | MuJoCo render of tiny red floor markers at known positions: marker centres land within 0.3 px of the projection with +0.5; dropping the +0.5 changes the slope by only +1 % (0.969 -> 0.978) | correct as is; not the cause |
| focal length / cx / cy | same render: error < 0.5 % at 70-100 px from the centre; forcing f x 1.04 makes errors worse (median 0.48 -> 1.36 cm) | not the cause |
| LK pulled towards the zero-body-motion guess | seeding LK with the TRUE body motion, or re-seeding with the first estimate (1-3 passes), leaves the slope at 0.961 (vs 0.969); 100 iterations / eps 0.001: 0.969 | not the cause; LK converges to the same fixed point |
| no guess at all | slope 0.81 (large-motion shrink, tile aliasing) | worse |
| image row order / timing | head-only pairs (body still): image motion = 0.985 of the pose prediction, but noise-free MuJoCo renders give 0.999, and body yaw during "still" pairs can explain the rest; shifting the truth interval by +-10 ms changes the slope by 4 %, best fit is at 0 shift for both ends (no evidence of a stamp offset) | could not confirm a timing error |
| non-floor features (c) | restricting feature range: 0.94 (< 0.7 m), 0.97 (0.7-1.0 m), 1.05 (> 1 m), noisy; not conclusive | not confirmed, but see quality below |
| **LK window on a perspective-distorted floor patch** | noise-free MuJoCo renders (known body motion 1-6 cm, tracker seeded with the TRUE motion): image-domain LK under-estimates forward motion by 6-10 % (lateral over by ~3 %); the same frames tracked in a top-down (floor-rectified) view: 0-5 %. Real bags: pair slope 0.88-0.93 (image) vs 0.96-1.00 (rect) on every one of the 12 bags | **confirmed, main cause** |
| **weak features** | the looser the selection the lower the scale on hdg_s3: forward-backward 1.0 px + quality 0.005: 0.91; fb 0.5 / q 0.01: 0.98; fb 0.3 / q 0.02 / min 6 points: 0.999. Ambiguous (edge, noise) corners bias towards zero motion | **confirmed, secondary**; costs coverage |

Fix (`track_rect`, the default): warp both images onto the floor plane of their own level frame (8 mm per pixel, from
`/camera/pose_level`), track corners there (LK 11 px window, 2 pyramid levels, forward-backward < 0.3 px, quality 0.02,
displacement gate 12 cm because the checker repeats), RANSAC rigid fit on the floor coordinates. Pair slope averaged over
the 12 bags: 0.986 (rect) vs 0.92 (image). Residual -1 % to -4 % on single bags (0.962 hdg_s5). I could NOT prove the
remaining ~1 % is not a sim timing effect; MuJoCo renders show no such error.

Price: coverage (share of pairs with a fit) drops to 0.54 mean (0.39-0.82), against 0.74-0.87 for the image tracker on
the same bags (the image tracker also admits wall and box points, which is part of its bias). An `image` and a `warp`
(homography-compensated image tracker, scale 0.93, no better) method are kept in the file for comparison.

## 2. Scoring (docstring of `floor_flow.py` has the exact definitions)

Everything is integrated with the TRUE heading. Net displacement over non-overlapping 2 s and 10 s windows inside runs
of consecutive pairs, error = |estimated net - true net| in the frame at the window start. "moving" windows: true net
speed > 0.02 m/s; scale = sum(est . true) / sum(true . true); err % = sum(error) / sum(|true net|). "pushing" windows:
command predicts > 0.03 m/s net motion but the robot achieved < 25 % of it. Flow with no fit counts as zero motion;
`flow-or-cmd` falls back to the command for those pairs (the usable estimator). The old `command_scale` 0.71 was an
artefact of the sway-inflated denominator: over net displacement the command OVER-estimates by 5-8 % on every bag
(scale 1.05-1.08).

## 3. Results, 12 bags x 600 s

Median window error in cm (all windows, including standing still; tilt bags are mostly still, hence tiny command errors):

| bag | pairs | cov | ms/pair | 2 s cmd | 2 s flow | 2 s f/c | 10 s cmd | 10 s flow | 10 s f/c | pair scale rect (img) | pushing 2 s (n): cmd / flow cm | pushing 10 s (n): cmd / flow cm |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hdg_s0 | 2997 | 0.69 | 24.0 | 0.2 | 1.5 | 1.4 | 2.8 | 3.7 | 3.2 | 0.976 (0.930) | 1: 12.2 / 2.9 | 0 |
| tilt_s0 | 3000 | 0.82 | 27.5 | 0.1 | 0.8 | 0.8 | 0.1 | 3.1 | 2.8 | 1.001 (0.914) | 0 | 0 |
| hdg_s1 | 1640 | 0.39 | 18.9 | 0.9 | 1.2 | 1.2 | 2.5 | 4.2 | 2.7 | 0.992 (0.932) | 0 | 0 |
| tilt_s1 | 3000 | 0.72 | 24.2 | 0.1 | 0.7 | 0.7 | 0.1 | 2.6 | 2.4 | 0.991 (0.878) | 0 | 0 |
| hdg_s2 | 3000 | 0.55 | 17.8 | 0.9 | 1.5 | 1.3 | 4.4 | 10.2 | 4.3 | 0.972 (0.932) | 1: 6.8 / 1.5 | 0 |
| tilt_s2 | 2070 | 0.53 | 21.2 | 0.8 | 1.5 | 1.3 | 2.5 | 6.8 | 4.6 | 0.983 (0.924) | 0 | 0 |
| hdg_s3 | 1451 | 0.39 | 19.7 | 1.3 | 3.5 | 1.1 | 5.5 | 25.7 | 4.8 | 0.999 (0.928) | 0 | 0 |
| tilt_s3 | 3000 | 0.44 | 20.0 | 0.1 | 0.7 | 0.7 | 0.1 | 2.1 | 1.7 | 0.977 (0.917) | 0 | 0 |
| hdg_s4 | 1592 | 0.46 | 14.0 | 1.7 | 2.1 | 1.4 | 4.6 | 31.8 | 2.8 | 0.989 (0.938) | 0 | 0 |
| tilt_s4 | 1114 | 0.50 | 13.6 | 2.3 | 6.6 | 2.3 | 5.6 | 33.0 | 6.3 | 0.994 (0.908) | 1: 25.4 / 0.8 | 0 |
| hdg_s5 | 858 | 0.49 | 16.1 | 2.1 | 7.1 | 2.1 | 4.7 | 31.2 | 5.2 | 0.962 (0.918) | 0 | 0 |
| tilt_s5 | 814 | 0.49 | 15.7 | 2.8 | 5.5 | 2.5 | 9.3 | 33.3 | 8.6 | 0.991 (0.931) | 4: 23.5 / 3.6 | 2: 62.2 / 8.7 |

Moving 10 s windows, relative error (sum error / sum true net distance):

| bag | moving windows | cmd % | flow % | flow-or-cmd % | cmd scale | flow-or-cmd scale | flow heading err deg | pairs err > 3 cm % |
|---|---|---|---|---|---|---|---|---|
| hdg_s0 | 11 | 10.7 | 57.4 | 8.6 | 1.06 | 1.00 | 0.13 | 0.3 |
| tilt_s0 | 9 | 10.6 | 76.1 | 10.5 | 1.07 | 1.04 | 0.12 | 0.4 |
| hdg_s1 | 12 | 11.9 | 62.1 | 10.1 | 1.06 | 1.04 | 0.15 | 0.5 |
| tilt_s1 | 9 | 10.4 | 73.9 | 11.5 | 1.07 | 1.03 | 0.13 | 0.0 |
| hdg_s2 | 28 | 11.3 | 59.9 | 10.9 | 1.07 | 1.06 | 0.14 | 0.2 |
| tilt_s2 | 15 | 16.3 | 67.4 | 11.3 | 1.07 | 1.04 | 0.15 | 0.4 |
| hdg_s3 | 19 | 10.5 | 61.0 | 8.2 | 1.06 | 1.03 | 0.14 | 0.4 |
| tilt_s3 | 14 | 11.2 | 68.9 | 8.2 | 1.07 | 1.02 | 0.14 | 0.3 |
| hdg_s4 | 20 | 11.5 | 59.8 | 8.2 | 1.06 | 1.03 | 0.16 | 0.4 |
| tilt_s4 | 16 | 12.1 | 69.4 | 10.6 | 1.08 | 1.05 | 0.13 | 0.2 |
| hdg_s5 | 11 | 13.6 | 67.4 | 11.4 | 1.05 | 1.05 | 0.13 | 0.0 |
| tilt_s5 | 12 | 12.7 | 58.5 | 12.0 | 1.07 | 1.07 | 0.13 | 0.5 |

Means over bags (moving windows, relative error): 2 s: command 17.4 %, flow-or-cmd 16.4 %; 10 s: command 11.9 %, flow-or-cmd
10.1 %. Pure flow (no fallback) is 65 % because half of the pairs have no fit and count as zero motion: it cannot replace the
command on its own.

Headlines
- When flow has a fit it is accurate (0.4 % of pairs off by > 3 cm, heading 0.13 deg median, pair scale 0.99) but over a
  window the net displacement is only as good as the worst missing pair, so flow-or-command is only 1-2 points better than
  the command over 10 s in moving windows. Pushing windows (n = 8 at 2 s, 2 at 10 s, all on 3 bags) are where flow clearly
  wins (cm: 12 -> 3, 25 -> 0.8, 24 -> 4, 62 -> 9).
- Pushing windows are rare and the 10 s counts are small; treat these as indicative.

## 4. Runtime

Wall time per image pair of the flow step only (rectify both images, features, two LK runs, RANSAC), Python + OpenCV 4.6
on this PC under WSL: 13-28 ms per pair, mean 19 ms. These figures come from running the rect and image jobs
concurrently, so they are inflated; run alone on hdg_s3 60 s the rect tracker took 13 ms and the old image tracker 11 ms.
Excludes bag reading and the command integration.

## Not verified
- The remaining ~1 % under-scale (mean pair scale 0.986) and its source; timing offset in the sim not excluded, only not seen.
- Coverage vs scale trade-off was tuned on hdg_s3 only; looser settings recover coverage at a few percent of scale.
- Real-robot behaviour (no depth pose noise, different floor texture) is untested.
