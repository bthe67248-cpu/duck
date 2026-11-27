# Status of the ff34442 review handoff

Branch `review-ff34442-fixes`, based on the reviewed merge `ff344423aa63857a1d9f88e44165c6f49dc5d267`
(27 November 2026). This file says, item by item, what this branch did and what is still open, with the evidence.
"Open (hardware)" means the item needs the physical robot, bench measurements or bought parts. That is not a
finding that anything is defective.

## Verification on this branch

| Check | Result |
|-|-|
| Fresh clone, `pip install -r requirements-test.txt` only (the CI path), pinned upstream `microduck_rl`, no pre-existing model assets | **229 passed, 0 failed**, 6 skipped (4 OpenCV, 1 Torch, 1 MuJoCo < 3.4). The review recorded 216 passed / 3 failed. |
| Same clone plus `requirements-vision.txt` | `ros2/duck_nav/test`: 208 passed; the only skip is the MuJoCo-version one. The OpenCV tests run. |
| Training export / warm start (`tests/test_training.py`, with Torch and ONNX) | 4 passed. The CI job uses the PyTorch CPU index (this sandbox cannot reach its download host, so it was run in the main environment). |
| Compiled ground-contact model SHA-256 | `33e41244c8edb2fcfd7cb4550f30c9275c91c27fc07a3cb8502d099eb7884d85`, the same as the contact report in the merge. No geometry changed. |
| Contact gate, mesh fit, print checks | Not re-run. The geometry is identical (hash above), so the committed reports still apply. |

## A. Code and test fixes: all done

| ID | Done | Evidence |
|-|-|-|
| A01 sensor QoS | `slam_node` subscribes to all sensor topics (imu, camera_info, depth, tof/zones, image_raw, both pose_level) best-effort by default. A best-effort subscription accepts both best-effort and reliable publishers. The new parameter `sensor_qos:=best_effort\|reliable` overrides this. Commands and detections stay reliable. IMU history depth stays 50. | `test/test_sensor_io.py::test_sensor_subscriptions_are_best_effort_by_default` runs the real `SlamNode.__init__` against a recording fake Node, in both sensor modes. The real-ROS delivery test is B03 (open). |
| A02 A010 eviction race | The `full()`/`get_nowait()` pair is replaced by a bounded, non-blocking put loop that tolerates the consumer draining the queue in between. Drops are still counted and the newest frame wins. | `test_sensor_contract.py::test_a010_queue_eviction_survives_consumer_draining_in_between` (deterministic interleaving). The old code path raises `queue.Empty` under the same interleaving (reproduced). |
| A03 camera watchdog | New `duck_nav/capture_supervisor.py`. Each device handle belongs to one worker thread. On stale, the worker is abandoned (it releases its own handle if its read ever returns, and its late frames are dropped) and a new worker reopens the device. At most two may still be blocked before the status becomes `stuck`. `close()` is bounded. `usb_camera_node` uses it. | `test_sensor_io.py`: blocked read, read failure, unplug/replug, bounded close with permanently blocked reads (5 repeated runs, no flakes). The real V4L2 behaviour on the UNO Q is H02/H10 (open). |
| A04 image stride | New `duck_nav/image_codec.py`: rgb8/bgr8/mono8 decoding that honours `step` and rejects malformed buffers. `slam_node` floor flow and `yolo_node` both use it, and a bad frame is dropped with a warning. | `test_sensor_io.py`: contiguous vs padded (0/1/6 bytes) are identical for all three encodings. The review's 5×2 RGB8 `step=16` case decodes. Malformed cases raise `ImageDecodeError`. The real `on_image` accepts padded rows and survives a truncated buffer. Grey matches OpenCV. |
| A05 rtree | The chamfer test now uses a face-plane test on the hull, with no optional dependency. `rtree` and `embreex` are listed in `requirements-tools.txt` for the print and proxy tools. | Fresh-clone run above. |
| A06 training tests | New `tests/conftest.py` with a shared builder: models are generated into a temporary root from the pinned upstream source. The training tests use the session fixture `gc_scene`, and `test_model_calibration.py` reuses the builder. | Fresh-clone run above: both environment tests run and pass. |

## B. Coverage and reproducibility

| ID | Status | What |
|-|-|-|
| B01 OpenCV in CI | Done | CI job `vision-tests` installs `requirements-vision.txt` and fails if any test is skipped for a missing cv2. |
| B02 Torch/ONNX in CI | Done | CI job `training-export-tests` (CPU PyTorch) builds the model and fails if the export test is skipped. Training dependencies stay out of `requirements-hw.txt`. |
| B03 real ROS job | **Open** | Needs a ROS 2 Jazzy installation. In this sandbox, apt could not use `packages.ros.org` (its HTTPS certificate names `*.osuosl.org`), and a RoboStack/conda install was not permitted. Next step: a CI job on `ros:jazzy` (Docker) that builds `duck_nav` with colcon and publishes IMU/ToF/image best-effort and reliable into `slam_node`, checking delivery, exact-stamp pose matching and teardown. |
| B04 one fresh-checkout procedure | Partly | The README quick start now lists the requirement sets. The full chain (URDF export, asset sync, colcon) still needs ROS (B03). |
| B05 hardware extras | Done | `requirements-hw.txt` (pyserial, OpenCV, onnxruntime, numpy/scipy/yaml for the robot), `requirements-vision.txt`, `requirements-tools.txt`, `requirements-train.txt`. |
| B06 provenance | Done for policy evaluations | `training/evaluate.py` writes commit, dirty state, the compiled-model SHA-256 (covers included XML and meshes), robot mass, policy SHA-256s, versions, the command and the pacing note, next to the table (`eval.md` + `eval.json`). Navigation trials already record their own run metadata. |

## C. Walking policy and balance: open

Nothing in C was changed on this branch. The installed `duck_v4_walk.onnx` is unchanged (SHA-256 `04a4db00…`,
9/12: the 8 nominal motions pass; stiffness ×0.8, ×1.2 and 10 ms delay walking fail).
- **C01–C04:** need more policy training. On 4 CPU cores, training past iteration 550 did not improve them
  (README Part D).
- **C05, C06, C09, C10:** need evaluation runs.
- **C08:** needs the measured model (F01–F05).
- **C07:** kept deliberately. The launch default stays `velstand.onnx` until C06 validates the new policy in
  closed loop. Every tool takes the policy explicitly (`walk_policy:=`, `--walk-policy`).

## D. SLAM and navigation: open

Not changed. D06 can now be tested on padded camera images (A04 is fixed). D12 still needs the real ROS job (B03)
first.

## E. Mechanical: open (hardware), one decision needed

**E01 needs the owner's answer:** the review mentions an earlier 50 g total-head target. The current estimate is
113.2 g for the three head prints alone, and about 197 g for the complete head link plus jaw. Which assembly the 50 g
covers decides whether the head shells get redesigned (thinner walls, lower infill: `v4_params.json` →
`print._proposed_head` brings the shells and jaw to about 77 g) or the revised budget is adopted.

E02–E12 need printed parts.

## F, G, H, I: open (hardware)

These need weighing, the servo bench test, the assembled HAT, the UNO Q and the sensors. Software that already
supports them: the calibration file (F), `HARDWARE_SENSORS.md` (H), the camera recovery (A03), the decoding (A04)
and the QoS (A01).

## J. Documentation

| ID | Status | What |
|-|-|-|
| J01 | Partly | The README's top section describes the V4. The V3 sections are labelled history where they were touched. A full rewrite should wait for hardware results. |
| J02 | Done | The purchase steps are replaced: TODO step 5, UNO_Q_GUIDE Step 6 and its plan table now say the parts are in hand, with the A010 as the V4 sensor and the VL53L5CX as the alternative. |
| J03 | Done | TODO step 6 no longer says "never the Microduck policy". It names the V4 fine-tune as nominally passing but unvalidated on measured hardware, with the XGO runtime as the alternative. |
| J04 | Partly | The README's V3 sensor table is labelled historical, and the slam_nav command is labelled legacy. Other old commands still need a pass. |
| J05 | Done | `scripts/slam_nav.py` refuses to run without `--legacy` and explains why. The README and TODO are updated. |
| J06 | Open | Needs fresh closed-loop runs with the final policy and geometry (C06/D13). |

## K. Optional: not started (as the handoff intends)
