# Hardware sensor acquisition boundary

These nodes are untested on ROS 2 or hardware. They provide an acquisition
interface and timestamped pose publication, **not a completed VL53L5CX I²C
driver**. No servo, Bridge protocol or motion control is implemented here.

`duck_tof` publishes the same 8×8 `PointCloud2` schema as `world_sim`:
`x/y/z/range` float32, `status` uint8 at byte 16, `zone` uint16 at byte 18.
Coordinates are metres, +x forward, +y left, +z up. Zone order is row-major,
top-left first as viewed from behind the sensor. Status is normalized to 0
(valid), 1 (no target/out of range), 2 (invalid/dropout). All invalid geometry
and ranges are NaN; invalid zones must never be used to clear free space.

## Backend to supply on the board

Install a Python module implementing `factory(config_dict) -> backend`, then set
`backend` to `your_module:factory`. The object must implement:

- `read() -> duck_nav.sensor_contract.TofFrame | None`: nonblocking, at most one
  new acquisition per call; return None while waiting. Run blocking firmware
  loading and I²C/Bridge reads in an adapter-owned worker with a bounded queue.
- `close()`: release the device and terminate any worker.

`TofFrame(stamp_ns, ranges, status)` contains exactly 64 metre ranges and 64
normalized statuses. `stamp_ns` is a positive integer **acquisition timestamp
in the ROS clock**, not publication time or an unmapped MCU uptime. Account for
integration/transport latency and synchronize an MCU clock before publishing.
An adapter must reorder zones, convert units and map raw vendor status codes
using the actual driver documentation and measurements. Raw ST status codes
are not this interface's normalized status codes.

`range_convention` must be explicitly `radial` (along each ray) or `axial`
(along the boresight). `zone_order_verified` must explicitly be true. These are
operator assertions after left/right/up/down target and range tests, not a
substitute for those tests. Minimum/maximum accepted ranges are in the selected
convention. Tune `fov_deg` and the SLAM node's `tof_fov_deg` together. The
acquisition wrapper rejects malformed, duplicate, reversed, future and stale
timestamps; default `max_age` is 0.5 seconds. `/tof/status` reports stale when
validly formed acquisitions stop. It does not assess whether a scene has enough
returns for localization, and it is not a motion safety watchdog.

A bundled **offline replay** adapter is `duck_nav.tof_replay:create_backend`.
Its config is `{"path":"/absolute/path/to/frames.jsonl","stamp_offset_ns":0}`.
Each line has `stamp_ns`, `ranges`, and `status` as above. It emits one frame
per poll, then None at EOF; it does not reproduce real sensor cadence. Use
recorded ROS bags with a matching clock for timed integration tests. Do not
represent replay as a live sensor. A nonzero explicit timestamp offset can align
a short recording with a test clock. Device read errors and rejected frames are
logged; an adapter must recover its own connection or be restarted.

## Exact acquisition-time poses

`sensor_pose_node` subscribes to `/camera/image_raw` and `/tof/zones`. For each
header it requests TF from that message's `frame_id` to `base_level` at exactly
its acquisition stamp, queues temporarily missing transforms for up to
`wait_timeout` (default 0.5 s), and publishes a matching-stamp PoseStamped on
`/camera/pose_level` or `/tof/pose_level`. It drops requests with missing TF;
it never substitutes the latest robot pose. Sensor input subscriptions accept
both reliable and best-effort producers.

Supply this measured TF chain separately:

1. Hardware encoder joint states, in URDF radians/signs/zero offsets, through
   `robot_state_publisher` for the four neck/head joints.
2. IMU-derived `base_level -> trunk_base`, with heading removed and the correct
   torso height above the floor. `base_level` is level with z up and origin on
   the floor below the torso, matching simulation. IMU mounting rotation,
   orientation validity, clocks and encoder/IMU sampling must be calibrated.
3. Measured rigid transforms from the head to the camera **REP-103 optical**
   frame and the ToF **+x forward, +y left, +z up** frame. Use frame names
   matching the acquisition headers. Do not publish optical frame names for
   a transform in another convention.

The node composes the optical camera rotation with 180° about optical x for
SLAM's MuJoCo camera convention (looks along -z, y up); translation is unchanged.
The ToF pose is already in the required convention. Do not also apply this
camera conversion upstream. The supplied URDF does not include measured sensor
extrinsics. No static identity transforms are provided as calibration defaults.

## Perception-only bring-up

After building the workspace, launch with your measured parameter file:

```bash
ros2 launch duck_nav hardware_sensors.launch.py sensor_config:=/absolute/path/sensors.yaml
```

The file uses node sections `usb_camera`, `duck_tof`, `sensor_pose_node` with
`ros__parameters` mappings. Set the USB camera's stable `device` path,
`calibration` YAML and optical `frame_id`; supply the ToF adapter parameters
above. This launch starts only acquisition and pose publication, uses wall
clock time, and requires the external measured TF chain. It does not start
navigation, SLAM, a policy or a command publisher. Run SLAM separately only
after calibrating hardware odometry; simulation gains are not hardware gains.

Before autonomous use: confirm timestamp alignment during head motion, check
all zone directions and depth convention, verify camera intrinsics and both
extrinsics, measure performance on the UNO Q, verify stale sensor/localization
stopping in the motion owner, and map a measured pen within 5 cm. The physical
bus/firmware adapter, IMU/encoder TF source and these checks still require the
actual robot.

Pure Python contract tests (no ROS/device required):

```bash
python -m pytest ros2/duck_nav/test/test_sensor_contract.py -q
```

## Official XGO runtime integration finding

The official [UNO Q runtime](https://github.com/LuwuDynamics/xgoduck_runtime_arduino)
was inspected at commit `0ba5cdcc2fef9b0da12fa11b338926c1fad8496e`. It already
implements the optional VL53L5CX on `Wire1` at 0x29, 8×8 at 5 Hz. This is a real
starting point for the bus implementation; there is no need to invent its I²C
protocol here. See
[`sketch/tof_sensor.h`](https://github.com/LuwuDynamics/xgoduck_runtime_arduino/blob/0ba5cdcc2fef9b0da12fa11b338926c1fad8496e/sketch/tof_sensor.h),
[`tools/read-tof.py`](https://github.com/LuwuDynamics/xgoduck_runtime_arduino/blob/0ba5cdcc2fef9b0da12fa11b338926c1fad8496e/tools/read-tof.py)
and
[`python/tof_status.py`](https://github.com/LuwuDynamics/xgoduck_runtime_arduino/blob/0ba5cdcc2fef9b0da12fa11b338926c1fad8496e/python/tof_status.py).

Its read-only `duck_tof` RPC returns a 292-byte little-endian `DTF1` version-1
snapshot (`<4s8B6I64h64B64B`), including sequence, MCU `sample_ms`, `age_ms`,
`read_us`, 64 millimetre distances, target status and target count. The official
decoder treats raw statuses 5 and 9 with positive target count/distance as valid.
Its HTTP `/api/tof/status` adds cache age and exposes decoded arrays. However,
`sampleMs` is assigned **after** `getRangingData()` completes: it identifies MCU
read completion, not sensor integration/acquisition. Neither that ABI nor HTTP
supplies an MCU-to-ROS clock mapping or a bounded acquisition-time offset. Using
host receipt time or subtracting HTTP age would therefore not satisfy this
package's acquisition-time pose contract.

Before implementing a live adapter: establish the integration timing/reference
and bounded latency, synchronize MCU and ROS clocks including reset/wrap
handling, and verify physical zone order and radial/axial convention. The
vendor's UI `grid_mm` reverses columns relative to its raw arrays, so do not
silently treat displayed grid order as canonical sensor geometry. Map raw target
statuses/counts explicitly. Until those checks and timing changes are made,
use the vendor's reader for board diagnostics and keep this package's strict
live backend unconfigured. No firmware was flashed or hardware exercised.
