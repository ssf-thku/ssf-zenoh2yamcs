# ssf-zenoh2yamcs

Bridge between Zenoh and YAMCS. Sends rover telemetry and camera images to YAMCS, and forwards YAMCS commands to the motion stack and Isaac runtime.

## Install

Deps: Ubuntu and Pixi

From this repo:

```bash
pixi install
```

## Usage

Start YAMCS, the Zenoh router, the Isaac runtime and the motion stack, then:

```bash
pixi run bridge

# options:
pixi run bridge --help
pixi run bridge --address localhost:8090
pixi run bridge --hz 5
```

By default, the bridge connects to the router at `tcp/localhost:7447` (see `zenoh.json5`) and YAMCS at `localhost:8090`, instance `ssf-tohoku`, processor `realtime`.

Command decoding uses the MDB at `../yamcs-mission-control/yamcs-server/src/main/yamcs/mdb/rover.xml`. Keep that repo alongside this one, or pass `--mdb /path/to/rover.xml`. It must match the running YAMCS server. Destination parameters must be defined as LOCAL in YAMCS.

## Zenoh

KeyExpr:

- `OmniLRS/pioneer/imu` (subscribes) -- IMU readings to `/Rover/cdh/imu_*`.
- `OmniLRS/pioneer/joint_telemetry` (subscribes) -- All named joint states to `/Rover/motor/joint_telemetry` as JSON, including position, velocity, effort and source timestamp.
- `OmniLRS/pioneer/COMMAND` (publishes) -- Motion requests from YAMCS. Commands include `drive_straight`, `drive_turn`, deployment and rock pickup commands.
- `cameras/capture` (publishes) [`List[str]`, JSON] -- Camera names requested through YAMCS.
- `cameras/one_shot/CAMERA_NAME` (subscribes) [`image`] -- Captured image, encoded as MessagePack `[dtype, shape, data]` by the Isaac runtime.

For example, to do the equivalent of `python -m ssf_motion_stack.cmd drive -1 1`, send `/Rover/motor/drive_straight` in YAMCS with `linear_velocity = -1` and `distance = 1`.

## Cameras

In YAMCS:

- `/Rover/camera/camera_capture_high` -- Select one camera from the dropdown.
- `/Rover/camera/camera_capture_group` -- Comma-separated names, e.g. `nav_left,nav_right`. Leave the default to capture all cameras together at the same time.

Names are the camera prim's USD `displayName`, falling back to its prim name. Current names: `gripper`, `mast_vert`, `mast_horiz`, `haz_front`, `haz_back`, `belly`, `nav_left`, `nav_right`, `sim_front`, `sim_back`, `sim_ortho_top`.

Images are uploaded as PNGs to the YAMCS `images_oncommand` bucket. Each capture saves a  new timestamped `CAMERA_NAME_TIMESTAMP.png` and replaces `CAMERA_NAME_latest.png` (so you can continuously poll/refresh it).

For example, you can display and poll at 1Hz the latest capture of the front simulation camera with:

```text
feh --reload 1 http://localhost:8090/api/storage/buckets/images_oncommand/objects/sim_front_latest.png
```

## Code

Scheduling and Zenoh integration use asyncio and `asyncio_for_robotics`. Forwarding tasks share a Zenoh session and run inside an afor scope.

- `bridge.py` -- Starts the forwarding tasks.
- `telemetry.py` -- Converts incoming telemetry to YAMCS parameters.
- `commands.py` -- Decodes YAMCS commands using the MDB and publishes Zenoh requests.
- `cameras.py` -- Decodes images and encodes PNGs.
- `yamcs.py` -- afor interfaces for parameter publishing, image uploads and receiving UDP commands.

Publishing and uploads are synchronous. Waiting for commands and Zenoh samples is async.

## Image storage errors

A failed camera upload is logged and displayed in red in the UI. The message gives the bucket path, quota, and a plain delete-or-move command. The bridge keeps running.
