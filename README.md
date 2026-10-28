# rtde_driver

A `robot_interface` driver plugin for Universal Robots controllers via the
[Universal Robots Client Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library).

It implements the `robot_interface::RobotDriver` interface so it can be loaded
by `robot_interface` at runtime as a shared library — no mc_rtc dependency required.

## Docker

The [`Dockerfile`](Dockerfile) builds an image that runs `uri interface` with
this driver. It starts from the `unified_robot_interface` image (ROS Jazzy,
mc_rtc, zenoh and URI already installed in `/opt/uri`), so only this driver and`ur_client_library` are added.

[`compose.yaml`](compose.yaml) runs the published image
(`ghcr.io/isri-aist/rtde_driver:latest`) with the options it needs, reading
configs from `./etc` (mounted on `/config`):

```bash
docker compose up                                       # etc/robot_interface.yaml
URI_CONFIG_FILE=ursim_ur5e.yaml docker compose up       # URSim
docker compose pull                                     # update to the latest main
docker compose up --build                               # build locally instead
```

It sets:

- `network_mode: host`, so zenoh reaches the manager and the driver reaches
  the UR controller (which also connects back to this host for the
  external-control script); `ipc: host` for the `zenoh/shm` protocol.
- `cap_add: SYS_NICE`, required: `uri` carries that file capability, and the
  kernel refuses to run it if the container can't grant it. Plus `rtprio` and
  `memlock` limits for real-time scheduling.

Environment variables, read from the shell or a `.env` file next to
`compose.yaml`:

| Variable | Default | Meaning |
|---|---|---|
| `URI_CONFIG_DIR` | `./etc` | Host directory mounted on `/config` |
| `URI_CONFIG_FILE` | `robot_interface.yaml` | File in it passed to `uri interface -c` |
| `RTDE_DRIVER_IMAGE` | `ghcr.io/isri-aist/rtde_driver:latest` | Image to run (e.g. pin a `:<commit sha>` tag) |
| `BASE_IMAGE` | `ghcr.io/isri-aist/unified_robot_interface:latest` | URI image to build on (with `--build`) |

The base image is published by the URI repository's CI. To build it locally
instead, from the URI repository:

```bash
docker buildx bake uri
```

Example configs are also in the image, in `/opt/uri/share/rtde_driver`.

## Dependencies

| Dependency | Where to get it |
|---|---|
| `unified_robot_interface` | [isri-aist/unified_robot_interface](https://github.com/isri-aist/unified_robot_interface) |
| `ur_client_library` | ROS package **or** built from source (see below) |

### Installing ur_client_library

**Option A — ROS package** (Humble / Iron / Jazzy):

```bash
sudo apt install ros-<distro>-ur-client-library
```

**Option B — from source** (no ROS required):

```bash
git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
cd Universal_Robots_Client_Library
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build .
sudo cmake --install .
```

`ur_client_library` must be discoverable by CMake.  Either source a ROS workspace,
install it system-wide, or pass the prefix explicitly:

```bash
# ROS install
cmake -DCMAKE_PREFIX_PATH="/path/to/unified_robot_interface/install;/opt/ros/humble" ..

# Source / system install
cmake -DCMAKE_PREFIX_PATH="/path/to/unified_robot_interface/install;/usr/local" ..
```

## Build

```bash
cd rtde_driver
mkdir build && cd build
cmake .. \
  -DCMAKE_PREFIX_PATH="/path/to/install;/opt/ros/humble" \
  -DCMAKE_INSTALL_PREFIX=/path/to/install
cmake --build .
cmake --install .
```

The plugin is installed to `lib/robot_interface/libRobotDriverRTDE.so`.

## Robot-side setup

`rtde_driver` runs in **headless mode**: it sends the URScript program directly to
the robot over the primary interface, so no URCap installation is needed.

The URScript file (`external_control.urscript`) ships with `ur_client_library` and
is found automatically.  Override the path with the `UR_SCRIPT_FILE` environment
variable if needed:

```bash
export UR_SCRIPT_FILE=/path/to/external_control.urscript
```

Search order:
1. `$UR_SCRIPT_FILE` environment variable
2. `/opt/ros/<humble|iron|jazzy>/share/ur_client_library/resources/external_control.urscript`
3. `/usr/share/ur_client_library/resources/external_control.urscript`

## Troubleshooting

### Plugin fails to load ("Available plugins" list is empty)

`uri` has the `cap_sys_nice` file capability, so the dynamic loader ignores
`LD_LIBRARY_PATH`. `liburcl.so` is found instead through the plugin's RUNPATH,
which `add_robot_driver()` in [`src/CMakeLists.txt`](src/CMakeLists.txt) sets
to the directories it was linked against. Check it:

```bash
readelf -d lib/robot_interface/libRobotDriverRTDE.so | grep RUNPATH
# e.g. [/opt/ros/<distro>/lib/x86_64-linux-gnu]
```

If `ur_client_library` has since moved (reinstalled under another prefix,
different ROS distro), rebuild the plugin. No `ld.so.conf.d` entry is needed.

## RTDE output fields

The driver requests the following RTDE fields from the robot:

| Field | Used by |
|---|---|
| `actual_q` | `getActualQ()` — joint positions \[rad\] |
| `actual_qd` | `getActualQd()` — joint velocities \[rad/s\] |
| `target_moment` | `getJointTorques()` — target joint torques \[Nm\] |
| `robot_mode`, `safety_mode`, `runtime_state` | `getStatus()`, and the program re-send below |

## Status, stops and recovery

`getStatus()` reports, for the controller (URI puts them in the mc_rtc datastore as `URI::<robot>::status`):

| Value | Meaning |
|---|---|
| `robot_mode` | RTDE robot mode, 7 = running |
| `safety_mode` | RTDE safety mode, 1 = normal, 2 = reduced; 6 and 7 are emergency stops |
| `runtime_state` | RTDE runtime state, 2 = playing |
| `program_running` | 1 while the external-control program runs: urcl's program state and `runtime_state` playing (after a power cycle urcl's state can stay "running" for a stopped program) |

After a stop (protective stop, emergency stop, the program stopping), once the robot is back in normal mode
(`robot_mode` 7, `safety_mode` 1 or 2) with the program stopped or paused, the driver sends the external-control
program again: at most every 2 s, and not in the first 5 s after startup, while the program sent at startup connects
back. The pendant must be in Remote Control for that.

After any stop, the arm then holds its actual joints until the controller's command is within 0.01 rad of them, so a
command that moved while the arm was stopped does not make it jump. This gate acts in `position` mode only (`servoJ`),
not on `speedJ` or `tauJ`.

Commands go to the program only while it plays (`program_running`). In Remote Control an emergency stop only pauses
the program (seen on CALL-M on 2026-10-28): its socket stays open but nothing reads it, so commands written to it
piled up until the write blocked the control loop, RTDE reads included, about 30 s after the stop. The program sent
once the robot is back in normal mode replaces the paused one.

## Control loop

`sync()` blocks until the robot delivers a new RTDE data package, pacing the
control loop at the robot's RTDE frequency (500 Hz by default).

```
loop:
  sync()          ← blocks, receives data package; sends the program again after a stop
  getActualQ()    ← reads from the cached package
  getActualQd()
  getJointTorques()
  getStatus()     ← robot and safety modes, program state
  servoJ(q)       ← sends position command (MODE_SERVOJ) while the program plays, or holds the arm after a stop
  speedJ(qd)      ← sends velocity command (MODE_SPEEDJ) while the program plays
```

## Plugin API

The shared library exports three C symbols consumed by `robot_interface`'s plugin loader:

```cpp
void ROBOT_DRIVER_PLUGIN(std::vector<std::string> & classes);  // registers "RobotDriverRTDE"
robot_interface::RobotDriver * create(const std::string & name,
                                         const std::string & ip,
                                         const uint16_t & port,
                                         const std::string & config_path,
                                         const std::vector<robot_interface::GripperInfo> & grippers);
void destroy(robot_interface::RobotDriver * ptr);
```

Configure it in `robot_manager/etc/mc_rtc_rtde.yaml` under the `robot_interface` key of the
relevant robot entry:

```yaml
Robots:
  ur5e:
    module: ur5e_module
    robot_interface:
      driver: RobotDriverRTDE
      ip: 192.168.1.100
      # port: 0          # optional, unused (RTDE uses a fixed port)
```
