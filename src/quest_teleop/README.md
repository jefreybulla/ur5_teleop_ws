# quest_teleop — Meta Quest 3 controller → ROS 2 → UR5

Streams the 6-DoF pose (position + orientation) and buttons of a Meta Quest 3
Touch controller into ROS 2, and teleoperates the UR5 with our own
resolved-rates IK from `ur5_kinematics`.

**No Unity, no APK.** The Quest side is a WebXR page served by the bridge
node and opened in the Quest Browser. The same page runs on a desktop with
Meta's **Immersive Web Emulator**, so everything can be tested without the
headset.

```
Quest 3 Browser (WebXR page) --WebSocket JSON @72 Hz--> quest_bridge --> /quest/right/pose, /joy, TF
                                                                          |
                                         quest_teleop (clutch + resolved rates) --> /joint_states (sim)
                                                                                 --> forward_position_controller (robot)
```

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/quest/<hand>/pose` | `geometry_msgs/PoseStamped` | grip pose, frame `quest_world` |
| `/quest/<hand>/twist` | `geometry_msgs/TwistStamped` | if the runtime reports velocities |
| `/quest/<hand>/joy` | `sensor_msgs/Joy` | axes `[thumb_x, thumb_y, trigger, squeeze]`, buttons `[trigger, squeeze, thumbstick, A/X, B/Y]` |
| `/quest/head/pose` | `geometry_msgs/PoseStamped` | headset pose |
| `/quest/rtt_ms` | `std_msgs/Float32` | page ↔ bridge round-trip time |
| `/quest_teleop/goal_pose` | `geometry_msgs/PoseStamped` | IK goal, frame `base_link` |
| `/quest_teleop/metrics` | `std_msgs/Float64MultiArray` | `[pos error m, rot error rad, manipulability]` |
| TF | | `quest_world → quest_right_grip`, `quest_world → quest_head` |

`quest_world` follows REP-103: x forward (the direction the headset faced when
the session started), y left, z up, origin on the floor (`local-floor`).
WebXR's own axes (x right, y up, z back) are converted in `frames.py`.

## Build

```bash
sudo apt install python3-aiohttp        # or: rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## 1. Test with no headset at all (fake_quest)

```bash
# Terminal 1: UR5 in RViz + bridge + teleop
ros2 launch quest_teleop quest_teleop_sim.launch.py
# Terminal 2: synthetic controller (circle, squeeze held, released 1 s every 6 s)
ros2 run quest_teleop fake_quest
```

In RViz add a **Pose** display on `/quest_teleop/goal_pose` and a **TF**
display. The tool should trace a 10 cm circle.

## 2. Test the real page with the emulator (no headset)

1. Install **Immersive Web Emulator** (by Meta) in Chrome or Edge.
2. `ros2 launch quest_teleop quest_teleop_sim.launch.py`
3. Open `http://localhost:8080/` and enable the emulator for the page (device: Meta Quest 3).
4. Click **Start VR** (or **Start AR**). In the emulator panel, select the right
   controller, hold **grip**, and drag/rotate it with the 3D gizmo.
5. Check: `ros2 topic echo /quest/right/pose`, and the arm moves in RViz.

`http://localhost` counts as a secure context, so no certificate is needed here.

## 3. Real Quest 3 in the lab

WebXR only runs on `https://` or `localhost`. Two options:

**A. USB + adb reverse (recommended; works on locked-down university Wi-Fi)**
Requires Developer Mode on the headset (Meta Horizon app) and `adb`.

```bash
adb devices                      # accept the USB-debugging prompt in the headset
adb reverse tcp:8080 tcp:8080
ros2 launch quest_teleop quest_teleop_sim.launch.py
```
In the Quest Browser open `http://localhost:8080/` → **Start AR (passthrough)**.

**B. Wi-Fi + self-signed HTTPS** (laptop and Quest on the same network, client-to-client allowed)

```bash
openssl req -x509 -newkey rsa:2048 -nodes -days 365 \
  -keyout ~/quest_key.pem -out ~/quest_cert.pem -subj "/CN=ur5-teleop"
ros2 launch quest_teleop quest_teleop_sim.launch.py port:=8443 \
  certfile:=$HOME/quest_cert.pem keyfile:=$HOME/quest_key.pem
```
In the Quest Browser open `https://<laptop-ip>:8443/`, accept the certificate warning.

**In the headset:** hold **squeeze** on the right controller to move the arm
(view tints green); release to freeze and re-grip to reposition your hand
(clutching). A red tint means the WebSocket dropped and the robot will not move.
If "forward" feels rotated, relaunch with `operator_yaw_deg:=90` (or 180, -90).

## 4. Real UR5 (after the sim test works)

Start with **URSim** (UR's official robot simulator, supported by `ur_robot_driver`)
before the real arm. The teleop node in hardware mode reads the driver's
`/joint_states` and streams joint positions to a forward position controller:

```bash
# with the course's UR driver running, switch controllers:
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller \
  --activate forward_position_controller
ros2 launch quest_teleop quest_bridge.launch.py &
ros2 run quest_teleop quest_teleop --ros-args -p mode:=hardware \
  -p scale:=0.5 -p v_max:=0.10 -p max_joint_speed:=0.5
```

Safety built in: squeeze is a dead-man switch; stale Quest data (>0.25 s)
opens the clutch; tool goal clamped to a workspace shell; Cartesian speed
capped by `v_max`/`w_max`; per-joint step capped by `max_joint_speed`.
Keep a hand on the teach-pendant e-stop. Confirm the command topic and
controller name match the course-provided driver config.

## Headless tests (no ROS needed)

```bash
PYTHONPATH=src/quest_teleop:src/ur5_kinematics python3 -m pytest src/quest_teleop/test
```

Covers frame conversion, clutch mapping, workspace clamping, a closed-loop
resolved-rates tracking test, and an end-to-end WebSocket test with `fake_quest`.

## Notes

- `ur5_kinematics`' `HOME_Q` has `q5 = 0`, a wrist singularity (manipulability ≈ 0;
  see `test_home_pose_singularity_check`). The teleop node starts from
  `[0, -90, 90, -90, -90, 0]` deg instead, so in sim the arm jumps once when teleop takes over.
- Rough reference from the literature: Quest2ROS (Unity app) reports ~82 ms
  Quest→ROS latency at ~72 Hz on a Quest 2.
