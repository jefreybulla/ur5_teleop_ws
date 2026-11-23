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

Dependencies: ROS 2 Jazzy, `python3-aiohttp` (`rosdep install --from-paths src --ignore-src -y`,
or `sudo apt install python3-aiohttp`).

**Using a Python venv instead of apt** (what we did in the lab): create one shared venv that
can still see ROS, install aiohttp there, and run colcon *through the venv's Python*.
Plain `colcon` has a `/usr/bin/python3` shebang, so it ignores an active venv; both the test
runner and the generated node scripts would then miss aiohttp.

```bash
# once
/usr/bin/python3 -m venv --system-site-packages ~/.venvs/ros2-jazzy
~/.venvs/ros2-jazzy/bin/pip install aiohttp

# every new terminal (order matters)
source /opt/ros/jazzy/setup.bash                 # usually already in ~/.bashrc
source ~/.venvs/ros2-jazzy/bin/activate
cd ~/Workspaces/ur5_teleop_ws                    # workspace root
python3 -m colcon build --symlink-install        # NOT plain `colcon build`
source install/setup.bash
ros2 pkg executables quest_teleop                # fake_quest, pose_server, quest_bridge, quest_teleop
```

With apt-installed aiohttp the usual `colcon build --symlink-install` is enough.
Do not use a venv that was created without `--system-site-packages` (it cannot import `rclpy`).

## 1. Test with no headset at all (fake_quest)

```bash
# Terminal 1: UR5 in RViz + bridge + teleop
ros2 launch quest_teleop quest_teleop_sim.launch.py
# Terminal 2: synthetic controller (circle, squeeze held, released 1 s every 6 s)
ros2 run quest_teleop fake_quest
```

The UR5 in RViz follows the synthetic controller: the tool traces a 10 cm circle and pauses
for about 1 s every 6 s while the squeeze is released. The bridge log reports
`72.0 Hz from Quest, 0 seq gaps, bad msgs total 0` and the teleop node logs
`Clutch released` / `Clutch engaged`. Optional: in RViz add a **Pose** display on
`/quest_teleop/goal_pose` (Add > By topic) and a **TF** display to see the target.

Quick numeric check (second terminal, env sourced): `ros2 topic hz /quest/right/pose` is about
72 Hz, `/joint_states` is published by `quest_teleop` at 50 Hz, and `/quest_teleop/metrics`
(`[pos error m, rot error rad, manipulability]`) shows a position error of about 0.006-0.007 m
while the clutch is held.

## 2. Test the real page with the emulator (no headset)

Tested with **Immersive Web Emulator v2.0** (Meta) in Chrome. Stop `fake_quest` first; the
emulator takes its place.

1. Install **Immersive Web Emulator** from the Chrome Web Store and pin it to the toolbar.
2. `ros2 launch quest_teleop quest_teleop_sim.launch.py`
3. Open `http://localhost:8080/`. Click the extension button so its popup says
   **"Emulating WebXR on this site"** (v2 enables emulation per site). In DevTools (F12) >
   **WebXR** tab set **Device = MetaQuest3**, **Input Mode = Controller**, then reload the page.
   The page's WebXR line must read `immersive-ar: yes, immersive-vr: yes`; the
   **Start VR** button is greyed out until then.
   (The popup's "AI Agent / MCP" section is optional and not needed.)
4. Click **Start VR**. A floating overlay shows **Controller [L]** and **Controller [R]** panels.
5. On **Controller [R]** set **Grip = 1.00** and click the **pin** icon next to it to latch it
   (the page turns **green**: clutch engaged). Squeezing alone does not move the arm.
6. Change the right controller's **Pos** X / Y / Z, or drag the 3D gizmo. The arm in RViz follows
   the *relative* motion. To clutch (reposition without moving the arm), set Grip back to 0 (or
   unpin it), move the controller, then latch the grip again.
7. Check: `ros2 topic hz /quest/right/pose` (about 60-72 Hz) and `ros2 topic echo /quest/right/pose --field pose.position`.

Axes: controller -Z (forward) becomes ROS +x, controller +X (right) becomes ROS -y, +Y is up.
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

## Tests

```bash
# headless pytest (with the ros2-jazzy venv active; keep the PYTHONPATH from sourcing ROS)
PYTHONPATH=src/quest_teleop:src/ur5_kinematics python3 -m pytest src/quest_teleop/test -v

# full colcon run incl. flake8 / pep257 lint (use the venv's Python, see Build)
python3 -m colcon test --packages-select quest_teleop ur5_kinematics
colcon test-result --verbose
```

14 tests: frame conversion, clutch mapping, workspace clamping, a closed-loop
resolved-rates tracking test, parser tolerance for null/NaN values, and an end-to-end
WebSocket test with `fake_quest`. `colcon test` on both packages reports 17 tests
(14 here + 3 flake8/pep257/copyright tests of `ur5_kinematics`, of which copyright is skipped).

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ModuleNotFoundError: No module named 'aiohttp'` in `colcon test` or `ros2 run` | colcon ran with the system Python. Use the venv and `python3 -m colcon build/test`, or install `python3-aiohttp`. |
| Page says `immersive-vr: no`, **Start VR** greyed out | Emulator not enabled for the site. Enable it from the toolbar popup, set the device in DevTools > WebXR, reload. The page re-checks for a few seconds after load. |
| Page streams, but no `/quest/*` topics and the bridge says `No controller data` | Check the bridge log for `rejected pose frame #N: ...`. The node log is also in `~/.ros/log/python3_<pid>_*.log`. (Fixed: the emulator sends null/NaN for some button/axis values.) |
| Grip is 1.00 and the page is green, but the arm does not move | The arm follows controller *motion* while the clutch is engaged. Change the controller `Pos` (or drag the gizmo) with the grip latched. |
| RViz crashes with `/snap/core20/.../libpthread.so.0: undefined symbol __libc_pthread_init` | Snap library conflict when launched from a snap-based shell (e.g. the VS Code terminal). Launch from a normal terminal. |
| `colcon test` for `ur5_kinematics` fails on flake8 / pep257 | Fixed in this branch (style-only commit). |

## Notes

- `ur5_kinematics`' `HOME_Q` has `q5 = 0`, a wrist singularity (manipulability ≈ 0;
  see `test_home_pose_singularity_check`). The teleop node starts from
  `[0, -90, 90, -90, -90, 0]` deg instead, so in sim the arm jumps once when teleop takes over.
- Rough reference from the literature: Quest2ROS (Unity app) reports ~82 ms
  Quest→ROS latency at ~72 Hz on a Quest 2.
