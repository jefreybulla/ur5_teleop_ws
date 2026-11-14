# ur5-teleop

Teleoperation for UR5 robotic arm.

## Build workspace

From the workspace root (`ur5_teleop_ws`), source ROS 2 Jazzy, install dependencies, then build:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Build kinematics package:

```bash
colcon build --packages-select ur5_kinematics --symlink-install
source install/setup.bash
```

Run forward kinematics demo:

```bash
ros2 run ur5_kinematics forward_kinematics_demo
```

## Online resolved-rates demo in RViz 2
This implementation uses our own FK and IK algorithm. 

Tested using two terminals: RViz stays open, restart motion by re-running servo.

**Terminal 1** — robot model and RViz (`launch_servo` defaults to `false`). A small **idle** node publishes `HOME_Q` on `/joint_states` so the arm displays in RViz before you start motion:

```bash
ros2 launch ur5_kinematics resolved_rates_demo.launch.py
```

**Terminal 2** — resolved-rates servo. This stops the idle publisher and takes over `/joint_states`:

```bash
ros2 run ur5_kinematics resolved_rates_demo
```

Stop terminal 2 with Ctrl+C and run the same command again to replay without closing RViz.

One-terminal all-in-one (optional):
```bash
ros2 launch ur5_kinematics resolved_rates_demo.launch.py launch_servo:=true
```

To test your own end effector goal:
```bash
ros2 run ur5_kinematics resolved_rates_demo --ros-args \
  -p use_goal_position_param:=true \
  -p goal_position:="[-0.447, -0.191, 0.629]"
```

In RViz, add displays if needed:
- **Path** — topic `/resolved_rates/tool_path`
- **Marker** — topic `/resolved_rates/goal_marker` (green sphere at the IK goal)

<img width="492" height="486" alt="Screenshot 2026-11-14 000921" src="https://github.com/user-attachments/assets/71f88512-1936-4ece-b001-24e67e83facd" />


## Launch UR5 model in RViz 2
```bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5
```

While visualization is open in another terminal window, run the following to obtain the forward kinematics from TF:

```bash
ros2 run tf2_ros tf2_echo base tool0
```


