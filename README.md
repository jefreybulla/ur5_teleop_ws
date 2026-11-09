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

## Launch UR5 model in RViz 2

The `ur_description`, `joint_state_publisher_gui`, `robot_state_publisher` and `rviz2` packages are declared in `ur5_kinematics/package.xml`, so the `rosdep install` step above installs them automatically.

```bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5
```

While visualization is open in another terminal window, run the following to obtain the forward kinematics from TF:

```bash
ros2 run tf2_ros tf2_echo base tool0
```

<img width="414" height="498" alt="image" src="https://github.com/user-attachments/assets/5f47ca06-ed86-4d60-b0b7-64f357d75d68" />

