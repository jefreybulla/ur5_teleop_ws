# ur5-teleop

Teleoperation for UR5 robotic arm.

# Launch UR5 model in RViz 2

```
ros2 launch ur_description view_ur.launch.py ur_type:=ur5
```

while visualization is open in another temrinal window run the following to obtain the forward kinematics

```
ros2 run tf2_ros tf2_echo base tool0
```