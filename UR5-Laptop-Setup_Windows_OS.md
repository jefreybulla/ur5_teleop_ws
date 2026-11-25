# Connect the CB3 UR5 to ROS 2 Jazzy on a Windows laptop (WSL)

This guide reproduces the November 24, 2026 lab setup: a CB3 UR5 running PolyScope 3.14.3.1031232, a USB-C Ethernet adapter, and Ubuntu in WSL 2 on Windows 11. The session verified live joint feedback, a reverse connection from the robot, and an active position controller.

**Project root:** Start each Ubuntu terminal in `ur5_teleop_es/`. The robot-specific calibration lives at `./ur5_calibration.yaml`. Commands use `realpath` to expand that relative path when a ROS launch argument needs an absolute filename. Windows user settings and the system ROS installation have their own locations.

**Returning to an already configured laptop?** Connect the cable and go directly to [Start a lab session](#start-a-lab-session).

## One-time setup

### 1. Connect Ethernet and set the two addresses

Connect the laptop's USB-C Ethernet adapter directly to the Ethernet port on the UR control box. On the pendant, dismiss **Initialize Robot → OK**. If that opens the **Run** tab, use **File → Exit** to reach the Welcome screen. Choose **Setup Robot → Network** and verify **Static Address**. On Windows, open **Settings → Network & internet → Ethernet**, select the USB adapter, and edit **IP assignment → Manual → IPv4**.

| Setting | UR5 control box | Windows USB Ethernet adapter |
|---|---|---|
| Address | `192.168.1.11` | `192.168.1.10` |
| Subnet mask | `255.255.255.0` | `255.255.255.0` or prefix length `24` |
| Gateway and DNS | Existing values were `0.0.0.0` | Leave blank on this direct link |

In **PowerShell**, test the robot's Real-Time Data Exchange (RTDE) port:

```powershell
Test-NetConnection 192.168.1.11 -Port 30004
```

Expected: `TcpTestSucceeded : True`, source `192.168.1.10`, and the wired adapter (called `Ethernet 2` in this session).

### 2. Enable WSL mirrored networking

In **PowerShell**, verify Ubuntu uses WSL version 2 and open the Windows user configuration:

```powershell
wsl -l -v
notepad "$env:USERPROFILE\.wslconfig"
```

Add the setting under the existing `[wsl2]` section, or create that section if needed:

```ini
[wsl2]
networkingMode=mirrored
```

Save WSL work before shutting it down. In **PowerShell**, run:

```powershell
wsl --shutdown
wsl -- ip route get 192.168.1.11
wsl -- python3 -c "import socket; s = socket.create_connection(('192.168.1.11', 30004), 3); print(s.getsockname()[0]); s.close()"
```

The route should show `src 192.168.1.10`, and Python should print `192.168.1.10`. During setup, an outbound test also succeeded under NAT with a `172.x.x.x` source. Check the source address as well as the TCP result.

### 3. Allow the robot to connect back into WSL

The robot fetches its External Control program from the laptop on TCP 50002. Live joint feedback can already be working while this inbound connection is blocked.

Open **PowerShell as Administrator** and check for an existing rule:

```powershell
Get-NetFirewallHyperVRule -Name "UR5-ROS2" |
    Format-List Name,Enabled,Direction,Action,LocalAddresses,LocalPorts,RemoteAddresses
```

If the elevated command reports no matching rule, create it once:

```powershell
New-NetFirewallHyperVRule -Name "UR5-ROS2" -DisplayName "UR5 to ROS 2 in WSL" -Direction Inbound -Action Allow -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol TCP -LocalAddresses 192.168.1.10 -LocalPorts 50001-50004 -RemoteAddresses 192.168.1.11
```

If you see `Access is denied` (Windows System Error 5), open an elevated window with `Start-Process powershell.exe -Verb RunAs` and execute the command in the new window.

### 4. Configure the External Control program on the pendant

1. Open **Setup Robot → URCaps** and verify **External Control** is installed. On a fresh controller, install the CB3-compatible `.urcap` using the **+** button and restart when prompted. [Official URCap downloads](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases)
2. Open **Program Robot** and load the lab's `external_control_test.urp`. If creating a new program, add a single **External Control** node from **Structure → URCaps**.
3. Under **Installation → External Control**, set **Host IP** to the **laptop**, `192.168.1.10`, and **Custom port** to `50002`. Save the installation and any newly created program.
4. Under **Program → Command**, select the External Control node and check the full address. This program initially showed `192.168.1.100`; the working value ends in **.10**. Leave the program stopped until the driver is up.

### 5. Install the Jazzy packages and extract calibration

In **Ubuntu/WSL**, with the robot powered on and idle:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install ros-jazzy-ur-robot-driver ros-jazzy-ur-calibration ros-jazzy-ur-client-library

ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.11 \
  target_filename:="$(realpath -m ./ur5_calibration.yaml)"

head ./ur5_calibration.yaml
```

Look for `Wrote output` and `Calibration correction done`. The YAML begins with `kinematics:`. Reuse this file when reconnecting to the same arm.

## Start a lab session

### 1. Launch the driver in WSL terminal A

Use a normal Ubuntu terminal so RViz inherits the display settings. Stop the prior simulation and its joint-state publisher.

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5 \
  robot_ip:=192.168.1.11 \
  reverse_ip:=192.168.1.10 \
  kinematics_params_file:="$(realpath ./ur5_calibration.yaml)" \
  launch_rviz:=true
```

Leave terminal A running. In the lab, the driver negotiated RTDE protocol 2 at 125 Hz, printed `Calibration checked successfully`, and activated `joint_state_broadcaster`.

### 2. Verify measured joint angles in WSL terminal B

The following read-only check works before pressing Play. It uses a direct Python subscriber because the ROS CLI timed out on this laptop even while joint feedback was arriving.

```bash
source /opt/ros/jazzy/setup.bash
python3 - <<'PY'
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("verify_ur5_feedback")
samples = []
sub = node.create_subscription(JointState, "/joint_states", samples.append, qos_profile_sensor_data)
deadline = time.monotonic() + 8
while not samples and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
if samples:
    print(dict(zip(samples[0].name, samples[0].position)))
else:
    print("No joint sample received within 8 seconds.")
node.destroy_node()
rclpy.shutdown()
PY
```

Expect six named joint angles in radians. Compare RViz with the stationary physical arm. In the original eight-second test, the subscriber received 797 joint-state messages.

### 3. Establish the reverse connection

With the workcell clear and the driver running, press **Play** on the pendant's single-node `external_control_test.urp`. Watch terminal A for:

```text
Robot requested program
Sent program to robot
Robot connected to reverse interface. Ready to receive control commands.
```

**The connection is verified when both checks pass:** terminal B receives the arm's joint angles and terminal A reports the reverse connection.

### 4. Select position control for the resolved-rate IK node

Use this step when preparing to stream joint angles. The lab's IK loop produces six **absolute angles in radians**. Keep its publisher stopped during the switch. The verified switch released the trajectory controller and activated the position controller in one request.

In **WSL terminal B**, run:

```bash
python3 - <<'PY'
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController

rclpy.init()
node = rclpy.create_node("select_ur5_position_controller")
old = "scaled_joint_trajectory_controller"
new = "forward_position_controller"

def call(service, name, request):
    client = node.create_client(service, "/controller_manager/" + name)
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError(f"Service unavailable: {name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8)
    if not future.done():
        raise RuntimeError(f"No reply from {name}. Check states before retrying.")
    return future.result()

def states():
    reply = call(ListControllers, "list_controllers", ListControllers.Request())
    return {c.name: c.state for c in reply.controller}

try:
    current = states()
    if current.get(new) == "inactive" and current.get(old) == "active":
        request = SwitchController.Request()
        request.deactivate_controllers = [old]
        request.activate_controllers = [new]
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 5
        print("Switch accepted:", call(SwitchController, "switch_controller", request).ok)
        current = states()
    for name in (old, new):
        print(name, current.get(name, "not loaded"))
    if current.get(new) != "active":
        print("Check the pendant program and driver log. Active controllers:",
              [name for name, state in current.items() if state == "active"])
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

Expected final states: `scaled_joint_trajectory_controller inactive` and `forward_position_controller active`. This completes the setup verified in the lab.

| IK interface | ROS topic | Message and ordering |
|---|---|---|
| Feedback | `/joint_states` | `sensor_msgs/msg/JointState`. Pair each position with the name at the same index. |
| Position commands | `/forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray`. The configured array order is shown below. |

```text
[shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
 wrist_1_joint, wrist_2_joint, wrist_3_joint]
```

The observed feedback listed the elbow first. The `resolved_rates_goal` node reads joints by name and clips per-cycle joint motion (`max_joint_speed`). Start with conservative `v_max` and `max_joint_speed` on the first motion test.

## Troubleshooting

| Symptom | Check or fix |
|---|---|
| Calibration exits with an undefined symbol containing `HardwareInfoMessage` | In WSL, run `apt-cache policy ros-jazzy-ur-calibration ros-jazzy-ur-client-library`. The session had client library 2.15.2 installed with 2.17.0 available. Run `sudo apt update` and `sudo apt install --only-upgrade ros-jazzy-ur-client-library`, then retry extraction. |
| Pendant displays “Receive program failed” and times out at `192.168.1.10:50002` | Keep the driver running. In another WSL terminal, run `ss -ltnp 'sport = :50002'`. The working listener was `0.0.0.0:50002`, owned by `ros2_control_node`. If present, verify the elevated Hyper-V firewall rule in step 3 of one-time setup. |
| Windows reaches port 30004 but WSL uses a `172.x.x.x` source | Check the Windows user’s `.wslconfig`, restart WSL after saving work, and repeat the route and socket checks. |
| ROS CLI hangs on `topic echo` or `list_controllers` | Use terminal B and the bounded Python checks above. To inspect discovery independently, run `timeout -k 2s 8s ros2 node list --no-daemon --spin-time 3`. An advertised topic alone does not prove sample delivery. |
| Controller switch returns `False` | Check the actual active controllers, the pendant program state, and the driver log at the switch timestamp. Release the active joint-motion controller in the same request. The position switch above succeeded with the scaled trajectory controller active. |

## References

- [Universal Robots · CB3 robot setup and External Control](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/jazzy/doc/ur_client_library/doc/setup/robot_setup.html)
- [Universal Robots · Calibration extraction](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/jazzy/doc/ur_robot_driver/ur_calibration/doc/usage.html)
- [Microsoft · WSL mirrored networking and firewall](https://learn.microsoft.com/windows/wsl/networking)
