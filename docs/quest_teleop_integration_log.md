# quest_teleop integration log (ME 650 final project, second half)

Date: 2026-11-22. Repo: `~/Workspaces/ur5_teleop_ws` (origin: `jefreybulla/ur5_teleop_ws`). ROS 2 Jazzy, Ubuntu, Python 3.12.

Goal: add the `quest_teleop` package (Meta Quest 3 WebXR controller to UR5 teleop, built on `ur5_kinematics` resolved rates) as a branch, build it, and test it.

---

## 1. Repo preparation

```bash
cd ~/Workspaces/ur5_teleop_ws
git status                  # clean (patch + zip were in ~/Downloads, not in the repo)
git fetch origin
git checkout main
git pull --ff-only          # already up to date at b66984b
git checkout -b quest-teleop
```

## 2. Apply the patch

```bash
git am ~/Downloads/0001-Add-quest_teleop-Quest-3-WebXR-controller-pose-bridg.patch
```

- Applied cleanly as commit `ffc2bc0`, original author and message kept. The zip fallback was not needed.
- 18 files, +1679 lines: `src/quest_teleop/**` plus an 11-line edit to the root `README.md`.
- The patch and zip are NOT committed (they live in `~/Downloads`).
- `ur5_kinematics` was not modified.

## 3. Python environment (the venv decision)

Findings:
- `apriltag_ws` has no venv. It used system Python 3.12 + `/opt/ros/jazzy`.
- The only existing venv was `~/.venvs/reinforcement-learning-python`. It is isolated (`include-system-site-packages = false`), so it cannot see `rclpy`. Not suitable for ROS.
- System Python had pytest but no aiohttp, so `quest_teleop` tests failed with `ModuleNotFoundError: No module named 'aiohttp'`.

Decision: a shared ROS venv in the central `~/.venvs/` folder (similar to the old Windows habit of one folder of reusable venvs), with system site-packages so ROS and apt packages stay visible. No `sudo` / apt was needed.

```bash
/usr/bin/python3 -m venv --system-site-packages ~/.venvs/ros2-jazzy
~/.venvs/ros2-jazzy/bin/pip install aiohttp pytest   # aiohttp 3.14.5; pytest was already provided by the system
```

Verified: `import rclpy, aiohttp, pytest, numpy` all work inside the venv.

Gotcha: the shell's default `python3` was the RL venv, so the venv has to be deactivated before sourcing ROS.

## 4. Build (done by hand, step by step)

Order matters: ROS first, then the venv, then the workspace overlay.

```bash
deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source ~/.venvs/ros2-jazzy/bin/activate
which python3                  # must be ~/.venvs/ros2-jazzy/bin/python3
cd ~/Workspaces/ur5_teleop_ws  # workspace ROOT, not src/
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables quest_teleop
```

Result: both packages built with no errors. Executables: `fake_quest`, `pose_server`, `quest_bridge`, `quest_teleop`.

Notes:
- `--symlink-install`: editing `.py` files needs no rebuild; new packages, entry points, launch files or `setup.py` changes do.
- Run colcon only from the workspace root.

## 5. Headless pytest

```bash
PYTHONPATH=src/quest_teleop:src/ur5_kinematics python3 -m pytest src/quest_teleop/test -v
```

Result: **13 passed in 1.47 s.** Coverage: frame conversions (WebXR to ROS), clutch mapping, workspace clamp, joint-step clipping, closed-loop tracking through `resolved_rates_step`, home-pose singularity check, and a server end-to-end test with `fake_quest`.

Finding: the `ur5_kinematics` demo home pose has q5 = 0 (wrist singularity, manipulability < 1e-6). Teleop uses a different start pose, `[0, -pi/2, pi/2, -pi/2, -pi/2, 0]`.

## 6. `colcon test` (first run, from a second terminal)

```bash
colcon test --packages-select quest_teleop ur5_kinematics
```

| Package | Result | Cause |
|---|---|---|
| `quest_teleop` | failed (exit code 2) | That terminal had no venv active, so `import aiohttp` failed. Environment issue, not a code bug. |
| `ur5_kinematics` | test failures | `test_flake8` (21 style errors: 10 E, 6 I, 5 W) and `test_pep257` fail on the existing lint tests. `test_copyright` is skipped. Pre-existing code style issues; we did not touch this package. |

Also noticed: that terminal sourced `install/setup.bash` BEFORE `/opt/ros/jazzy/setup.bash`. Use the order from section 4.

Next: rerun `colcon test` in the venv terminal to confirm `quest_teleop` passes. Decide with the team whether to fix the `ur5_kinematics` lint (only if we agree to touch that package).

## 7. Still to do

- Smoke test: `ros2 launch quest_teleop quest_teleop_sim.launch.py`, then `ros2 run quest_teleop fake_quest --duration 10`; check `/quest/right/pose` is about 72 Hz, `/joint_states` is published by quest_teleop, and `/quest_teleop/metrics` shows position error < ~0.01 m.
- Summarize README instructions for the Immersive Web Emulator (`http://localhost:8080`) and the real Quest 3 (`adb reverse tcp:8080 tcp:8080`).
- Final branch summary, then ask before any push / PR for Jefrey's review. Nothing has been pushed.

---

## 8. Sourcing order (recommended sequence)

```bash
source /opt/ros/jazzy/setup.bash          # 1. ROS underlay
source ~/.venvs/ros2-jazzy/bin/activate   # 2. shared venv (aiohttp etc.)
source install/setup.bash                 # 3. this workspace's overlay (from the workspace root)
```

Why the old habit of a single `source install/setup.bash` also works:
- `~/.bashrc` already has `source /opt/ros/jazzy/setup.bash` (line 118), so every new terminal has the ROS underlay.
- `install/setup.bash` records the underlays that were sourced when the workspace was built (its "prefix chain": `/opt/ros/jazzy`, `ros2_zed_ws`) and re-sources them. So it is effectively one line per workspace.
- Do not edit `install/setup.bash`; it is regenerated on every build. The standard practice is: ROS in `~/.bashrc`, one `source install/setup.bash` per workspace.
- The `(ros2-jazzy)` prefix is just the venv indicator. It is separate from ROS and is not part of `install/setup.bash`.

## 9. Why `colcon test` failed for quest_teleop even inside the venv

- Log shows colcon runs the tests as `/usr/bin/python3 -m pytest` (hardcoded system interpreter), so the venv's aiohttp is not visible: `ModuleNotFoundError: No module named 'aiohttp'`.
- The same applies to the installed executables: `install/quest_teleop/lib/quest_teleop/quest_bridge` and `fake_quest` have shebang `#!/usr/bin/python3`, so `ros2 run` / `ros2 launch` would also miss aiohttp.
- Cause: the `colcon` command is a script with shebang `/usr/bin/python3`, so it ignores the active venv. Plain `pytest` worked earlier because it ran under the venv python.
- Fix (no sudo): run colcon through the venv interpreter, `python3 -m colcon build --symlink-install` (colcon is importable in the venv because of `--system-site-packages`). Then `sys.executable` is the venv python, which is used for the test runner and for the generated shebangs.

```bash
cd ~/Workspaces/ur5_teleop_ws
python3 -m colcon build --symlink-install --packages-select quest_teleop
source install/setup.bash
python3 -m colcon test --packages-select quest_teleop
colcon test-result --verbose
```

(Alternative that needs sudo: `sudo apt install python3-aiohttp`, the standard ROS way, but this makes the shared venv unnecessary for this package.)

## 10. Venv-based rebuild and quest_teleop test result (user ran this)

```bash
python3 -m colcon build --symlink-install --packages-select quest_teleop
source install/setup.bash
head -1 install/quest_teleop/lib/quest_teleop/quest_bridge   # now #!~/.venvs/ros2-jazzy/bin/python3
python3 -m colcon test --packages-select quest_teleop
colcon test-result --verbose
```

Result: quest_teleop tests pass (0 errors). Only the ur5_kinematics lint tests were still failing.

## 11. ur5_kinematics lint cleanup (commit d19b7a4, requested by user)

Style-only, no logic changes. 9 files, +72 / -44.

- pep257: `ur5_dirkin`, `compute_jacobian`, `trajectory_control` docstrings converted to numpy style (summary on the 2nd line, `Parameters/Returns/Raises` with dashed underlines, blank line before closing quotes).
- flake8 import order (google style, sorted by module name): `setup.py`, `resolved_rates.py`, `resolved_rates_demo.py` (names in the `from ... import (...)` are ordered as flake8 asks: `resolved_rates_step, tool_pose, TrajectoryControlParams`).
- Missing newline at end of file (W292): `resolved_rates_demo.launch.py`, `frame_utils.py`, `joint_state_idle.py`, `resolved_rates_demo.py`.
- `forward_kinematics_demo.py:18`: spaces after commas (E231).
- Long lines (E501), inline-comment style (E262) and trailing whitespace (W291): the long comments in `resolved_rates_demo.py` and the `dt` comment in `resolved_rates.py` moved to their own lines or shortened. The `dt` comment wording was shortened slightly.

Verification: `python3 -m colcon test --packages-select quest_teleop ur5_kinematics` gives 16 tests, 0 errors, 0 failures, 1 skipped (the skipped one is `test_copyright`).

Lesson: a plain `pytest` run with `PYTHONPATH=...` overwritten loses the ROS paths (`ament_flake8` etc. not found). Prefer `colcon test` for the lint tests and keep the original `PYTHONPATH` when needed.

## 12. Git history cleanup (done by Claude at user's request, then git handed over to the user)

- Confirmed we work on branch `quest-teleop` (created from up-to-date `main` b66984b; `main` untouched).
- Removed `Co-Authored-By: Claude ...` trailers from both local commits, and the `Claude-Session:` line from the quest_teleop commit, via a non-interactive `git rebase main --exec 'git commit --amend ...'`. Nothing had been pushed, so no force push was needed.
- Current history (author kept; the lint commit uses a second email address, which is fine because GitHub is linked to both emails):
  - `c7030e7` Add quest_teleop: Quest 3 WebXR controller pose bridge and UR5 teleop
  - `4292d6b` Fix ur5_kinematics flake8/pep257 lint errors
- From now on the user does all commits, branches, deletions and pushes personally.

## 13. ROS smoke test (sim mode, 2026-11-22)

Environment: venv + `install/setup.bash` sourced (see section 8). Launch:

```bash
ros2 launch quest_teleop quest_teleop_sim.launch.py      # terminal A
ros2 run quest_teleop fake_quest --duration 15           # terminal B
ros2 topic hz /quest/right/pose
ros2 topic info /joint_states -v
ros2 topic echo /quest_teleop/metrics
```

Results:

| Check | Result |
|---|---|
| Nodes started | robot_state_publisher, joint_state_idle, quest_bridge, quest_teleop ("mode=sim, hand=right, 50 Hz"); bridge serves http://0.0.0.0:8080/ |
| `/quest/right/pose` rate | **72.02 Hz** (min 11 ms / max 16 ms, std 1 ms) |
| `/joint_states` | published by **quest_teleop** at **50.0 Hz** (idle publisher stops itself: "servo owns /joint_states") |
| `/quest_teleop/metrics` `[pos err m, rot err rad, manipulability]` | clutched: pos error **0.006-0.007 m** (< 0.01 target), rot error 0.009-0.054 rad, manipulability 0.06-0.087; 0.0 when clutch released |
| fake_quest | sent 864 frames (12 s) / 1080 frames (15 s), RTT 0.2-0.4 ms |
| After fake_quest stops | "Clutch released (Quest data stale)" and bridge warns "No controller data" (expected failsafe) |
| Shutdown | launch stopped with SIGINT; no leftover processes |

Known issue (environment, not code): **RViz died** at launch with `/snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE`. This is a snap library conflict that appears when RViz is started from this shell environment. Try launching from a normal terminal (e.g. Tilix), or `unset LD_LIBRARY_PATH` / check for snap-related `GTK_PATH` / `LD_LIBRARY_PATH` entries.

## 14. Test 1 (no headset): synthetic controller in RViz, run by the user - PASSED

Setup (two Tilix terminals, each with the venv + `install/setup.bash` sourced; RViz works from Tilix, the snap error only appeared in the VS Code / tool shell):

```bash
# Terminal 1
ros2 launch quest_teleop quest_teleop_sim.launch.py
# Terminal 2
ros2 run quest_teleop fake_quest
```

Result: the UR5 in RViz follows the synthetic controller, tracing a circle and pausing for about 1 s between loops (the fake controller releases the squeeze for 1 s every 6 s). Bridge log: `72.0 Hz from Quest, 0 seq gaps, bad msgs total 0`; teleop log alternates `Clutch released` / `Clutch engaged`.

Screenshots (for the instructions / report):

## 15. Test 2 (emulator, no headset): first attempt, page opened in Chrome

Opened `http://localhost:8080/` in Chrome while the launch was running. The page loads and shows `WebSocket: connected`, round trip about 1.5 ms. `Start AR` / `Start VR` are greyed out and the WebXR line reads `immersive-ar: no, immersive-vr: no`. This is expected: plain Chrome has no WebXR device, so the page disables the buttons until the Immersive Web Emulator extension is enabled for the site (see below).

Notes for the instructions:
- "Add a Pose display" is an RViz step (Add -> By topic -> `/quest_teleop/goal_pose` -> Pose; also add TF). It is not part of the web page or the emulator.
- Immersive Web Emulator v2.x is enabled per site: pin the extension, open the page, click the extension button to enable emulation for that site, reload. DevTools has a WebXR panel for persistent settings; live controller control is in an in-page overlay. Keyboard shortcuts in the older docs: left grip = Q, right grip = Shift.

## 16. Emulator: page still said "immersive-vr: no" even with the emulator enabled (diagnosis + fix)

Observed (Chrome, Immersive Web Emulator v2.0.0, popup: "Emulating WebXR on this site", DevTools > WebXR panel: Device = MetaQuest3, Input Mode = Controller):
- Page: `immersive-ar: no, immersive-vr: no`, Start AR / Start VR greyed out.
- DevTools console on the same page: `navigator.xr.isSessionSupported('immersive-vr')` -> `true`, and `navigator.xr.requestSession` was the emulator's code. So the emulator WAS active; the page's own check was stale.
- The console error `Unchecked runtime.lastError: The message port closed before a response was received` comes from the extension messaging and is harmless.

Cause: `initXR()` in `web/index.html` asked `isSessionSupported` only once at page load. The emulator registers its virtual headset a moment later, so the first answer was "no" and the page never asked again.

Fix (edit to `src/quest_teleop/quest_teleop/web/index.html`, 9 lines added, NOT yet committed): `initXR()` now returns whether AR/VR is supported, and a new `watchXR()` re-runs it on `navigator.xr` `devicechange` and retries at 0.5 s, 1.5 s and 3.5 s (skipped while a session is active). The bridge serves the file from disk, so no rebuild is needed, only a page reload.

Notes for the instructions:
- Emulator v2.0 needs: toolbar button -> "Emulating WebXR on this site" (per-site), device set in DevTools > WebXR (persistent defaults), then reload. Live controller controls are in a floating overlay on the page itself (appears in the page once a VR session starts), not in DevTools.
- The "AI Agent / MCP bridge" part of the extension popup is optional and not needed.

## 17. Emulator streaming but arm does not move: bridge rejected every frame (root cause + fix)

Symptoms (emulator in VR, right grip pinned at 1.00, green tint = clutch engaged):
- Page: `immersive-vr (local-floor)`, Frames sent 36681 at about 60 Hz, round trip 0.5 ms.
- ROS: no data on `/quest/right/pose`, `/quest/right/joy`, `/quest/head/pose`, `/quest/rtt_ms`; bridge printed `No controller data for 1745 s`. Arm did not move.

Diagnosis:
1. The server silently dropped frames that failed `parse_frame`. Added logging in `pose_server.py`: `rejected pose frame #N: <reason>` (first one, then every 200th).
2. Where to read the bridge output when the terminal scrolled away: the node log `~/.ros/log/python3_<bridge pid>_*.log` (the launch.log only has the process start lines).
3. Log result: `rejected pose frame #12800: TypeError("float() argument must be a string or a real number, not 'NoneType'")`, i.e. every emulator frame was rejected.

Root cause: the emulator sends `null` (JSON.stringify of NaN) for some gamepad button value or axis, and `protocol.parse_frame` did `float(None)`. A real Quest does not do this, but the parser should be tolerant.

Fix (uncommitted edits, 4 files):
- `quest_teleop/protocol.py`: new `_num()` helper (null/NaN/non-numeric -> 0.0), used for `buttons_value` and `axes`.
- `quest_teleop/pose_server.py`: log rejected frames (see above).
- `quest_teleop/web/index.html`: re-check WebXR support (section 16).
- `test/test_quest_teleop.py`: new test `test_parse_frame_tolerates_null_button_values_and_axes`.

Verification: pytest 14 passed; `colcon test --packages-select quest_teleop`: 17 tests, 0 errors, 0 failures, 1 skipped (lint included).

Next: restart the launch (Ctrl+C, relaunch), reload the page, Start VR, pin Grip = 1.00, then change the right controller `Pos` values and watch the arm in RViz.

## 18. Test 2 (emulator, no headset): WORKING (confirmed by user)

After the fixes in sections 16 and 17, the Immersive Web Emulator drives the UR5 in RViz.

Procedure that works (for the final instructions):
1. Terminal (venv + `install/setup.bash` sourced): `ros2 launch quest_teleop quest_teleop_sim.launch.py` (starts bridge, teleop, robot_state_publisher, RViz).
2. Chrome with the Immersive Web Emulator v2.x: open `http://localhost:8080/`, click the extension button so the popup says "Emulating WebXR on this site"; DevTools > WebXR panel: Device = MetaQuest3, Input Mode = Controller; reload the page. The WebXR line must read `immersive-ar: yes, immersive-vr: yes`.
3. Click **Start VR**. The emulator overlay shows Controller [L] and Controller [R] panels.
4. On Controller [R]: set **Grip = 1.00** and click the pin icon next to it to latch it (the page turns green = clutch engaged).
5. Change the right controller `Pos` X / Y / Z (or drag the 3D gizmo). The UR5 in RViz follows the relative motion. To clutch (reposition without moving the arm), unpin / set Grip to 0, move, then latch the grip again.

Axis reminder: controller -Z is forward and becomes ROS +x; controller +X is right and becomes ROS -y; +Y is up.

Screenshot (right controller Pos changed to X -0.35 / Y 0.10 / Z -0.40 with Grip pinned at 1.00, RViz arm moved):

## 19. Commit and README update (2026-11-22)

- User committed the emulator fixes: `b8ae5e8` Make bridge and page work with the Immersive Web Emulator. Branch `quest-teleop` is now 3 commits ahead of `main` (`c7030e7` quest_teleop package, `4292d6b` ur5_kinematics lint fix, `b8ae5e8` emulator fixes). Not pushed yet; PR for Jefrey planned for tomorrow (2026-11-23).
- Claude updated `src/quest_teleop/README.md` (uncommitted, for the user to commit): venv build instructions (`python3 -m colcon`), the tested fake_quest result and numeric checks, the full emulator v2.0 procedure (enable per site, device in DevTools, Start VR, pin Grip = 1.00, change Pos), test commands and counts, and a troubleshooting table (aiohttp/venv, greyed-out Start VR, rejected frames, arm not moving, snap/RViz crash).

## 19b. PR opened (2026-11-22)

- User committed the README update, pushed `quest-teleop` to origin (`jefreybulla/ur5_teleop_ws`) and opened **PR #3** into `main`: "Add quest_teleop: Meta Quest 3 controller teleop for the UR5" (4 commits, 27 files, +1,843 / -44). Status: open, **approval required (1 still required)**, no reviewers assigned yet, checks 0 (no CI configured).
- Small things noticed in the screenshot: the title reads "...for the UR5Quest teleop" (a missing space/typo, editable via the pencil icon); reviewer Jefrey still needs to be requested.

## 20. Pending without the Meta Quest

1. DONE: README committed, `quest-teleop` pushed, PR #3 opened. Remaining: fix the PR title typo, request Jefrey as reviewer, and address his review comments.
2. Check the root `README.md` section added by the patch is consistent with the new venv notes (and decide whether the root README should mention them).
3. Review with Jefrey: the `ur5_kinematics` lint commit touches his package (style-only), and the teleop home pose `[0,-90,90,-90,-90,0]` vs the demo `HOME_Q` singularity.
4. Emulator extras (progress 2026-11-23: baseline, `scale` and `operator_yaw_deg` DONE, see section 21; Rot DONE (section 22); remaining: left controller, AR mode, workspace clamp, stale-data failsafe): test AR mode (`Start AR`), left controller, Rot values (orientation following), `scale` and `operator_yaw_deg` launch args, workspace limits (push the target out of reach), and the stale-data failsafe (stop streaming while gripped).
5. Look at `/quest_teleop/metrics` (manipulability) near singular poses while driving with the emulator.
6. URSim hardware-mode test: run URSim + `ur_robot_driver`, switch to `forward_position_controller`, run `quest_teleop mode:=hardware` with low `scale` / `v_max`. Verify the controller name and command topic against the course driver config. (No headset needed: use the emulator or fake_quest as the input.)
7. Record a short video / screenshots of RViz + emulator for the report, and add the final screenshots to the instructions.
8. Optional code ideas: `fake_quest` integration test in CI, document `rtt` / latency measurement, and a launch argument to start without RViz.

## PENDING (needs the Meta Quest 3)

- Real Quest 3 over USB: Developer Mode, `adb devices`, `adb reverse tcp:8080 tcp:8080`, Quest Browser -> `http://localhost:8080/` -> Start AR; check green tint, clutching, and tune `operator_yaw_deg`.
- Measure real latency / frame rate, then real UR5 with the safety checklist.

## 21. Emulator tests run by the user (2026-11-23)

Status: PR #3 title typo fixed and Jefrey is a requested reviewer (approval pending). Tests below were run by the user (Claude only gave the commands), with the emulator driving RViz.

**Reading `/quest_teleop/metrics`:** it is `[pos error m, rot error rad, manipulability]` (`Float64MultiArray`, values unlabelled). Baseline at rest: pos error 0.00292 m (2.9 mm), rot error 0.024 rad (about 1.4 deg), manipulability 0.0367. Identical repeated messages are normal when the arm has converged on a still target. Labelled view without code changes:
`ros2 topic echo /quest_teleop/metrics --field data --csv | awk -F, '{printf "pos_err=%.4f m  rot_err=%.4f rad  manip=%.4f\n",$1,$2,$3}'`
(Decision: do not add labels or a custom msg now; a custom `.msg` is the idiomatic fix if wanted later.)

**Test 0, baseline: PASS.** Arm follows the emulator controller.

**Test 1, `scale:=0.5`: PASS.** Controller Pos X +0.10 -> goal y changed by exactly -0.0500 (x, z unchanged). Controller +X maps to ROS -y. A first run gave -0.047 because the Pos field was not exactly at the typed start value; re-run with `/quest/right/pose` checked gave exactly -0.0500. Goal formula (no smoothing): `p_goal = clamp(p_ee0 + scale * R_align @ (p_c - p_c0))`.

**Test 2, `operator_yaw_deg:=90`: PASS.** Pos X +0.10 (scale 1.0) -> goal x +0.1000, y and z unchanged (before x 0.2369, after 0.3369). A +90 deg yaw takes -y to +x, as predicted.

**What `operator_yaw_deg` is (important for the headset):**
- It builds `R_align = R_BASE_FROM_BASE_LINK @ rot_z(yaw)` (`teleop_core.py`) and rotates the controller's motion about z before it is applied to the tool. Orientation changes are conjugated by the same matrix.
- It does NOT rotate the robot, its base frame, TF or URDF, and the arm does not move when it changes. It only changes what a given hand motion means: it is an operator-to-robot calibration.
- It exists because `quest_world` +x is the direction the headset faced when the WebXR session started, while the robot +x is fixed by how it is mounted. If the user starts the session facing a different direction than the robot's +x, a forward push will not move the tool forward. Fix: relaunch with `operator_yaw_deg:=90`, `180` or `-90` until pushing the controller away moves the tool away. Re-check it every time the session is started from a different heading.

Note on test procedure: after a page reload the emulator Pos can come back at a default (0.25); if Grip is pinned before setting clean Pos values the arm follows that jump. Set Pos first, then pin Grip.

Next: Test 3 (Rot / orientation following), then left controller, AR mode, workspace clamp, stale-data failsafe.

## 22. Test 3, orientation following (Rot) and what the topic values mean (2026-11-23)

**Test 3: PASS.** Orientation of the goal follows the controller 1:1.

Findings (all verified with `ros2 topic echo` + a quaternion calculation):
1. **The emulator's Rot fields are in RADIANS**, not degrees. Rot Y = 30 gave an 81.13 deg rotation (30 rad mod 2*pi = -81.13 deg). Rot Y = 0.52 gave 29.79 deg. Use 0.52 for about 30 deg, 1.571 for 90 deg.
2. **Rot Y is yaw (rotation about the vertical axis).** The emulator uses WebXR/three.js axes (x right, y up, z back), so Rot X = rotation about the right axis (pitch), Rot Y = about the up axis (yaw), Rot Z = about the back axis (roll). Roll/pitch/yaw = XYZ only holds in ROS axes (x forward, y left, z up).
3. **Goal rotation = controller rotation exactly** (81.127 deg and 29.794 deg, both about +z, identical for `/quest/right/pose` and `/quest_teleop/goal_pose`).
4. **Pos = (0,0,0) in the emulator does not give zeros on `/quest/right/pose`.** At Pos 0 the topic reads about (-0.05, -0.004, 1.584). `/quest/right/pose` is the grip pose in `quest_world` (floor origin, z up): the 1.584 m is about head height, so the emulator's Pos appears to be relative to the headset (defaults 0.25 / -0.10 / -0.40 = right / below / in front of the head; not yet confirmed with `/quest/head/pose`). The grip point also sits about 5 cm from the controller origin. This is irrelevant for teleop because the clutch only uses differences: `p_goal = clamp(p_ee0 + scale * R_align @ (p_now - p_at_engage))`.
5. **Rotating the controller also moves its position** (grip offset lever arm of about 5 cm): 0.52 rad gave a 2.58 cm shift (predicted 2*0.05*sin(15 deg) = 2.59 cm), 30 rad gave 6.46 cm. This is real physical behaviour and will also happen with the real Quest when the wrist rotates.
6. At Rot 0 the grip orientation is not identity (about x -0.02, y -0.38, z -0.06, w 0.92): the controller model's grip frame is tilted. Irrelevant for teleop, which uses relative rotation `r_goal = (R_align * (R_now * R_at_engage^T) * R_align^T) * r_ee0`.
7. Axis check from the baseline: controller -Z (forward) = ROS +x, +X (right) = ROS -y, +Y (up) = ROS +z (goal moved (-0.40, +0.25, +0.10) for controller deltas (Z +0.40, X -0.25, Y +0.10)).

Next: left controller (`hand:=left`), AR mode, workspace clamp, stale-data failsafe.

## 23. Test 4, left controller (`hand:=left`), in progress (2026-11-23)

- Launch OK: `Quest teleop ready: mode=sim, hand=left`; bridge receives 60 Hz.
- **Confirmed: emulator Pos is relative to the headset.** `/quest/right/pose` at the default Pos (0.25, -0.10, -0.40) read (0.350, -0.254, 1.484), exactly the prediction from head at about (0, 0, 1.584) + grip offset about 5 cm: ROS x = +0.40 - 0.05, y = -0.25, z = 1.584 - 0.10.
- **Page does not turn green with the left grip.** Cause: `web/index.html` line 171 computes the tint only from the RIGHT controller's squeeze (`right.buttons[1][0]`). Cosmetic only; the teleop node itself uses the configured hand. Possible fix: make the tint follow the selected hand or any grip (decide with Jefrey; the page is part of PR #3). The dark (not red) background means the WebSocket is connected.
- Pitfall: echo commands copied from Test 3 still pointed at `/quest/right/pose`. For the left test use `/quest/left/pose` and `/quest_teleop/goal_pose --field pose.position`.
- **Test 4 left controller: PASS (motion).** Left Pos Z -0.10 -> 0: `/quest/left/pose` x 0.0504 -> -0.0496 (-0.1000), goal x 0.18668 -> 0.08668 (-0.1000), y and z unchanged. Same mapping as the right hand. At Pos 0 the left grip offset is (-0.0496, +0.0040, 1.584), the mirror image in y of the right one (-0.0496, -0.0038).
- Still to check: that the right controller does NOT move the arm while `hand:=left`.
- **Control check: PASS.** With `hand:=left`, the right controller's Pos Z -0.40 -> 0.20 (0.6 m) did not move the goal (x 0.0867 -> 0.0896, i.e. 3 mm, y and z unchanged; a repeat echo was identical). The 3 mm is not from the right hand (a right-hand effect would be 0.6 m); most likely the left Pos/Rot fields were re-typed in between (they show `0` instead of `0.00`). Pinning the RIGHT grip made the page turn green even though `hand=left`, which confirms the tint is right-only (index.html line 171).
- Test 4 DONE. Next: Test 5 (AR mode), workspace clamp, stale-data failsafe.

## 24. Test 5, AR mode (emulator): PASS (2026-11-23)

- `Start AR` works in the Immersive Web Emulator with `hand:=right` (default). Right Pos Z -0.40 -> -0.50 (grip pinned): goal x 0.4869 -> 0.5869 (+0.1000), y and z unchanged. Same mapping as VR mode.
- In AR the emulator shows a synthetic room mesh (the "rugged wall") as passthrough; the clutch tint is a translucent green over it (alpha 0.25 in `index.html`), versus an opaque green in VR. So the tint is visible but subtle in AR: on the real Quest it will be a light green wash over the real room.
- Next: workspace clamp (limits in `teleop_core.py`: r_max 0.80 m from the shoulder, r_min_xy 0.15 m around the joint-1 axis, z 0.05 to 0.90 m), then the stale-data failsafe.

## 25. Test 6, workspace clamp (2026-11-23)

- **Reach limit r_max 0.80 m: PASS.** Right Pos Z -0.40 -> -1.50: unclamped goal would be (1.5869, 0.1092, 0.4319) (1.627 m from the shoulder, shoulder at z = 0.089159). Echoed goal (0.78021, 0.05366, 0.25765), 0.8000 m from the shoulder, equal to the predicted scaled-down vector to 8 digits. The clamp scales the whole shoulder-to-goal vector, so y and z shrink too.
- **Floor limit z_min 0.05 m: PASS.** Pos Y -0.10 -> -1.0: goal (0.4869, 0.10915, 0.05).
- **Inner cylinder r_min_xy 0.15 m: NOT triggered yet.** Pos Z -0.40 -> +0.40 is a 0.80 m move, so the goal went to x = -0.3131 (behind the base, horizontal distance 0.33 m, outside the cylinder). To enter it, use Pos Z = 0.0 (move 0.40, goal about x 0.087, y 0.109, horizontal distance 0.139); the clamp should push it to (0.0934, 0.1173), horizontal distance 0.150.
- **Inner cylinder r_min_xy 0.15 m: PASS (re-run with Pos Z = 0.0).** Goal (0.09343, 0.11735, 0.43186), horizontal distance 0.15, exactly the predicted (0.0934, 0.1173). All three workspace clamps (reach, floor, inner cylinder) verified. Next: Test 7, stale-data failsafe.

## 26. Test 7, stale-data failsafe: PASS (2026-11-23)

- Quit the emulator session with the right Grip pinned (launch kept running). Bridge log: last full 60 Hz window ends at t=164.7, the next window reports 43 Hz (data stopped about 1.4 s into it, around t=166.17), and `[quest_teleop] Clutch released (Quest data stale)` appears at t=166.42, i.e. about 0.25 s after the last frame, matching the `stale` limit. The arm did not move. The bridge then logged `No controller data for N s` every 2 s (up to 122 s).
- `ros2 topic echo /quest_teleop/goal_pose --once` hung with no output while stale: the node only publishes the goal while the clutch is engaged, and `/joint_states` keeps publishing the last pose, so the arm holds still.
- Re-entering the emulator (`Start AR`): `[page] session started: immersive-ar, ref space local-floor`, then `Clutch engaged` 25 ms later, because the grip was still pinned. The clutch re-engages relative to the current pose, so there is no jump. On the real Quest this means: if the grip is held when the session starts or reconnects, the arm is live immediately; start sessions with the grip released.
- Page status after the session ended: WebSocket connected, 60.3 Hz send rate, 0.5 ms round trip (localhost), right grip p [+0.254, +1.484, +0.050] / left [-0.254, +1.484, -0.350] in WebXR axes (x right, y up, z back), consistent with head-relative Pos plus the grip offset.

## Emulator test summary (2026-11-23)

All planned emulator tests passed: baseline, scale, operator_yaw_deg, Rot (orientation), left controller, AR mode, workspace clamp (reach, floor, inner cylinder), stale-data failsafe. Findings to remember for the headset: emulator Rot is in radians; Pos is head-relative; page tint follows only the right grip; teleop starts live if grip is held at session start.

## 27. Green tint fix, status message for Jefrey, plan for the Quest trials (2026-11-23)

**Green tint fix (uncommitted, for the user to commit):** `web/index.html` now tints green when the grip of EITHER controller is held (before: right only). Optional `?hand=left` / `?hand=right` in the URL restricts the tint to one hand, matching the node's `hand` argument. The hint text on the page was updated. The page is served straight from the source folder, so no rebuild is needed: reload the page (verify with `hand:=left` + left grip: the view must turn green). Not yet re-tested in the emulator; JS syntax not machine-checked (no node installed).

**Status message for Jefrey (draft):**

> Hi Jefrey, quick status on the Quest teleop (PR #3, `quest-teleop` branch, waiting for your review).
> - What it is: `quest_teleop` package. A WebXR page served by a ROS node streams the Quest controller pose and buttons over WebSocket at 60-72 Hz into ROS 2; squeeze is a clutch, and our resolved-rates IK from `ur5_kinematics` drives the UR5 in RViz. No Unity or APK needed.
> - Tested today without the headset, using Meta's Immersive Web Emulator, and all checks passed: scale, operator yaw calibration, orientation following, left and right hand, AR mode, workspace limits (reach 0.80 m, floor 0.05 m, inner cylinder 0.15 m), and the stale-data failsafe (clutch opens about 0.25 s after the data stops, arm freezes). Also: build and 17 colcon tests pass.
> - For your review: the lint commit touches your `ur5_kinematics` (style only, no logic change), and the teleop start pose `[0,-90,90,-90,-90,0]` differs from your demo `HOME_Q` (q5 = 0 is a wrist singularity), so the arm jumps once when teleop starts.
> - Not tested yet: the real Quest 3, URSim, and the real UR5. Tomorrow we do the first trials on the Quest (sim only, no real robot).
> - Please approve or comment on the PR when you can.

**Plan for tomorrow (first Quest 3 trials, simulation only):**

0. Before leaving: `sudo apt install adb` (adb is NOT installed on this laptop). Charge the Quest, bring a USB-C cable that carries data. Developer Mode must be enabled for the headset (Meta Horizon app, needs a developer organisation/account on the phone). Keep the laptop in the same venv as today.
1. USB path: plug in, accept the USB-debugging prompt in the headset, `adb devices` must list it, then `adb reverse tcp:8080 tcp:8080`.
2. `ros2 launch quest_teleop quest_teleop_sim.launch.py` (add `hand:=left` if needed). Quest Browser -> `http://localhost:8080/` -> check `immersive-ar: yes` -> Start AR.
3. First checks (grip RELEASED when starting the session; teleop is live as soon as the grip is held): `ros2 topic hz /quest/right/pose` (expect 72 Hz), `/quest/rtt_ms`, bridge log without seq gaps.
4. Calibrate `operator_yaw_deg`: push the controller away from you; the tool should move away from you. If not, relaunch with 90 / 180 / -90. Re-check if the headset faces a different way when the session starts.
5. Clutching and workspace: move, release, reposition, grip again; check the green tint (reload the page first to get the tint fix), and that the red tint appears if the WebSocket drops.
6. Orientation: twist the wrist; check manipulability with `ros2 topic echo /quest_teleop/metrics --field data --csv` near stretched and tilted poses.
7. Measure latency (rtt) and write numbers, take screenshots / a short video for the report.
8. Fallback if adb fails: Wi-Fi + self-signed HTTPS (README section 3B), needs client-to-client traffic allowed.
9. Only after that, and with Jefrey: URSim hardware mode with low `scale` / `v_max`, then the real UR5 with the e-stop in hand. Not tomorrow.

## 28. Tint fix verified, README update, Quest guide (2026-11-23)

- User verified the green tint now works with the left controller (emulator).
- Claude updated `src/quest_teleop/README.md` (uncommitted): emulator quirks (Rot in radians, Pos head-relative, grip offset), tests done, tint for either hand / `?hand=`, start with grip released. Together with `web/index.html` these are the two files to commit and push to PR #3 (user does the git steps).
- Step-by-step guide for the headset session written to `quest3_trial_guide.md` (same folder as this log): adb install, Developer Mode, adb reverse, checks, operator_yaw_deg calibration, what to record, troubleshooting. adb is NOT installed on the laptop yet (user is installing it).
- adb installed on the laptop (2026-11-23): Android Debug Bridge 1.0.41, version 34.0.4-debian, `/usr/lib/android-sdk/platform-tools/adb`. Not yet tested with the headset.
- Meta Horizon app login failed for the user on the iPhone (2026-11-23 evening). Fallback documented in the guide, section 0.5: Wi-Fi + HTTPS needs no Developer Mode or adb.

## 29. Headset access (2026-11-23 evening)

- The Quest 3 is a shared lab headset, signed in with a lab account, so Developer Mode has to be enabled by whoever owns that account. It turned out to be already on (see section 30). The Wi-Fi + HTTPS path (no Developer Mode, no adb) is documented as a fallback in the trial guide.

## 30. First real Quest 3 session (2026-11-24)

- Headset (lab Quest, serial omitted) showed up in `adb devices` with Developer Mode already on; first it said `no permissions`, fixed with the udev rule `/etc/udev/rules.d/51-meta-quest.rules` (vendor 2833, MODE 0666, GROUP plugdev), `udevadm control --reload-rules && udevadm trigger`, `adb kill-server`, replug, accept the USB-debugging dialog. `adb reverse tcp:8080 tcp:8080` -> `adb reverse --list` shows `UsbFfs tcp:8080`.
- Quest Browser at `http://localhost:8080/`, Start AR: bridge reports **90.0 Hz, 0 seq gaps, 0 bad msgs** (the emulator gave 60 Hz).
- **Latency (`/quest/rtt_ms`): 1.7 to 3.7 ms, typical about 2.3 ms** (page <-> bridge round trip over USB, not including IK / RViz).
- **Clutch: OK.** Squeeze engages (green tint, `Clutch engaged`), release freezes the arm, re-engage does not jump. Both grips tint green (tint fix); only the right hand drives the arm (`hand:=right`). Occasional gaps while moving (5 to 8 seq gaps per 2 s window, about 3 to 4 %, rate 86 to 87 Hz), probably brief controller tracking loss; gone in the next window.
- **Direction calibration: `operator_yaw_deg:=0` is right** for facing the laptop screen when starting the AR session. Pushing the right controller about 17 cm toward the screen moved the goal from x 0.5407 to x 0.7146 (y 0.162 -> 0.161, z 0.433 -> 0.411; the small z drop is the hand sagging). The robot moved forward in RViz.
- Hand/tracking jitter at rest is about 1 to 2 mm peak-to-peak on the goal.
- Tip: piping `ros2 topic echo` into `awk` printed nothing because of output buffering; use `PYTHONUNBUFFERED=1` and `fflush()` or read the stream without a pipe.
- Remaining checks in the guide: scale feel, orientation (wrist twist), workspace limits with real motion, failsafe (unplug / quit session), manipulability near singular poses, screenshots and a video for the report.

## 31. Quest 3 checks, part 2 (2026-11-24)

User results (sim, real headset):
- Scale feel (`scale:=1.0`): comfortable in simulation; still to judge with the real robot.
- Failsafe: arm freezes when the AR session is disconnected. Workspace limits: arm stops at the limits, does not chase.
- Orientation: works but lags (user: "functional, not at the level we want").
- `/quest_teleop/metrics` while stretching: pos error up to about 0.017 m, rot error up to about 0.06 rad, manipulability 0.068 -> 0.081 (rising, nowhere near a singularity).
- Metrics while tilting the wrist at about 1 Hz: pos error stays below 0.011 m, manipulability 0.061 to 0.093 (no singularity), but **rotation error swings up to about 1.83 rad (105 deg)**, typically 0.5 to 1.0 rad.
- Reading: the arm is rate-limited in orientation, not near a singularity. Defaults in `teleop_node.py`: `w_max` 1.0 rad/s (tool angular speed cap), `max_joint_speed` 1.0 rad/s per joint, `lambda_w` 5.0, `eps_w` 0.03, `v_max` 0.25 m/s. A 1 Hz wrist oscillation of +-60 deg needs about 6.6 rad/s peak, so the 1 rad/s caps dominate. These caps are safety features for the real robot; raising them is a sim-only experiment first.
- The sim launch only exposes `hand`, `scale`, `operator_yaw_deg`, `port`, `certfile`, `keyfile`; tuning `w_max` / `max_joint_speed` needs new launch arguments (parameters are read once at node start).

## 32. Speed-limit launch arguments and repo copy of this log (2026-11-24)

- `quest_teleop_sim.launch.py` now exposes `v_max` (default 0.25 m/s), `w_max` (default 1.0 rad/s) and `max_joint_speed` (default 1.0 rad/s); defaults equal the node's, so behaviour is unchanged unless set. Example for the sim-only wrist-lag experiment: `ros2 launch quest_teleop quest_teleop_sim.launch.py w_max:=3.0 max_joint_speed:=2.0`. The file compiles; not yet run (uncommitted, for the user to commit). For URSim / the real UR5 keep the conservative defaults or lower them.
- The Claude memory of this project is NOT in the repo: it lives outside, at `~/.claude/projects/<workspace path>/memory/` (index `MEMORY.md` plus three notes: git flow, ROS env / venv, quest_teleop status). The repo has no CLAUDE.md or docs folder, so a sanitized copy of this log is kept in the repo at `docs/quest_teleop_integration_log.md` (images, personal details and the headset serial removed because the repo is public). This OneDrive file stays the full original.
