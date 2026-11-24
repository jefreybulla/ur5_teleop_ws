"""Message format between the WebXR page and the bridge (pure Python, no ROS).

The page sends one JSON message per XR frame:

    {
      "type": "pose", "seq": 123, "t_client_ms": 4567.8, "rtt_ms": 21.0,
      "ref_space": "local-floor",
      "head": {"p": [x, y, z], "q": [x, y, z, w]},
      "controllers": {
        "right": {
          "grip": {"p": [...], "q": [...]},
          "aim":  {"p": [...], "q": [...]},          # optional
          "lin_vel": [...], "ang_vel": [...],          # optional (m/s, rad/s)
          "buttons": [[pressed, value], ...],          # xr-standard order
          "axes": [a0, a1, a2, a3]
        },
        "left": {...}
      }
    }

All poses arrive in WebXR axes and are converted to ROS axes here.

xr-standard gamepad mapping on Quest Touch controllers:
    buttons[0] trigger, [1] squeeze (grip), [3] thumbstick press,
    [4] A / X, [5] B / Y;  axes[2], axes[3] thumbstick x / y.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from quest_teleop.frames import xr_position_to_ros, xr_quat_to_ros

BTN_TRIGGER = 0
BTN_SQUEEZE = 1
BTN_THUMBSTICK = 3
BTN_A_X = 4
BTN_B_Y = 5


@dataclass
class Pose:
    p: np.ndarray  # (3,) ROS axes [m]
    q: np.ndarray  # (4,) [x, y, z, w] ROS axes


@dataclass
class ControllerSample:
    hand: str
    grip: Pose
    aim: Optional[Pose] = None
    lin_vel: Optional[np.ndarray] = None
    ang_vel: Optional[np.ndarray] = None
    buttons_pressed: List[bool] = field(default_factory=list)
    buttons_value: List[float] = field(default_factory=list)
    axes: List[float] = field(default_factory=list)

    def pressed(self, index: int) -> bool:
        return index < len(self.buttons_pressed) and self.buttons_pressed[index]

    def value(self, index: int) -> float:
        return self.buttons_value[index] if index < len(self.buttons_value) else 0.0


@dataclass
class QuestFrame:
    seq: int
    t_client_ms: float
    rtt_ms: Optional[float]
    ref_space: str
    head: Optional[Pose]
    controllers: Dict[str, ControllerSample]


def _pose(d: Optional[dict]) -> Optional[Pose]:
    if not d or 'p' not in d or 'q' not in d:
        return None
    return Pose(p=xr_position_to_ros(d['p']), q=xr_quat_to_ros(d['q']))


def _num(x, default: float = 0.0) -> float:
    """Float that tolerates null/NaN (JSON.stringify turns NaN into null; the emulator sends some)."""
    try:
        v = float(x)
    except (TypeError, ValueError):
        return default
    return v if np.isfinite(v) else default


def _vec(v) -> Optional[np.ndarray]:
    if v is None:
        return None
    return xr_position_to_ros(v)  # velocities transform like positions


def parse_frame(msg: dict) -> QuestFrame:
    """Validate and convert one page message. Raises ValueError if malformed."""
    if msg.get('type') != 'pose':
        raise ValueError(f'not a pose message: {msg.get("type")!r}')
    controllers: Dict[str, ControllerSample] = {}
    for hand, c in (msg.get('controllers') or {}).items():
        grip = _pose(c.get('grip'))
        if grip is None:
            continue
        buttons = c.get('buttons') or []
        controllers[hand] = ControllerSample(
            hand=hand,
            grip=grip,
            aim=_pose(c.get('aim')),
            lin_vel=_vec(c.get('lin_vel')),
            ang_vel=_vec(c.get('ang_vel')),
            buttons_pressed=[bool(b[0]) for b in buttons],
            buttons_value=[_num(b[1]) for b in buttons],
            axes=[_num(a) for a in (c.get('axes') or [])],
        )
    rtt = msg.get('rtt_ms')
    return QuestFrame(
        seq=int(msg.get('seq', 0)),
        t_client_ms=float(msg.get('t_client_ms', 0.0)),
        rtt_ms=float(rtt) if rtt is not None else None,
        ref_space=str(msg.get('ref_space', '')),
        head=_pose(msg.get('head')),
        controllers=controllers,
    )


__all__ = [
    'BTN_TRIGGER', 'BTN_SQUEEZE', 'BTN_THUMBSTICK', 'BTN_A_X', 'BTN_B_Y',
    'Pose', 'ControllerSample', 'QuestFrame', 'parse_frame',
]
