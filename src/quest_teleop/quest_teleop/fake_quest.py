"""Headless stand-in for the Quest page: streams a synthetic controller motion.

Sends exactly the same WebSocket messages as web/index.html, so it exercises
the bridge and teleop nodes with no headset and no browser.

    ros2 run quest_teleop fake_quest --url ws://localhost:8080/ws

Motion (WebXR axes): the right grip traces a horizontal circle of --radius
in front of the user at 1.1 m height while slowly rolling +/-20 deg. Squeeze
is held, except it is released for 1 s every 6 s to exercise the clutch.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import time

import aiohttp


def _quat_axis_angle(axis, angle):
    s = math.sin(angle / 2.0)
    return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0)]


def make_frame(seq: int, t: float, radius: float, period: float, rtt_ms) -> dict:
    w = 2.0 * math.pi / period
    p = [radius * math.cos(w * t), 1.1, -0.4 + radius * math.sin(w * t)]
    q = _quat_axis_angle([0.0, 0.0, 1.0], math.radians(20.0) * math.sin(w * t))  # roll about xr z
    squeeze = (t % 6.0) < 5.0
    return {
        'type': 'pose', 'seq': seq, 't_client_ms': t * 1000.0, 'rtt_ms': rtt_ms,
        'ref_space': 'local-floor',
        'head': {'p': [0.0, 1.6, 0.0], 'q': [0.0, 0.0, 0.0, 1.0]},
        'controllers': {
            'right': {
                'grip': {'p': p, 'q': q},
                'buttons': [[False, 0.0], [squeeze, 1.0 if squeeze else 0.0],
                            [False, 0.0], [False, 0.0], [False, 0.0], [False, 0.0]],
                'axes': [0.0, 0.0, 0.0, 0.0],
            },
        },
    }


async def run(url: str, rate: float, duration: float, radius: float, period: float) -> int:
    rtt = {'ms': None}
    sent = 0
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(url, ssl=False) as ws:
            async def reader():
                async for msg in ws:
                    if msg.type == aiohttp.WSMsgType.TEXT:
                        m = json.loads(msg.data)
                        if m.get('type') == 'pong':
                            rtt['ms'] = (time.perf_counter() * 1000.0) - m['t']
            reader_task = asyncio.create_task(reader())
            t0 = time.perf_counter()
            next_ping = 0.0
            while True:
                t = time.perf_counter() - t0
                if duration > 0 and t > duration:
                    break
                if t >= next_ping:
                    await ws.send_str(json.dumps({'type': 'ping', 't': time.perf_counter() * 1000.0}))
                    next_ping = t + 0.5
                await ws.send_str(json.dumps(make_frame(sent, t, radius, period, rtt['ms'])))
                sent += 1
                await asyncio.sleep(max(0.0, (sent / rate) - (time.perf_counter() - t0)))
            reader_task.cancel()
    rtt_txt = f'{rtt["ms"]:.2f} ms' if rtt['ms'] is not None else 'n/a'
    print(f'fake_quest: sent {sent} frames, last RTT {rtt_txt}')
    return sent


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--url', default='ws://localhost:8080/ws')
    ap.add_argument('--rate', type=float, default=72.0, help='Hz (Quest 3 default refresh)')
    ap.add_argument('--duration', type=float, default=0.0, help='seconds, 0 = forever')
    ap.add_argument('--radius', type=float, default=0.10, help='circle radius [m]')
    ap.add_argument('--period', type=float, default=8.0, help='circle period [s]')
    args, _ = ap.parse_known_args()  # ignore --ros-args when started via ros2 run
    try:
        asyncio.run(run(args.url, args.rate, args.duration, args.radius, args.period))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
