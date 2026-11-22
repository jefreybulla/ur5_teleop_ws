"""HTTP(S) + WebSocket server for the Quest WebXR page (no ROS dependency).

Serves ``web/index.html`` and accepts pose frames on ``/ws``. One server
handles both so the WebSocket is always same-origin (wss:// when HTTPS).

Standalone use, handy for checking the page/emulator without ROS:

    python3 -m quest_teleop.pose_server --port 8080
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import ssl
import threading
import time
from typing import Callable, Optional

from aiohttp import WSMsgType, web

from quest_teleop.protocol import QuestFrame, parse_frame

DEFAULT_WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')

FrameCallback = Callable[[QuestFrame], None]
LogCallback = Callable[[str], None]


class PoseServer:
    def __init__(
        self,
        on_frame: FrameCallback,
        host: str = '0.0.0.0',
        port: int = 8080,
        certfile: str = '',
        keyfile: str = '',
        web_dir: str = DEFAULT_WEB_DIR,
        log: LogCallback = print,
    ) -> None:
        self._on_frame = on_frame
        self._host = host
        self._port = port
        self._certfile = certfile
        self._keyfile = keyfile
        self._web_dir = web_dir
        self._log = log
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._runner: Optional[web.AppRunner] = None
        self._thread: Optional[threading.Thread] = None
        self._ready = threading.Event()
        self._start_error: Optional[BaseException] = None
        self.clients = 0
        self.frames_received = 0
        self.bad_frames = 0

    # ---------------------------------------------------------------- HTTP
    async def _index(self, _request: web.Request) -> web.StreamResponse:
        return web.FileResponse(os.path.join(self._web_dir, 'index.html'))

    async def _ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=5.0, max_msg_size=1 << 20)
        await ws.prepare(request)
        self.clients += 1
        self._log(f'WebXR client connected from {request.remote} ({self.clients} total)')
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    self.bad_frames += 1
                    continue
                kind = data.get('type')
                if kind == 'ping':
                    # Echo client timestamp so the page can measure round-trip time.
                    await ws.send_str(json.dumps(
                        {'type': 'pong', 't': data.get('t'), 'server_time': time.time()}
                    ))
                elif kind == 'pose':
                    try:
                        frame = parse_frame(data)
                    except (ValueError, KeyError, TypeError, IndexError):
                        self.bad_frames += 1
                        continue
                    self.frames_received += 1
                    self._on_frame(frame)
                elif kind == 'log':
                    self._log(f'[page] {data.get("msg")}')
        finally:
            self.clients -= 1
            self._log(f'WebXR client disconnected ({self.clients} remaining)')
        return ws

    def _make_app(self) -> web.Application:
        app = web.Application()
        app.router.add_get('/', self._index)
        app.router.add_get('/ws', self._ws)
        app.router.add_static('/static/', self._web_dir)
        return app

    def _ssl_context(self) -> Optional[ssl.SSLContext]:
        if not self._certfile:
            return None
        ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ctx.load_cert_chain(self._certfile, self._keyfile or None)
        return ctx

    @property
    def url(self) -> str:
        scheme = 'https' if self._certfile else 'http'
        return f'{scheme}://{self._host}:{self._port}/'

    # ------------------------------------------------------------ lifecycle
    async def _start(self) -> None:
        self._runner = web.AppRunner(self._make_app())
        await self._runner.setup()
        site = web.TCPSite(self._runner, self._host, self._port,
                           ssl_context=self._ssl_context())
        await site.start()

    def _thread_main(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._start())
        except BaseException as exc:  # report bind/cert errors to caller
            self._start_error = exc
            self._ready.set()
            return
        self._ready.set()
        self._loop.run_forever()
        self._loop.run_until_complete(self._runner.cleanup())
        self._loop.close()

    def start(self) -> None:
        """Start serving in a background thread; raises if the port/cert fails."""
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()
        self._ready.wait(timeout=10.0)
        if self._start_error is not None:
            raise self._start_error
        self._log(f'Serving Quest page at {self.url}')

    def stop(self) -> None:
        if self._loop is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=5.0)


def main() -> None:
    parser = argparse.ArgumentParser(description='Quest WebXR pose server (no ROS)')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--cert', default='')
    parser.add_argument('--key', default='')
    parser.add_argument('--hand', default='right')
    args = parser.parse_args()

    last_print = [0.0]

    def on_frame(frame: QuestFrame) -> None:
        now = time.time()
        if now - last_print[0] < 0.5:
            return
        last_print[0] = now
        c = frame.controllers.get(args.hand)
        if c is None:
            print(f'seq {frame.seq}: no {args.hand} controller')
            return
        p, q = c.grip.p, c.grip.q
        rtt = f'{frame.rtt_ms:.1f}' if frame.rtt_ms is not None else '-'
        print(f'seq {frame.seq:6d} {args.hand} p=[{p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f}] '
              f'q=[{q[0]:+.3f} {q[1]:+.3f} {q[2]:+.3f} {q[3]:+.3f}] '
              f'trig={c.value(0):.2f} grip={c.value(1):.2f} rtt={rtt} ms')

    server = PoseServer(on_frame, args.host, args.port, args.cert, args.key)
    server.start()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == '__main__':
    main()
