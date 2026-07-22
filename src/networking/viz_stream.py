"""Live visualization stream: full scene frames over one websocket.

Publishes the whole scene (per-drone poses, plus any annotation a client injects)
as JSON frames at a fixed SIM-time rate, for the self-contained viewer page
`viz/drone_viewer.html`. It runs alongside (not instead of) the per-drone
ground-truth stream in `websocket_publisher.py`: separate port, separate schema.

Messages (server -> viewer):
  {"type":"meta", "n_drones":4, "spacing":2.0, "source":"main_multi",
   "stack":[...]}                                              (~1 Hz, and on demand)
  {"type":"frame", "t":12.34, "rtf":1.0,
   "drones":[{"p":[N,E,D], "q":[w,x,y,z]},...],
   "target":[N,E,D] | absent, "phase":"..." | absent}          (30 Hz sim)

Any JSON a client sends is kept in `inbox` by its "type" (e.g. an offboard node
injecting {"type":"target","p":[...],"phase":"HOVER"}); a fresh target/phase from
the inbox is merged into outgoing frames, so a separate planner process can
annotate the scene without touching the sim loop.

Recording: set record_path (env SIM_VIZ_RECORD) to also append every frame as
JSON-lines; the viewer page loads such a file for offline playback (same schema
as live). The first line is the meta message.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from typing import Any

try:
    import websockets
    import asyncio
except ImportError:  # pragma: no cover
    websockets = None

logger = logging.getLogger(__name__)


class VizStream:
    """Broadcast sim frames to viewer clients; collect client-injected annotations."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8766, rate_hz: float = 30.0,
                 record_path: str = "", source: str = "sim", enabled: bool = True):
        self.enabled = bool(enabled) and websockets is not None
        if enabled and websockets is None:
            logger.warning("viz stream disabled: 'websockets' package not installed")
        self.host, self.port = host, int(port)
        self.period = 1.0 / float(rate_hz)
        self.source = source
        self._meta: str | None = None
        self._meta_dict: dict | None = None
        self._meta_resend = 0.0    # periodic re-broadcast: a dropped meta after a
        self._next_t = -1e9        # topology change otherwise breaks connected viewers
        self._next_meta_t = -1e9
        self.inbox: dict[str, dict] = {}          # latest client message per "type"
        self._inbox_ts: dict[str, float] = {}     # receipt time; stale entries don't merge
        self._inbox_lock = threading.Lock()
        self._clients: set[Any] = set()
        self._loop = None
        self._queue = None
        self._thread: threading.Thread | None = None
        self._rec = open(record_path, "w") if record_path else None
        self._wall0: float | None = None
        self._sim0 = 0.0
        self._rtf = 0.0

    # -- lifecycle -------------------------------------------------------
    def start(self) -> None:
        if not self.enabled:
            return
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()
        logger.info("viz stream on ws://%s:%d (viewer: viz/drone_viewer.html)", self.host, self.port)

    def stop(self) -> None:
        if self._rec is not None:
            self._rec.close()
            self._rec = None
        loop, queue = self._loop, self._queue
        if not self.enabled or loop is None or queue is None or loop.is_closed():
            return
        try:
            loop.call_soon_threadsafe(lambda: queue.put_nowait(None))
        except RuntimeError:
            return
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)

    def _thread_main(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._queue = asyncio.Queue(maxsize=4)
        try:
            self._loop.run_until_complete(self._serve())
        except OSError as exc:
            logger.error("viz stream failed on ws://%s:%d: %s", self.host, self.port, exc)
            logger.error("Another process already owns this port; free it or run with "
                         "SIM_VIZ_WS_PORT=%d.", self.port + 1)
            self.enabled = False
        except Exception:
            logger.exception("viz stream thread crashed")
            self.enabled = False
        finally:
            if self._loop is not None and not self._loop.is_closed():
                self._loop.close()
            self._loop = None
            self._queue = None
            self._clients.clear()

    async def _serve(self) -> None:
        async def handler(ws):
            self._clients.add(ws)
            try:
                if self._meta is not None:
                    await ws.send(self._meta)
                async for raw in ws:                      # client-injected annotations
                    try:
                        msg = json.loads(raw)
                        if isinstance(msg, dict) and "type" in msg:
                            with self._inbox_lock:
                                self.inbox[str(msg["type"])] = msg
                                self._inbox_ts[str(msg["type"])] = time.monotonic()
                    except (json.JSONDecodeError, TypeError):
                        pass
            except Exception:      # client vanished (ping timeout etc.): not our problem
                pass
            finally:
                self._clients.discard(ws)

        # ping_interval=None: under HIL load this thread can starve past the 20 s
        # keepalive window and the server would hard-drop clients. Dead clients are
        # collected via send-failure below; keepalive adds nothing but the drop.
        async with websockets.serve(handler, self.host, self.port, ping_interval=None):
            while True:
                msg = await self._queue.get()
                if msg is None:
                    break
                if not self._clients:
                    continue
                clients = list(self._clients)
                results = await asyncio.gather(*(c.send(msg) for c in clients),
                                               return_exceptions=True)
                for c, r in zip(clients, results):
                    if isinstance(r, Exception):
                        self._clients.discard(c)

    def _enqueue(self, msg: str) -> None:
        loop, queue = self._loop, self._queue
        if loop is None or queue is None or loop.is_closed():
            return

        def _put():
            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            queue.put_nowait(msg)

        try:
            loop.call_soon_threadsafe(_put)
        except RuntimeError:
            pass

    def pop(self, msg_type: str) -> dict | None:
        """Consume the latest client-injected message of this type (None if none)."""
        with self._inbox_lock:
            return self.inbox.pop(msg_type, None)

    # -- publishing ------------------------------------------------------
    def set_meta(self, n_drones: int, spacing: float | None = None,
                 extra: dict | None = None) -> None:
        """Describe the scene. Call again on changes; new clients get the latest meta
        on connect, connected clients get it re-broadcast in-stream."""
        meta = {"type": "meta", "source": self.source, "n_drones": int(n_drones)}
        if spacing is not None:
            meta["spacing"] = float(spacing)
        if extra:
            meta.update(extra)
        rebroadcast = self._meta is not None
        self._meta_dict = meta
        self._meta = json.dumps(meta, separators=(",", ":"))
        if self._rec is not None:
            self._rec.write(self._meta + "\n")
        if rebroadcast and self.enabled:
            self._enqueue(self._meta)

    def frame(self, t_sim: float, worlds, target_ned=None,
              phase: str | None = None) -> None:
        """Publish one frame if the sim-time rate gate opens. Cheap when gated."""
        if t_sim < self._next_t - 2 * self.period:   # sim-time regression = run reset
            self._next_t = -1e9
        if (not self.enabled and self._rec is None) or t_sim < self._next_t:
            return
        self._next_t = t_sim + self.period
        # real-time factor over a sliding anchor (re-anchored every 5 sim-seconds,
        # and on a sim-time regression = run reset)
        now = time.perf_counter()
        if t_sim < self._sim0:
            self._wall0, self._sim0 = now, t_sim
        if self._wall0 is None or t_sim - self._sim0 > 5.0:
            if self._wall0 is not None and now > self._wall0:
                self._rtf = (t_sim - self._sim0) / (now - self._wall0)
            self._wall0, self._sim0 = now, t_sim
        elif now > self._wall0 and t_sim > self._sim0:
            self._rtf = (t_sim - self._sim0) / (now - self._wall0)

        with self._inbox_lock:
            inj = self.inbox.get("target")
            ts = self._inbox_ts.get("target", 0.0)
        if inj is not None and time.monotonic() - ts > 2.0:
            inj = None   # annotator stopped: drop stale target/phase
        if target_ned is None and inj is not None:
            target_ned = inj.get("p")
        if phase is None and inj is not None:
            phase = inj.get("phase")

        drones = []
        for w in worlds:
            y = w.y
            drones.append({"p": [round(float(v), 3) for v in y[0:3]],
                           "q": [round(float(v), 4) for v in y[3:7]]})
        msg = {"type": "frame", "t": round(t_sim, 3), "rtf": round(self._rtf, 2),
               "drones": drones}
        if phase is not None:
            msg["phase"] = phase
        if target_ned is not None:
            msg["target"] = [round(float(v), 3) for v in target_ned]
        raw = json.dumps(msg, separators=(",", ":"))
        if self._rec is not None:
            self._rec.write(raw + "\n")
        if self.enabled:
            if self._meta is not None and now - self._meta_resend > 2.0:
                self._meta_resend = now
                self._enqueue(self._meta)   # network only; the recording keeps one per change
            self._enqueue(raw)
