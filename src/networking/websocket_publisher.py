import asyncio
import json
import logging
import threading
from typing import Any


try:
    import websockets
except ImportError:
    websockets = None

logger = logging.getLogger(__name__)

class GroundTruthWebSocketPublisher:
    def __init__(self, host: str, port: int, enabled: bool = True):
        self.host = host
        self.port = int(port)
        self.enabled = bool(enabled) and (websockets is not None)
        self._clients: set[Any] = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._queue: asyncio.Queue[str | None] | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if not self.enabled:
            if websockets is None:
                logger.warning("Ground-truth WS disabled: install 'websockets' package to enable it")
            return
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()

    def _thread_main(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._queue = asyncio.Queue(maxsize=1)
        try:
            self._loop.run_until_complete(self._run_server())
        except OSError as exc:
            logger.error(
                "Ground-truth WS failed on ws://%s:%s: %s",
                self.host,
                self.port,
                exc,
            )
            self.enabled = False
        except Exception:
            logger.exception("Ground-truth WS thread crashed")
            self.enabled = False
        finally:
            if self._loop is not None and not self._loop.is_closed():
                self._loop.close()
            self._loop = None
            self._queue = None
            self._clients.clear()

    async def _run_server(self) -> None:
        assert websockets is not None

        async def handler(websocket):
            self._clients.add(websocket)
            try:
                await websocket.wait_closed()
            finally:
                self._clients.discard(websocket)

        async with websockets.serve(handler, self.host, self.port):
            logger.info("Ground-truth WS listening on ws://%s:%s", self.host, self.port)
            while True:
                queue = self._queue
                if queue is None:
                    break
                msg = await queue.get()
                if msg is None:
                    break
                if not self._clients:
                    continue

                clients = list(self._clients)
                results = await asyncio.gather(*(c.send(msg) for c in clients), return_exceptions=True)
                for client, result in zip(clients, results):
                    if isinstance(result, Exception):
                        self._clients.discard(client)

    def publish(self, payload: dict[str, Any]) -> None:
        loop = self._loop
        queue = self._queue
        if (not self.enabled) or loop is None or queue is None or loop.is_closed():
            return

        msg = json.dumps(payload, separators=(",", ":"))

        def _enqueue() -> None:
            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            queue.put_nowait(msg)

        try:
            loop.call_soon_threadsafe(_enqueue)
        except RuntimeError:
            return

    def stop(self) -> None:
        loop = self._loop
        queue = self._queue
        if (not self.enabled) or loop is None or queue is None or loop.is_closed():
            return

        def _stop_enqueue() -> None:
            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            queue.put_nowait(None)

        try:
            loop.call_soon_threadsafe(_stop_enqueue)
        except RuntimeError:
            return
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
