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
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = int(port)
        self.enabled = websockets is not None
        self._clients: set[Any] = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._queue: asyncio.Queue[str | None] | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if not self.enabled:
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
        finally:
            self._loop.close()

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
        if not self.enabled or self._loop is None or self._queue is None:
            return

        msg = json.dumps(payload, separators=(",", ":"))

        def _enqueue() -> None:
            assert self._queue is not None
            if self._queue.full():
                try:
                    self._queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            self._queue.put_nowait(msg)

        self._loop.call_soon_threadsafe(_enqueue)

    def stop(self) -> None:
        if not self.enabled or self._loop is None or self._queue is None:
            return

        def _stop_enqueue() -> None:
            assert self._queue is not None
            if self._queue.full():
                try:
                    self._queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            self._queue.put_nowait(None)

        self._loop.call_soon_threadsafe(_stop_enqueue)
        if self._thread is not None:
            self._thread.join(timeout=1.0)
