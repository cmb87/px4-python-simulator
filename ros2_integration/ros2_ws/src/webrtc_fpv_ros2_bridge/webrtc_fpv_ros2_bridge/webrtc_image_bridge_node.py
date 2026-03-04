#!/usr/bin/env python3
import asyncio
import contextlib
import json
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from aiohttp import WSMsgType, web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.mediastreams import MediaStreamError
from aiortc.sdp import candidate_from_sdp
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


@dataclass
class FrameStats:
    count: int = 0
    last_report_ts: float = 0.0


class WebRtcImageBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("webrtc_fpv_bridge_node")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 9001)
        self.declare_parameter("image_topic", "/sim/image")
        self.declare_parameter("frame_id", "camera")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(Image, self.image_topic, qos)

    def publish_image(self, frame_array: np.ndarray) -> None:
        if frame_array.ndim != 3 or frame_array.shape[2] != 3:
            return

        height, width = frame_array.shape[:2]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = int(height)
        msg.width = int(width)
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(width) * 3
        msg.data = frame_array.tobytes()
        self.image_pub.publish(msg)


def parse_candidate(candidate_payload):
    if isinstance(candidate_payload, str):
        candidate_sdp = candidate_payload
        sdp_mid = None
        sdp_mline_index = None
        username_fragment = None
    else:
        candidate_sdp = candidate_payload.get("candidate", "")
        sdp_mid = candidate_payload.get("sdpMid")
        sdp_mline_index = candidate_payload.get("sdpMLineIndex")
        username_fragment = candidate_payload.get("usernameFragment")

    if not candidate_sdp:
        return None

    if candidate_sdp.startswith("candidate:"):
        candidate_sdp = candidate_sdp.split(":", 1)[1]

    candidate = candidate_from_sdp(candidate_sdp)
    candidate.sdpMid = sdp_mid
    candidate.sdpMLineIndex = sdp_mline_index
    candidate.usernameFragment = username_fragment
    return candidate


async def consume_video_track(track, peer_id: str, bridge: WebRtcImageBridgeNode) -> None:
    stats = FrameStats(last_report_ts=time.time())
    try:
        while True:
            frame = await track.recv()
            stats.count += 1

            frame_array = frame.to_ndarray(format="bgr24")
            bridge.publish_image(frame_array)

            now = time.time()
            elapsed = now - stats.last_report_ts
            if elapsed >= 1.0:
                height, width = frame_array.shape[:2]
                fps = stats.count / elapsed
                bridge.get_logger().info(f"[{peer_id}] video {width}x{height} @ {fps:.1f} fps")
                stats.count = 0
                stats.last_report_ts = now
    except (MediaStreamError, asyncio.CancelledError):
        return


async def websocket_handler(request):
    bridge: WebRtcImageBridgeNode = request.app["bridge"]
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)

    peer = None
    track_task = None
    peer_id = f"peer-{id(ws)}"
    bridge.get_logger().info(f"[{peer_id}] signaling connected")

    async def close_peer():
        nonlocal peer, track_task
        if track_task is not None:
            track_task.cancel()
            with contextlib.suppress(asyncio.CancelledError, MediaStreamError):
                await track_task
            track_task = None

        if peer is not None:
            await peer.close()
            peer = None

    async def ensure_peer():
        nonlocal peer, track_task
        if peer is not None:
            return peer

        peer = RTCPeerConnection()

        @peer.on("track")
        def on_track(track):
            nonlocal track_task
            bridge.get_logger().info(f"[{peer_id}] track received: {track.kind}")
            if track.kind == "video":
                if track_task is not None:
                    track_task.cancel()
                track_task = asyncio.create_task(consume_video_track(track, peer_id, bridge))

        @peer.on("icecandidate")
        async def on_icecandidate(candidate):
            if candidate is None or ws.closed:
                return
            await ws.send_json(
                {
                    "type": "candidate",
                    "candidate": {
                        "candidate": f"candidate:{candidate.to_sdp()}",
                        "sdpMid": candidate.sdpMid,
                        "sdpMLineIndex": candidate.sdpMLineIndex,
                        "usernameFragment": candidate.usernameFragment,
                    },
                }
            )

        @peer.on("connectionstatechange")
        async def on_connectionstatechange():
            current_peer = peer
            if current_peer is None:
                return
            state = current_peer.connectionState
            bridge.get_logger().info(f"[{peer_id}] connection state: {state}")
            if state in {"failed", "closed", "disconnected"}:
                await close_peer()

        return peer

    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue

            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                await ws.send_json({"type": "error", "message": "Invalid JSON"})
                continue

            msg_type = payload.get("type")
            if msg_type == "offer":
                sdp = payload.get("sdp")
                if not sdp:
                    await ws.send_json({"type": "error", "message": "Missing offer sdp"})
                    continue

                meta = payload.get("meta") or {}
                bridge.get_logger().info(
                    f"[{peer_id}] offer meta source={meta.get('source', 'unknown')} "
                    f"camera={meta.get('cameraMode', 'unknown')} "
                    f"requested={meta.get('width')}x{meta.get('height')}@{meta.get('fps')}"
                )

                pc = await ensure_peer()
                await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                await ws.send_json({"type": "answer", "sdp": pc.localDescription.sdp})
                continue

            if msg_type == "candidate":
                current_peer = peer
                if current_peer is None:
                    continue
                candidate = parse_candidate(payload.get("candidate"))
                if candidate is None:
                    continue
                current_peer.addIceCandidate(candidate)
                continue

            if msg_type == "stop":
                await close_peer()
                continue

            await ws.send_json({"type": "error", "message": f"Unknown message type: {msg_type}"})
    finally:
        await close_peer()
        bridge.get_logger().info(f"[{peer_id}] signaling disconnected")

    return ws


async def healthcheck_handler(_request):
    return web.json_response({"status": "ok"})


def create_app(bridge: WebRtcImageBridgeNode):
    app = web.Application()
    app["bridge"] = bridge
    app.router.add_get("/healthz", healthcheck_handler)
    app.router.add_get("/webrtc", websocket_handler)
    return app


def main(args=None) -> None:
    rclpy.init(args=args)
    bridge = WebRtcImageBridgeNode()
    app = create_app(bridge)

    bridge.get_logger().info(
        f"Starting WebRTC FPV bridge on ws://{bridge.host}:{bridge.port}/webrtc -> {bridge.image_topic}"
    )

    try:
        web.run_app(app, host=bridge.host, port=bridge.port)
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
