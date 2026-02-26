import socket
import struct
import time
from dataclasses import dataclass

import numpy as np

from quaternion import Quaternion


_PACKET_HEADER = struct.Struct("!IQ")
_PACKET_FLOAT_COUNT = 26
_PACKET_FLOATS = struct.Struct("!" + ("d" * _PACKET_FLOAT_COUNT))
_PACKET_SIZE = _PACKET_HEADER.size + _PACKET_FLOATS.size


def _normalize_quat_wxyz(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    n = float(np.linalg.norm(q))
    if n <= 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.asarray(a, dtype=float).reshape(4)
    bw, bx, by, bz = np.asarray(b, dtype=float).reshape(4)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def omega_matrix(omega_b: np.ndarray) -> np.ndarray:
    p, q, r = np.asarray(omega_b, dtype=float).reshape(3)
    return np.array(
        [
            [0.0, -p, -q, -r],
            [p, 0.0, r, -q],
            [q, -r, 0.0, p],
            [r, q, -p, 0.0],
        ],
        dtype=float,
    )


def quat_from_euler_deg_wxyz(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    euler_rad = np.deg2rad(np.array([roll_deg, pitch_deg, yaw_deg], dtype=float))
    return _normalize_quat_wxyz(Quaternion.euler2quat(euler_rad))


@dataclass(frozen=True)
class TransferPacket:
    seq: int
    time_us: int
    y: np.ndarray
    ydot: np.ndarray


class TransferAlignmentMasterLink:
    def __init__(self, target_host: str, target_port: int):
        self._target = (str(target_host), int(target_port))
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq = 0

    def close(self) -> None:
        self._sock.close()

    def send(self, time_us: int, y: np.ndarray, ydot: np.ndarray) -> None:
        y = np.asarray(y, dtype=float).reshape(13)
        ydot = np.asarray(ydot, dtype=float).reshape(13)
        payload = _PACKET_HEADER.pack(int(self._seq), int(time_us))
        payload += _PACKET_FLOATS.pack(*np.concatenate((y, ydot)).tolist())
        self._sock.sendto(payload, self._target)
        self._seq = (self._seq + 1) & 0xFFFFFFFF


class TransferAlignmentSlaveLink:
    def __init__(self, bind_host: str, bind_port: int, timeout_s: float = 1.0):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((str(bind_host), int(bind_port)))
        self._sock.setblocking(False)
        self._timeout_s = float(timeout_s)
        self._last_rx_wall_s = time.time()
        self._last_seq = -1
        self._latest_packet: TransferPacket | None = None

    def close(self) -> None:
        self._sock.close()

    def timed_out(self) -> bool:
        return (time.time() - self._last_rx_wall_s) > self._timeout_s

    def poll_latest(self) -> TransferPacket | None:
        latest = self._latest_packet
        while True:
            try:
                data, _addr = self._sock.recvfrom(4096)
            except BlockingIOError:
                break

            if len(data) != _PACKET_SIZE:
                continue

            seq, time_us = _PACKET_HEADER.unpack_from(data, 0)
            if self._last_seq >= 0 and int(seq) <= self._last_seq:
                continue

            vals = np.array(_PACKET_FLOATS.unpack_from(data, _PACKET_HEADER.size), dtype=float)
            y = vals[:13].copy()
            ydot = vals[13:].copy()

            pkt = TransferPacket(seq=int(seq), time_us=int(time_us), y=y, ydot=ydot)
            latest = pkt
            self._latest_packet = pkt
            self._last_seq = int(seq)
            self._last_rx_wall_s = time.time()

        return latest


def transform_master_to_slave_state(
    y_master: np.ndarray,
    ydot_master: np.ndarray,
    arm_m: np.ndarray,
    q_slave_from_master_wxyz: np.ndarray,
    arm_frame: str = "world_ned",
) -> tuple[np.ndarray, np.ndarray]:
    y_master = np.asarray(y_master, dtype=float).reshape(13)
    ydot_master = np.asarray(ydot_master, dtype=float).reshape(13)
    arm_m = np.asarray(arm_m, dtype=float).reshape(3)
    q_sm = _normalize_quat_wxyz(q_slave_from_master_wxyz)
    arm_frame_value = str(arm_frame).strip().lower()
    if arm_frame_value not in {"world_ned", "master_body"}:
        raise ValueError(f"Unsupported arm frame '{arm_frame}'")

    y_slave = np.zeros(13, dtype=float)
    ydot_slave = np.zeros(13, dtype=float)

    q_m = _normalize_quat_wxyz(y_master[3:7])
    v_m_b = y_master[7:10]
    omega_m_b = y_master[10:13]
    dv_m_b = ydot_master[7:10]
    domega_m_b = ydot_master[10:13]

    r_wm = Quaternion.Mfg(q_m)
    r_mw = r_wm.T

    r_sm = Quaternion.Mfg(q_sm)

    if arm_frame_value == "master_body":
        r_world = r_mw @ arm_m
    else:
        r_world = arm_m

    p_m_ned = y_master[0:3]
    p_s_ned = p_m_ned + r_world

    omega_w = r_mw @ omega_m_b
    v_m_world = r_mw @ v_m_b
    if arm_frame_value == "master_body":
        v_s_world = v_m_world + np.cross(omega_w, r_world)
    else:
        v_s_world = v_m_world

    q_s = _normalize_quat_wxyz(quat_mul_wxyz(q_sm, q_m))
    r_sw = Quaternion.Mfg(q_s)
    v_s_b = r_sw @ v_s_world

    omega_s_b = r_sm @ omega_m_b

    a_m_world = r_mw @ (dv_m_b + np.cross(omega_m_b, v_m_b))
    alpha_world = r_mw @ domega_m_b
    if arm_frame_value == "master_body":
        a_s_world = a_m_world + np.cross(alpha_world, r_world) + np.cross(omega_w, np.cross(omega_w, r_world))
    else:
        a_s_world = a_m_world
    dv_s_b = r_sw @ a_s_world - np.cross(omega_s_b, v_s_b)

    alpha_s_b = r_sm @ domega_m_b
    qdot_s = 0.5 * (omega_matrix(omega_s_b) @ q_s)

    y_slave[0:3] = p_s_ned
    y_slave[3:7] = q_s
    y_slave[7:10] = v_s_b
    y_slave[10:13] = omega_s_b

    ydot_slave[0:3] = v_s_world
    ydot_slave[3:7] = qdot_s
    ydot_slave[7:10] = dv_s_b
    ydot_slave[10:13] = alpha_s_b

    return y_slave, ydot_slave
