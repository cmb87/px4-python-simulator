#!/usr/bin/env python3

import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster


def _add_sim_import_paths() -> None:
    file_path = Path(__file__).resolve()
    repo_root = file_path.parents[5]
    src_dir = repo_root / "src"
    vehicle_dir = src_dir / "vehicle"
    for p in (str(src_dir), str(vehicle_dir)):
        if p not in sys.path:
            sys.path.insert(0, p)


_add_sim_import_paths()
from world import World  # noqa: E402


MAVLINK_MSG_ID_HIL_STATE_QUATERNION = 115
RATE_HZ_DEFAULT = 250
CHECK_FACTOR = 2
DT_US_DEFAULT = int(1e6 / RATE_HZ_DEFAULT)
HEARTBEAT_INTERVAL_US = 1_000_000
SYSTEM_TIME_INTERVAL_US = 1_000_000
GPS_START_DELAY_US = 1_000_000


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_motor_map(raw: str) -> tuple[int, int, int, int]:
    tokens = [tok.strip() for tok in raw.split(",") if tok.strip()]
    if len(tokens) != 4:
        raise ValueError(f"Expected 4 entries in ts04_motor_map, got '{raw}'")
    values = tuple(int(tok) for tok in tokens)
    if sorted(values) != [0, 1, 2, 3]:
        raise ValueError(f"ts04_motor_map must be a permutation of 0,1,2,3, got '{raw}'")
    return (values[0], values[1], values[2], values[3])


def controls_to_u(
    latest_controls: tuple[float, ...] | None,
    armed: bool,
    ts04_motor_map: tuple[int, int, int, int],
    vehicle_model: str,
) -> np.ndarray:
    u = np.zeros(4)
    if (not armed) or latest_controls is None:
        return u
    if len(latest_controls) > 0:
        u[0] = float(latest_controls[0])
    if len(latest_controls) > 1:
        u[1] = float(latest_controls[1])
    if len(latest_controls) > 2:
        u[2] = float(latest_controls[2])
    if len(latest_controls) > 3:
        u[3] = clamp(float(latest_controls[3]), 0.0, 1.0)
    if vehicle_model == "ts04":
        u = u[list(ts04_motor_map)]
    return u


def quat_wxyz_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(v) for v in q]
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n <= 0.0:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rot_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    qn = np.linalg.norm(q)
    return q / qn if qn > 0.0 else np.array([1.0, 0.0, 0.0, 0.0], dtype=float)


def ned_to_enu_position(p_ned: np.ndarray) -> np.ndarray:
    return np.array([p_ned[1], p_ned[0], -p_ned[2]], dtype=float)


def frd_to_flu_vector(v_frd: np.ndarray) -> np.ndarray:
    return np.array([v_frd[0], -v_frd[1], -v_frd[2]], dtype=float)


def ned_frd_quat_to_enu_flu_quat(q_wxyz: np.ndarray) -> np.ndarray:
    R_ned_frd = quat_wxyz_to_rot(q_wxyz)
    T_enu2ned = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]], dtype=float)
    T_frd2flu = np.diag([1.0, -1.0, -1.0])
    R_enu_flu = T_frd2flu @ R_ned_frd @ T_enu2ned
    return rot_to_quat_wxyz(R_enu_flu)


class Px4LockstepRos2Node(Node):
    def __init__(self) -> None:
        super().__init__("px4_lockstep_ros2_node")

        self.declare_parameter("mavlink_bind_host", "0.0.0.0")
        self.declare_parameter("mavlink_bind_port", 4560)
        self.declare_parameter("vehicle_model", "ts04")
        self.declare_parameter("rate_hz", RATE_HZ_DEFAULT)
        self.declare_parameter("ts04_pitch90_start", True)
        self.declare_parameter("ts04_motor_map", "0,1,2,3")
        self.declare_parameter("gps_origin_lat", 47.397742)
        self.declare_parameter("gps_origin_lon", 8.545594)
        self.declare_parameter("gps_origin_alt", 470.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")

        self.mavlink_bind_host = str(self.get_parameter("mavlink_bind_host").value)
        self.mavlink_bind_port = int(self.get_parameter("mavlink_bind_port").value)
        self.vehicle_model = str(self.get_parameter("vehicle_model").value).strip().lower()
        self.rate_hz = int(self.get_parameter("rate_hz").value)
        self.ts04_pitch90_start = bool(self.get_parameter("ts04_pitch90_start").value)
        self.ts04_motor_map = parse_motor_map(str(self.get_parameter("ts04_motor_map").value))
        self.gps_origin = {
            "lat": float(self.get_parameter("gps_origin_lat").value),
            "lon": float(self.get_parameter("gps_origin_lon").value),
            "alt": float(self.get_parameter("gps_origin_alt").value),
        }
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)

        self.dt_us = int(1e6 / max(1, self.rate_hz))
        self.sim_time_us = 0
        self.next_heartbeat_time_us = 0
        self.next_system_time_us = 0
        self.gps_start_time_us = GPS_START_DELAY_US
        self.hil_state_interval_us = -1
        self.next_hil_state_time_us = 0
        self.last_time_ran_ms = 0
        self.slow_down_counter = 0

        self.latest_controls: tuple[float, ...] | None = None
        self.armed = False
        self.was_armed = False
        self.ever_armed = False

        endpoint = f"tcpin:{self.mavlink_bind_host}:{self.mavlink_bind_port}"
        self.conn: Any = mavutil.mavlink_connection(endpoint, source_component=51)

        self.get_logger().info(f"Waiting for PX4 heartbeat on {endpoint} ...")
        hb = self.conn.wait_heartbeat()
        px4_sysid = hb.get_srcSystem()
        if px4_sysid > 0:
            self.conn.source_system = px4_sysid
            self.conn.mav.srcSystem = px4_sysid

        y0 = np.zeros(13)
        y0[0:3] = np.array([0.0, 0.0, -3.0])
        if self.vehicle_model == "ts04" and self.ts04_pitch90_start:
            y0[3:7] = np.array([math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0])
        else:
            y0[3] = 1.0

        self.world = World(
            vehicle_model=self.vehicle_model,
            y0=y0,
            u0=np.zeros(4),
            wind0=np.zeros(6),
            ts04_pitch90_start=self.ts04_pitch90_start,
        )
        self.world.P.gps_origin = dict(self.gps_origin)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/sim/odom", 10)
        self.gps_fix_pub = self.create_publisher(NavSatFix, "/sim/gps/fix", 10)
        self.gps_vel_pub = self.create_publisher(TwistStamped, "/sim/gps/vel", 10)

        self.timer = self.create_timer(1.0 / max(1, self.rate_hz), self._step)
        self.get_logger().info("ROS2 lockstep node running")

    def _step(self) -> None:
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == "HIL_ACTUATOR_CONTROLS":
                controls = getattr(msg, "controls", None)
                self.latest_controls = tuple(float(v) for v in controls) if isinstance(controls, (list, tuple)) else None
                mode = int(getattr(msg, "mode", 0))
                if mode != 0:
                    self.armed = (mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    if self.armed:
                        self.ever_armed = True
            elif msg_type == "COMMAND_LONG":
                command = int(getattr(msg, "command", 0))
                if command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    msg_id = int(float(getattr(msg, "param1", 0.0)) + 0.5)
                    interval_us = int(float(getattr(msg, "param2", -1.0)) + 0.5)
                    if msg_id == MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
                        self.hil_state_interval_us = interval_us if interval_us > 0 else -1
                        self.next_hil_state_time_us = self.sim_time_us

        if self.armed and (not self.was_armed):
            self.get_logger().info("ARM transition: dynamics enabled")
        self.was_armed = self.armed

        self.world.set_controls(
            controls_to_u(
                latest_controls=self.latest_controls,
                armed=self.armed,
                ts04_motor_map=self.ts04_motor_map,
                vehicle_model=self.vehicle_model,
            )
        )

        self.sim_time_us += self.dt_us
        io_run_only = (self.slow_down_counter % CHECK_FACTOR) != 0
        now_ms = self.sim_time_us // 1000
        needs_to_pause = (self.last_time_ran_ms == now_ms) or io_run_only
        world_out = self.world.update(self.sim_time_us, needs_to_pause, freeze_dynamics=(not self.ever_armed))

        should_publish = world_out is not None and (not needs_to_pause)
        if should_publish:
            self._publish_and_send(world_out)
            self.last_time_ran_ms = now_ms

        if self.sim_time_us >= self.next_heartbeat_time_us:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self.next_heartbeat_time_us = self.sim_time_us + HEARTBEAT_INTERVAL_US

        self.slow_down_counter += 1

    def _publish_and_send(self, world_out: dict[str, Any]) -> None:
        z = world_out["sensors"]
        y = np.asarray(world_out["y"], dtype=float)
        gps = np.asarray(z["gps"], dtype=float)
        acc = np.asarray(z["accelerometer"], dtype=float)
        gyro = np.asarray(z["gyroscope"], dtype=float)
        mag = np.asarray(z["magnetometer"], dtype=float)
        baro = z["barometer"]

        self.conn.mav.hil_sensor_send(
            int(self.sim_time_us),
            float(acc[0]),
            float(acc[1]),
            float(acc[2]),
            float(gyro[0]),
            float(gyro[1]),
            float(gyro[2]),
            float(mag[0]),
            float(mag[1]),
            float(mag[2]),
            float(baro["staticAbsolute"]) * 0.01,
            float(baro["dynamic"]) * 0.01,
            float(baro.get("pressure_altitude_m", gps[2])),
            15.0,
            8191,
        )

        if self.hil_state_interval_us > 0 and self.sim_time_us >= self.next_hil_state_time_us:
            vel_north = float(gps[3])
            vel_east = float(gps[4])
            vel_down = float(-gps[5])
            horiz_speed_m_s = float(np.hypot(vel_north, vel_east))
            m_s2_to_mg = 1000.0 / 9.80665
            self.conn.mav.hil_state_quaternion_send(
                int(self.sim_time_us),
                [float(v) for v in y[3:7]],
                float(y[10]),
                float(y[11]),
                float(y[12]),
                int(round(float(gps[0]) * 1e7)),
                int(round(float(gps[1]) * 1e7)),
                int(round(float(gps[2]) * 1000.0)),
                int(round(vel_north * 100.0)),
                int(round(vel_east * 100.0)),
                int(round(vel_down * 100.0)),
                int(round(horiz_speed_m_s * 100.0)),
                int(round(horiz_speed_m_s * 100.0)),
                int(round(float(acc[0]) * m_s2_to_mg)),
                int(round(float(acc[1]) * m_s2_to_mg)),
                int(round(float(acc[2]) * m_s2_to_mg)),
            )
            self.next_hil_state_time_us = self.sim_time_us + self.hil_state_interval_us

        if self.sim_time_us >= self.next_system_time_us:
            self.conn.mav.system_time_send(int(time.time() * 1_000_000), int(self.sim_time_us / 1000))
            self.next_system_time_us = self.sim_time_us + SYSTEM_TIME_INTERVAL_US

        if self.sim_time_us >= self.gps_start_time_us and bool(z.get("gps_updated", False)):
            vel_north = float(gps[3])
            vel_east = float(gps[4])
            vel_down = float(-gps[5])
            vel_3d = float(np.linalg.norm(np.array([vel_north, vel_east, vel_down])))
            cog_rad = float(np.arctan2(vel_east, vel_north))
            if cog_rad < 0.0:
                cog_rad += 2.0 * np.pi
            self.conn.mav.hil_gps_send(
                int(self.sim_time_us),
                3,
                int(round(float(gps[0]) * 1e7)),
                int(round(float(gps[1]) * 1e7)),
                int(round(float(gps[2]) * 1000.0)),
                100,
                100,
                int(round(vel_3d * 100.0)),
                int(round(vel_north * 100.0)),
                int(round(vel_east * 100.0)),
                int(round(vel_down * 100.0)),
                int(round(np.degrees(cog_rad) * 100.0)),
                10,
                0,
                0,
            )

        self._publish_ros2_ground_truth(y, world_out["ydot"], gps)

    def _publish_ros2_ground_truth(self, y: np.ndarray, ydot: np.ndarray, gps: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()

        p_ned = y[0:3]
        q_ned_frd = y[3:7]
        v_body_frd = y[7:10]
        w_body_frd = y[10:13]

        p_enu = ned_to_enu_position(p_ned)
        q_enu_flu = ned_frd_quat_to_enu_flu_quat(q_ned_frd)
        v_body_flu = frd_to_flu_vector(v_body_frd)
        w_body_flu = frd_to_flu_vector(w_body_frd)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.frame_id
        tf_msg.child_frame_id = self.child_frame_id
        tf_msg.transform.translation.x = float(p_enu[0])
        tf_msg.transform.translation.y = float(p_enu[1])
        tf_msg.transform.translation.z = float(p_enu[2])
        tf_msg.transform.rotation.w = float(q_enu_flu[0])
        tf_msg.transform.rotation.x = float(q_enu_flu[1])
        tf_msg.transform.rotation.y = float(q_enu_flu[2])
        tf_msg.transform.rotation.z = float(q_enu_flu[3])
        self.tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = float(p_enu[0])
        odom.pose.pose.position.y = float(p_enu[1])
        odom.pose.pose.position.z = float(p_enu[2])
        odom.pose.pose.orientation.w = float(q_enu_flu[0])
        odom.pose.pose.orientation.x = float(q_enu_flu[1])
        odom.pose.pose.orientation.y = float(q_enu_flu[2])
        odom.pose.pose.orientation.z = float(q_enu_flu[3])
        odom.twist.twist.linear.x = float(v_body_flu[0])
        odom.twist.twist.linear.y = float(v_body_flu[1])
        odom.twist.twist.linear.z = float(v_body_flu[2])
        odom.twist.twist.angular.x = float(w_body_flu[0])
        odom.twist.twist.angular.y = float(w_body_flu[1])
        odom.twist.twist.angular.z = float(w_body_flu[2])
        self.odom_pub.publish(odom)

        gps_fix = NavSatFix()
        gps_fix.header.stamp = stamp
        gps_fix.header.frame_id = self.frame_id
        gps_fix.status.status = NavSatStatus.STATUS_FIX
        gps_fix.status.service = NavSatStatus.SERVICE_GPS
        gps_fix.latitude = float(gps[0])
        gps_fix.longitude = float(gps[1])
        gps_fix.altitude = float(gps[2])
        self.gps_fix_pub.publish(gps_fix)

        gps_vel = TwistStamped()
        gps_vel.header.stamp = stamp
        gps_vel.header.frame_id = self.frame_id
        gps_vel.twist.linear.x = float(gps[4])
        gps_vel.twist.linear.y = float(gps[3])
        gps_vel.twist.linear.z = float(gps[5])
        self.gps_vel_pub.publish(gps_vel)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Px4LockstepRos2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
