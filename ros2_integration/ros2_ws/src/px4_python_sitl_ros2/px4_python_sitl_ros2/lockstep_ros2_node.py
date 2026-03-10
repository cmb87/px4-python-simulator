#!/usr/bin/env python3

import os

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32MultiArray, UInt8
from tf2_msgs.msg import TFMessage

import time
from typing import Any

import numpy as np
from pymavlink import mavutil

from vehicle.sim_utils import (
    controls_to_u,
    ned_frd_quat_to_enu_flu_quat,
    ned_to_enu_position,
    parse_vehicle_model,
)
from vehicle.vehicle_catalog import list_vehicle_models
from vehicle.world import World


MAVLINK_MSG_ID_HIL_STATE_QUATERNION = 115
RATE_HZ_DEFAULT = 250
CHECK_FACTOR = 2
HEARTBEAT_INTERVAL_US = 1_000_000
SYSTEM_TIME_INTERVAL_US = 1_000_000
GPS_START_DELAY_US = 1_000_000


class Px4LockstepRos2Node(Node):
    def __init__(self) -> None:
        super().__init__("px4_lockstep_ros2_node")

        self.declare_parameter("mavlink_bind_host", "0.0.0.0")
        self.declare_parameter("mavlink_bind_port", 4560)
        self.declare_parameter("vehicle_model", "ts04")
        self.declare_parameter("rate_hz", RATE_HZ_DEFAULT)
        self.declare_parameter("ts04_pitch90_start", True)
        self.declare_parameter("gps_origin_lat", 47.397742)
        self.declare_parameter("gps_origin_lon", 8.545594)
        self.declare_parameter("gps_origin_alt", 470.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("tf_topic", "/sim/tf")
        self.declare_parameter("gps_fix_topic", "/sim/gps/fix")
        self.declare_parameter("gps_vel_topic", "/sim/gps/vel")
        self.declare_parameter("actuators_topic", "/sim/actuators")
        self.declare_parameter("aero_topic", "/sim/aero")
        self.declare_parameter("sysid_topic", "/sim/px4_sysid")

        self.mavlink_bind_host = str(self.get_parameter("mavlink_bind_host").value)
        self.mavlink_bind_port = int(self.get_parameter("mavlink_bind_port").value)
        self.vehicle_model = str(self.get_parameter("vehicle_model").value).strip().lower()
        self.vehicle_model = parse_vehicle_model(self.vehicle_model, list_vehicle_models())
        self.rate_hz = int(self.get_parameter("rate_hz").value)
        self.ts04_pitch90_start = bool(self.get_parameter("ts04_pitch90_start").value)
        self.gps_origin = {
            "lat": float(self.get_parameter("gps_origin_lat").value),
            "lon": float(self.get_parameter("gps_origin_lon").value),
            "alt": float(self.get_parameter("gps_origin_alt").value),
        }
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.tf_topic = str(self.get_parameter("tf_topic").value)
        self.gps_fix_topic = str(self.get_parameter("gps_fix_topic").value)
        self.gps_vel_topic = str(self.get_parameter("gps_vel_topic").value)
        self.actuators_topic = str(self.get_parameter("actuators_topic").value)
        self.aero_topic = str(self.get_parameter("aero_topic").value)
        self.sysid_topic = str(self.get_parameter("sysid_topic").value)

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
        self.px4_sysid = int(px4_sysid if px4_sysid > 0 else 1)
        if px4_sysid > 0:
            self.conn.source_system = px4_sysid
            self.conn.mav.srcSystem = px4_sysid

        self.world = World(
            vehicle_model=self.vehicle_model,
            u0=np.zeros(4),
            wind0=np.zeros(6),
            ts04_pitch90_start=self.ts04_pitch90_start,
        )
        self.world.P.gps_origin = dict(self.gps_origin)
        self.catapult_enabled = bool(getattr(self.world, "rail_launch_enabled", False))
        try:
            self.catapult_launch_countdown_s = max(0.0, float(os.getenv("SIM_CATAPULT_LAUNCH_COUNTDOWN_S", "3.0")))
        except ValueError as exc:
            raise ValueError("SIM_CATAPULT_LAUNCH_COUNTDOWN_S must be a float") from exc
        self.catapult_launch_countdown_us = int(self.catapult_launch_countdown_s * 1e6)
        self.catapult_countdown_active = False
        self.catapult_release_time_us: int | None = None
        self.next_countdown_announce_us = 0
        if self.catapult_enabled:
            self.get_logger().info(f"Catapult launch enabled with countdown {self.catapult_launch_countdown_s:.1f} s")

        self.tf_pub = self.create_publisher(TFMessage, self.tf_topic, 10)
        self.gps_fix_pub = self.create_publisher(NavSatFix, self.gps_fix_topic, 10)
        self.gps_vel_pub = self.create_publisher(TwistStamped, self.gps_vel_topic, 10)
        self.actuators_pub = self.create_publisher(Float32MultiArray, self.actuators_topic, 10)
        self.aero_pub = self.create_publisher(Vector3Stamped, self.aero_topic, 10)
        sysid_qos = QoSProfile(depth=1)
        sysid_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        sysid_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.sysid_pub = self.create_publisher(UInt8, self.sysid_topic, sysid_qos)

        self.timer = self.create_timer(1.0 / max(1, self.rate_hz), self._step)
        self.sysid_timer = self.create_timer(1.0, self._publish_sysid)
        self._publish_sysid()
        self.get_logger().info("ROS2 lockstep node running")

    def _publish_sysid(self) -> None:
        msg = UInt8()
        msg.data = int(self.px4_sysid) & 0xFF
        self.sysid_pub.publish(msg)

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
            if self.catapult_enabled and self.catapult_launch_countdown_us > 0:
                self.catapult_countdown_active = True
                self.catapult_release_time_us = self.sim_time_us + self.catapult_launch_countdown_us
                self.next_countdown_announce_us = self.sim_time_us
                self.get_logger().info(f"ARM transition: catapult launch in {self.catapult_launch_countdown_s:.1f} s")
            else:
                self.get_logger().info("ARM transition: dynamics enabled")
        if (not self.armed) and self.was_armed and self.catapult_countdown_active:
            self.catapult_countdown_active = False
            self.catapult_release_time_us = None
            self.get_logger().info("Catapult countdown cancelled (disarmed)")
        self.was_armed = self.armed

        self.world.set_controls(
            controls_to_u(
                latest_controls=self.latest_controls,
                armed=self.armed,
            )
        )

        self.sim_time_us += self.dt_us
        io_run_only = (self.slow_down_counter % CHECK_FACTOR) != 0
        now_ms = self.sim_time_us // 1000
        needs_to_pause = (self.last_time_ran_ms == now_ms) or io_run_only
        if self.catapult_countdown_active and self.catapult_release_time_us is not None:
            if self.sim_time_us >= self.catapult_release_time_us:
                self.catapult_countdown_active = False
                self.catapult_release_time_us = None
                self.get_logger().info("Catapult launch release")
            elif self.sim_time_us >= self.next_countdown_announce_us:
                remaining_s = max(0.0, (self.catapult_release_time_us - self.sim_time_us) / 1e6)
                self.get_logger().info(f"Catapult countdown: {remaining_s:.1f} s")
                self.next_countdown_announce_us = self.sim_time_us + 1_000_000

        freeze_for_countdown = self.catapult_enabled and self.catapult_countdown_active
        world_out = self.world.update(
            self.sim_time_us,
            needs_to_pause,
            freeze_dynamics=((not self.ever_armed) or freeze_for_countdown),
        )

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
        fields_updated = 8191
        has_airspeed_sensor = bool(getattr(self.world.P, "has_airspeed_sensor", False))
        if not has_airspeed_sensor:
            fields_updated &= ~int(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)

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
            int(fields_updated),
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

        self._publish_ros2_ground_truth(y, gps)

    @staticmethod
    def _controls_to_u8(latest_controls: tuple[float, ...] | None) -> list[float]:
        if latest_controls is None:
            return [0.0] * 8
        out = [0.0] * 8
        n = min(len(latest_controls), 8)
        for i in range(n):
            out[i] = float(latest_controls[i])
        return out

    def _compute_aero(self, y: np.ndarray) -> tuple[float | None, float | None, float]:
        wind = np.asarray(self.world.wind[:3], dtype=float)
        vel_rel = np.asarray(y[7:10], dtype=float) - wind
        u_r, v_r, w_r = vel_rel
        va_mps = float(np.linalg.norm(vel_rel))
        if va_mps <= 1e-5:
            return None, None, va_mps
        alpha_deg = float(np.rad2deg(np.arctan2(w_r, u_r)))
        beta_deg = float(np.rad2deg(np.arcsin(np.clip(v_r / va_mps, -1.0, 1.0))))
        return alpha_deg, beta_deg, va_mps

    def _publish_ros2_ground_truth(self, y: np.ndarray, gps: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()
        u = self._controls_to_u8(self.latest_controls)
        alpha_deg, beta_deg, va_mps = self._compute_aero(y)

        p_ned = y[0:3]
        q_ned_frd = y[3:7]

        p_enu = ned_to_enu_position(p_ned)
        q_enu_flu = ned_frd_quat_to_enu_flu_quat(q_ned_frd)

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
        self.tf_pub.publish(TFMessage(transforms=[tf_msg]))

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

        actuators = Float32MultiArray()
        actuators.data = [float(v) for v in u]
        self.actuators_pub.publish(actuators)

        aero = Vector3Stamped()
        aero.header.stamp = stamp
        aero.header.frame_id = self.frame_id
        aero.vector.x = float(alpha_deg) if alpha_deg is not None else float("nan")
        aero.vector.y = float(beta_deg) if beta_deg is not None else float("nan")
        aero.vector.z = va_mps
        self.aero_pub.publish(aero)


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
