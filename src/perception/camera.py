from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dynamics.quaternion import Quaternion


DEFAULT_BODY_FRD_TO_CAMERA = np.array(
    [
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=float,
)


@dataclass(frozen=True)
class CameraProjection:
    pixel: np.ndarray
    depth_m: float
    visible: bool
    point_camera: np.ndarray


class PinholeCamera:
    """Pinhole camera projecting NED world points into an ego-mounted camera.

    Simulator frames are NED for world and FRD for body. The camera optical frame is
    x-right, y-down, z-forward. The default camera is forward-facing on the body.
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fx: float = 400.0,
        fy: float = 400.0,
        cx: float | None = None,
        cy: float | None = None,
        position_body_frd=None,
        rotation_body_to_camera=None,
    ):
        self.width = int(width)
        self.height = int(height)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Camera width and height must be positive")

        self.fx = float(fx)
        self.fy = float(fy)
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("Camera focal lengths must be positive")

        self.cx = 0.5 * float(self.width) if cx is None else float(cx)
        self.cy = 0.5 * float(self.height) if cy is None else float(cy)
        self.position_body_frd = (
            np.zeros(3, dtype=float) if position_body_frd is None else np.asarray(position_body_frd, dtype=float).copy()
        )
        if self.position_body_frd.shape != (3,):
            raise ValueError("position_body_frd must have shape (3,)")

        self.rotation_body_to_camera = (
            DEFAULT_BODY_FRD_TO_CAMERA.copy()
            if rotation_body_to_camera is None
            else np.asarray(rotation_body_to_camera, dtype=float).copy()
        )
        if self.rotation_body_to_camera.shape != (3, 3):
            raise ValueError("rotation_body_to_camera must have shape (3, 3)")

    def project_world_point(self, point_ned, ego_position_ned, ego_quaternion_wxyz) -> CameraProjection:
        point_ned = np.asarray(point_ned, dtype=float)
        ego_position_ned = np.asarray(ego_position_ned, dtype=float)
        ego_quaternion_wxyz = np.asarray(ego_quaternion_wxyz, dtype=float)
        if point_ned.shape != (3,):
            raise ValueError("point_ned must have shape (3,)")
        if ego_position_ned.shape != (3,):
            raise ValueError("ego_position_ned must have shape (3,)")
        if ego_quaternion_wxyz.shape != (4,):
            raise ValueError("ego_quaternion_wxyz must have shape (4,)")

        q_norm = float(np.linalg.norm(ego_quaternion_wxyz))
        if q_norm <= 1e-9:
            ego_quaternion_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        else:
            ego_quaternion_wxyz = ego_quaternion_wxyz / q_norm

        rel_ned = point_ned - ego_position_ned
        rel_body_frd = Quaternion.Mfg(ego_quaternion_wxyz) @ rel_ned
        rel_from_camera_body = rel_body_frd - self.position_body_frd
        point_camera = self.rotation_body_to_camera @ rel_from_camera_body

        x_cam, y_cam, z_cam = [float(v) for v in point_camera]
        if z_cam <= 0.0:
            return CameraProjection(
                pixel=np.array([np.nan, np.nan], dtype=float),
                depth_m=z_cam,
                visible=False,
                point_camera=point_camera,
            )

        pixel = np.array(
            [
                self.fx * x_cam / z_cam + self.cx,
                self.fy * y_cam / z_cam + self.cy,
            ],
            dtype=float,
        )
        visible = bool(0.0 <= pixel[0] < self.width and 0.0 <= pixel[1] < self.height)
        return CameraProjection(pixel=pixel, depth_m=z_cam, visible=visible, point_camera=point_camera)
