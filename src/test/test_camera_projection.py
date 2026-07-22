import numpy as np

from perception import PinholeCamera


IDENTITY_QUAT_WXYZ = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)


def test_target_straight_ahead_projects_to_center():
    camera = PinholeCamera(width=640, height=480, fx=400.0, fy=400.0)

    measurement = camera.project_world_point(
        point_ned=np.array([10.0, 0.0, 0.0]),
        ego_position_ned=np.zeros(3),
        ego_quaternion_wxyz=IDENTITY_QUAT_WXYZ,
    )

    assert measurement.visible
    assert np.allclose(measurement.pixel, np.array([320.0, 240.0]))
    assert np.isclose(measurement.depth_m, 10.0)


def test_target_to_body_right_increases_u_pixel():
    camera = PinholeCamera(width=640, height=480, fx=400.0, fy=400.0)

    measurement = camera.project_world_point(
        point_ned=np.array([10.0, 1.0, 0.0]),
        ego_position_ned=np.zeros(3),
        ego_quaternion_wxyz=IDENTITY_QUAT_WXYZ,
    )

    assert measurement.visible
    assert measurement.pixel[0] > 320.0
    assert np.isclose(measurement.pixel[1], 240.0)


def test_target_body_down_increases_v_pixel():
    camera = PinholeCamera(width=640, height=480, fx=400.0, fy=400.0)

    measurement = camera.project_world_point(
        point_ned=np.array([10.0, 0.0, 1.0]),
        ego_position_ned=np.zeros(3),
        ego_quaternion_wxyz=IDENTITY_QUAT_WXYZ,
    )

    assert measurement.visible
    assert np.isclose(measurement.pixel[0], 320.0)
    assert measurement.pixel[1] > 240.0


def test_target_behind_camera_is_not_visible():
    camera = PinholeCamera(width=640, height=480, fx=400.0, fy=400.0)

    measurement = camera.project_world_point(
        point_ned=np.array([-10.0, 0.0, 0.0]),
        ego_position_ned=np.zeros(3),
        ego_quaternion_wxyz=IDENTITY_QUAT_WXYZ,
    )

    assert not measurement.visible
    assert measurement.depth_m < 0.0
    assert np.isnan(measurement.pixel).all()


def test_target_outside_image_is_not_visible():
    camera = PinholeCamera(width=640, height=480, fx=400.0, fy=400.0)

    measurement = camera.project_world_point(
        point_ned=np.array([10.0, 20.0, 0.0]),
        ego_position_ned=np.zeros(3),
        ego_quaternion_wxyz=IDENTITY_QUAT_WXYZ,
    )

    assert not measurement.visible
    assert measurement.depth_m > 0.0
    assert measurement.pixel[0] >= camera.width


def test_two_vehicle_states_project_both_directions():
    camera = PinholeCamera(width=640, height=480, fx=400.0, fy=400.0)
    ego0 = np.zeros(13)
    ego0[3] = 1.0
    ego1 = np.zeros(13)
    ego1[0:3] = np.array([30.0, 5.0, 0.0])
    ego1[3] = 1.0

    cam0_sees_1 = camera.project_world_point(ego1[0:3], ego0[0:3], ego0[3:7])
    cam1_sees_0 = camera.project_world_point(ego0[0:3], ego1[0:3], ego1[3:7])

    assert cam0_sees_1.visible
    assert np.isclose(cam0_sees_1.depth_m, 30.0)
    assert not cam1_sees_0.visible
