import numpy as np

from vehicles.vehicle_catalog import get_vehicle_definition, list_vehicle_models
from dynamics.world import World


def test_default_vehicle_registry_contains_known_models():
    names = set(list_vehicle_models())
    assert {"x8", "iris", "ts06"}.issubset(names)


def test_each_registered_model_runs_one_step():
    for name in list_vehicle_models():
        world = World(vehicle_model=name)
        world.set_controls(np.zeros(4))
        out = world.update(10_000, paused=False, freeze_dynamics=True)
        assert out is not None
        assert out["y"].shape == (13,)
        assert out["ydot"].shape == (13,)


def test_unknown_vehicle_model_raises_value_error():
    try:
        get_vehicle_definition("does-not-exist")
        assert False, "Expected ValueError for unknown model"
    except ValueError:
        pass


def test_ts06_forces_and_moments():
    defn = get_vehicle_definition("ts06")
    P = defn.make_parameters()
    models = defn.make_force_models(P)
    model = models[0]
    
    # Initialize inputs
    y = np.zeros(13)
    y[3] = 1.0  # unit quaternion
    
    # Test 1: Zero speed, full throttle on all motors should generate positive x thrust
    # and zero moment (due to symmetry of motor positions about CG in Y/Z if thrust is equal)
    u = np.ones(4)
    wind = np.zeros(6)
    model.set_inputs(y=y, u=u, wind=wind, P=P)
    
    out = model.update(10_000, paused=False)
    force, torque = out[:3], out[3:]
    
    # Thrust should be strictly positive along body X
    assert force[0] > 0.0
    # Pitch/Yaw/Roll torques should be zero due to symmetry (y_motors cancel and z_motors cancel)
    assert np.allclose(torque, 0.0, atol=1e-10)
    
    # Test 2: Unbalanced throttle (e.g., motor 0 only) should generate pitching/yawing moments
    # Motor 0: x = -0.235, y = 0.1025, z = -0.1025 (Top Right)
    u_unbalanced = np.array([1.0, 0.0, 0.0, 0.0])
    model.set_inputs(u=u_unbalanced)
    out_unbalanced = model.update(20_000, paused=False)
    force_u, torque_unbalanced = out_unbalanced[:3], out_unbalanced[3:]
    
    assert force_u[0] > 0.0
    # Torque y_motor_0 = z_motor_0 * thrust_0 = -0.1025 * thrust_0 < 0 (pitches nose down)
    assert torque_unbalanced[1] < 0.0
    # Torque z_motor_0 = -y_motor_0 * thrust_0 = -0.1025 * thrust_0 < 0 (yaws nose left)
    assert torque_unbalanced[2] < 0.0


def test_ts06_aerodynamic_angle_filtering():
    defn = get_vehicle_definition("ts06")
    P = defn.make_parameters()
    models = defn.make_force_models(P)
    model = models[0]
    
    # Verify low pass characteristics:
    # When raw alpha is changed, filtered alpha converges slowly over multiple steps.
    y = np.zeros(13)
    y[3] = 1.0  # unit quaternion
    y[7] = 10.0  # forward speed 10 m/s -> alpha_raw = 0.0
    
    u = np.zeros(4)
    wind = np.zeros(6)
    model.set_inputs(y=y, u=u, wind=wind, P=P)
    
    # First step initializes the filter state
    model.update(0, paused=False)
    assert np.isclose(model.alpha_filtered, 0.0)
    
    # Sudden change: add vertical velocity 10 m/s -> alpha_raw becomes arctan2(10.0, 10.0) = 45 degrees (pi/4 rad)
    y_step = y.copy()
    y_step[9] = 10.0  # vertical downward speed 10 m/s
    model.set_inputs(y=y_step)
    
    # Step at t = 10ms (10,000 us).
    # dt = 0.01s
    # alpha_f = dt / (tau_alpha + dt)
    # alpha_filtered should be alpha_f * arctan2(10, 10)
    model.update(10_000, paused=False)
    tau_alpha = getattr(P, "tau_alpha", 0.05)
    expected_alpha = (0.01 / (tau_alpha + 0.01)) * np.arctan2(10.0, 10.0)
    assert np.isclose(model.alpha_filtered, expected_alpha)


def test_mavlink_clipping_helpers():
    from networking.mavlink_simulator import clip_int16, clip_uint16
    
    # Test signed 16-bit limits
    assert clip_int16(0.0) == 0
    assert clip_int16(32767.0) == 32767
    assert clip_int16(32768.1) == 32767  # overflow clipped
    assert clip_int16(-32768.0) == -32768
    assert clip_int16(-32769.4) == -32768  # underflow clipped
    assert clip_int16(100000.0) == 32767   # extreme overflow clipped
    
    # Test unsigned 16-bit limits
    assert clip_uint16(0.0) == 0
    assert clip_uint16(-50.0) == 0         # underflow clipped
    assert clip_uint16(65535.0) == 65535
    assert clip_uint16(65536.0) == 65535   # overflow clipped


def test_ts06_weathercock_stability():
    defn = get_vehicle_definition("ts06")
    P = defn.make_parameters()
    
    # Positive sideslip: positive velocity along body Y (moving right relative to air, wind from right)
    y_pos_beta = np.zeros(13)
    y_pos_beta[3] = 1.0  # unit quaternion
    y_pos_beta[7] = 10.0  # Forward speed 10 m/s
    y_pos_beta[8] = 2.0  # positive lateral speed -> positive beta
    
    u = np.zeros(4)
    wind = np.zeros(6)
    
    model_pos = defn.make_force_models(P)[0]
    model_pos.set_inputs(y=y_pos_beta, u=u, wind=wind, P=P)
    out_pos_beta = model_pos.update(10_000, paused=False)
    torque_pos_beta = out_pos_beta[3:]
    
    # Aerodynamic yaw moment (index 2 of torque) should be positive to turn nose right (into wind)
    assert torque_pos_beta[2] > 0.0
    
    # Negative sideslip: negative velocity along body Y
    y_neg_beta = np.zeros(13)
    y_neg_beta[3] = 1.0  # unit quaternion
    y_neg_beta[7] = 10.0  # Forward speed 10 m/s
    y_neg_beta[8] = -2.0  # negative lateral speed -> negative beta
    
    model_neg = defn.make_force_models(P)[0]
    model_neg.set_inputs(y=y_neg_beta, u=u, wind=wind, P=P)
    out_neg_beta = model_neg.update(10_000, paused=False)
    torque_neg_beta = out_neg_beta[3:]
    
    # Aerodynamic yaw moment (index 2 of torque) should be negative to turn nose left (into wind)
    assert torque_neg_beta[2] < 0.0


