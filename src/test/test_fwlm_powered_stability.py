import numpy as np

from dynamics.world import World


def test_fwlm_powered_flight_stays_finite_for_20s():
    y0 = np.zeros(13, dtype=float)
    y0[0:3] = np.array([0.0, 0.0, -300.0], dtype=float)
    y0[3] = 1.0
    y0[7:10] = np.array([20.0, 0.0, 0.0], dtype=float)

    world = World(vehicle_model="fwlm", y0=y0, u0=np.zeros(4), wind0=np.zeros(6))
    controls = np.array([0.75, 0.0, 0.0, 0.0], dtype=float)

    dt_us = 10_000
    for step in range(2000):
        world.set_controls(controls)
        out = world.update((step + 1) * dt_us, paused=False, freeze_dynamics=False)
        assert np.all(np.isfinite(out["y"]))
        assert np.all(np.isfinite(out["tau"]))
