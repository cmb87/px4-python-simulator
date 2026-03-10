import numpy as np

from vehicle.vehicle_catalog import get_vehicle_definition, list_vehicle_models
from vehicle.world import World


def test_default_vehicle_registry_contains_known_models():
    names = set(list_vehicle_models())
    assert {"x8", "iris", "ts04"}.issubset(names)


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
