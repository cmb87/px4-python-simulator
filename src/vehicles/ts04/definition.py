from ..common_forces.passive_sphere_aero import PassiveSphereAeroForceModel
from .forces import TS04BlendedPassiveAeroForceModel, TS04ForceModel
from .initial_state import make_initial_state
from .parameters import TS04Parameters


def make_parameters():
    return TS04Parameters()


def make_force_models(parameters):
    passive_model = str(getattr(parameters, "ts04_passive_aero_model", "blended")).strip().lower()
    if passive_model == "sphere":
        aero_model = PassiveSphereAeroForceModel()
    else:
        aero_model = TS04BlendedPassiveAeroForceModel()
    return [TS04ForceModel(), aero_model]
