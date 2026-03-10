from ..common_forces.passive_sphere_aero import PassiveSphereAeroForceModel
from .forces import IrisQuadForceModel
from .initial_state import make_initial_state
from .parameters import IrisParameters


def make_parameters():
    return IrisParameters()


def make_force_models(parameters):
    _ = parameters
    return [IrisQuadForceModel(), PassiveSphereAeroForceModel()]
