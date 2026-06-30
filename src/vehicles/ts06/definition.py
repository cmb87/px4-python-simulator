from .forces import Ts06ForceModel
from .initial_state import make_initial_state
from .parameters import Ts06Parameters


def make_parameters():
    return Ts06Parameters()


def make_force_models(parameters):
    _ = parameters
    return [Ts06ForceModel()]
