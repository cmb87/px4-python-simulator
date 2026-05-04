from .forces import WingX8ForceModel
from .initial_state import make_initial_state
from .parameters import X8Parameters


def make_parameters():
    return X8Parameters()


def make_force_models(parameters):
    _ = parameters
    return [WingX8ForceModel()]
