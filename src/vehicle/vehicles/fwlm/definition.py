from .forces import FWLMAeroLUTForceModel, FWLMMotorForceModel
from .initial_state import make_initial_state
from .parameters import FWLMParameters


def make_parameters():
    return FWLMParameters()


def make_force_models(parameters):
    _ = parameters
    return [FWLMAeroLUTForceModel(), FWLMMotorForceModel()]
