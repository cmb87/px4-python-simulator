from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping

import numpy as np

from .iris import definition as iris
from .ts06 import definition as ts06
from .x8 import definition as x8


@dataclass(frozen=True)
class VehicleDefinition:
    name: str
    make_parameters: Callable[[], Any]
    make_force_models: Callable[[Any], list[Any]]
    make_initial_state: Callable[[Mapping[str, Any] | None], np.ndarray]


VEHICLES: dict[str, VehicleDefinition] = {
    "x8": VehicleDefinition(
        name="x8",
        make_parameters=x8.make_parameters,
        make_force_models=x8.make_force_models,
        make_initial_state=x8.make_initial_state,
    ),
    "iris": VehicleDefinition(
        name="iris",
        make_parameters=iris.make_parameters,
        make_force_models=iris.make_force_models,
        make_initial_state=iris.make_initial_state,
    ),
    "ts06": VehicleDefinition(
        name="ts06",
        make_parameters=ts06.make_parameters,
        make_force_models=ts06.make_force_models,
        make_initial_state=ts06.make_initial_state,
    ),
}


def get_vehicle_definition(name: str) -> VehicleDefinition:
    key = str(name).strip().lower()
    try:
        return VEHICLES[key]
    except KeyError as exc:
        joined = "|".join(list_vehicle_models())
        raise ValueError(f"Unknown vehicle model '{name}'. Available models: {joined}") from exc


def list_vehicle_models() -> tuple[str, ...]:
    return tuple(sorted(VEHICLES.keys()))
