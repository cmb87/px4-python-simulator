import sys
from importlib.util import find_spec
from pathlib import Path

_vehicle_spec = find_spec("vehicle")
if _vehicle_spec is not None and _vehicle_spec.submodule_search_locations is not None:
    _vehicle_dir = Path(next(iter(_vehicle_spec.submodule_search_locations))).resolve()
else:
    _vehicle_dir = Path(__file__).resolve().parents[2] / "vehicle"

if str(_vehicle_dir) not in sys.path:
    sys.path.insert(0, str(_vehicle_dir))

from vehicle.world import World

__all__ = ["World"]
