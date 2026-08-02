# PX4 Integration

This directory contains PX4 airframe files and notes for using the simulator with custom PX4 SITL vehicle targets.

## Files In This Folder

- `airframes/10021_none_x8`
- `airframes/10022_none_ts06`
- `airframes/ts06_params.params`

These airframes are meant to be copied into a PX4 checkout and registered there.

## Add The Vehicles To PX4

Assume your PX4 checkout is at `<PX4 repo>`.

### 1. Copy the airframe files

```bash
cp px4/airframes/* <PX4 repo>/ROMFS/px4fmu_common/init.d-posix/airframes/
```

### 2. Register the airframes in PX4

Edit:

- `<PX4 repo>/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`

Add the custom airframes to the list:

```text
10021_none_x8
10022_none_ts06
```

### 3. Add SITL launch targets

Edit:

- `<PX4 repo>/src/modules/simulation/simulator_mavlink/CMakeLists.txt`

Add these targets:

```cmake
add_custom_target(none_x8
	COMMAND ${CMAKE_COMMAND} -E env
		PX4_SYS_AUTOSTART=10021
		python3 -c "import os,sys; i=os.getenv('PX4_INSTANCE','0'); os.execv(sys.argv[1],[sys.argv[1],'-i',i])"
		$<TARGET_FILE:px4>
	WORKING_DIRECTORY ${SITL_WORKING_DIR}
	USES_TERMINAL
	VERBATIM
	DEPENDS
		px4
		${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/10021_none_x8
	COMMENT "launching px4 none_x8 (PX4_INSTANCE -> -i at runtime)"
)

add_custom_target(none_ts06
	COMMAND ${CMAKE_COMMAND} -E env
		PX4_SYS_AUTOSTART=10022
		python3 -c "import os,sys; i=os.getenv('PX4_INSTANCE','0'); os.execv(sys.argv[1],[sys.argv[1],'-i',i])"
		$<TARGET_FILE:px4>
	WORKING_DIRECTORY ${SITL_WORKING_DIR}
	USES_TERMINAL
	VERBATIM
	DEPENDS
		px4
		${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/10022_none_ts06
	COMMENT "launching px4 none_ts06 (PX4_INSTANCE -> -i at runtime)"
)
```

### 4. Rebuild PX4 SITL

```bash
cd <PX4 repo>
make px4_sitl_default
```

## Run PX4 With This Simulator

Start the Python simulator from this repo root first:

```bash
SIM_VEHICLE_MODEL=ts06 python3 src/main.py
```

If you also want websocket ground truth:

```bash
SIM_VEHICLE_MODEL=ts06 SIM_GT_OUTPUT_MODE=websocket python3 src/main.py
```

Then start PX4 SITL from your PX4 checkout:

```bash
make px4_sitl none_ts06
```

For the X8 target:

```bash
SIM_VEHICLE_MODEL=x8 python3 src/main.py
```

and in PX4:

```bash
make px4_sitl none_x8
```

## Adding A Completely New PX4 Vehicle

If you create a new simulator vehicle, you usually need matching changes on both sides.

### Simulator side

1. Create `src/vehicles/<name>/`
2. Add `parameters.py`, `forces.py`, `initial_state.py`, and `definition.py`
3. Register the new model in `src/vehicles/vehicle_catalog.py`

### PX4 side

1. Create a new airframe file under `px4/airframes/`
2. Choose a new PX4 autostart ID
3. Copy that airframe into the PX4 checkout
4. Add it to PX4's airframe `CMakeLists.txt`
5. Add a matching `none_<name>` target in PX4's `simulator_mavlink/CMakeLists.txt`
6. Rebuild PX4 SITL

The simulator-side `SIM_VEHICLE_MODEL=<name>` and the PX4-side `none_<name>` launch target should refer to the same physical vehicle concept.

## Visualization

To view websocket ground truth, open:

- `https://cmb87.github.io/`

and connect it to the simulator websocket.
