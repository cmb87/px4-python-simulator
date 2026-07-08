# How to initalize the new vehicles in PX4

Place new FMUS files (content of ./airframes) to 

    cp ./airframes/* <PX4 repo>/ROMFS/px4fmu_common/init.d-posix/airframes/.

Register them in **<PX4 repo>/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt**

    ...
	10021_none_x8
	10022_none_ts06
    ...

Open **<PX4 repo>/src/modules/simulation/simulator_mavlink/CMakeLists.txt** and add

```
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

Run a new session in PX4

    make px4_sitl none_ts06

This will start with the new vehicle. Start the simulator in the repo root directory

    SIM_VEHICLE_MODEL=ts06 python3 src/main.py

or simply run 

    start_ts06_websocket.sh

for convience.

# 3D live flight visualization

Go to **https://cmb87.github.io/** and select the simulator tab. Connect via websocket to the simulator and enjoy. Note the websocket must be activated (it is when you start with start_ts06_websocket.sh).


