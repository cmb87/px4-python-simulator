#!/bin/bash

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Define path to the websocket launch script
LAUNCH_SIM="./start_ts06_websocket.sh"

# Verify simulator script exists
if [ ! -f "$LAUNCH_SIM" ]; then
    echo "Error: Simulator script not found at: $LAUNCH_SIM"
    exit 1
fi

# Define paths relative to the script directory
VENV_ACTIVATE="${SCRIPT_DIR}/../../.venv/bin/activate"

# Verify virtual environment exists
if [ ! -f "$VENV_ACTIVATE" ]; then
    echo "Error: Virtual environment activation script not found at: $VENV_ACTIVATE"
    exit 1
fi

# ==============================================================================
# OPTIONAL TS06 SIMULATOR PARAMETER OVERRIDES
# Uncomment and adjust these to tweak flight physics on-the-fly without editing code
# ==============================================================================
# export SIM_TS06_X_WING="-0.013"   # Wing X-position relative to CG (m)
# export SIM_TS06_Y_WING="0.0"      # Wing Y-position relative to CG (m)
# export SIM_TS06_Z_WING="-0.04"    # Wing Z-position relative to CG (m)
# export SIM_TS06_K_T_P="0.02"      # Torque-to-thrust ratio
# export SIM_TS06_C_L_P="-0.4"      # Roll damping (opposes roll speed)
# export SIM_TS06_C_M_Q="-1.5"      # Pitch damping (opposes pitch speed)
# export SIM_TS06_C_N_R="-0.8"      # Yaw damping (opposes yaw speed)
# export SIM_TS06_C_N_BETA="0.35"   # Weathercock stability (yaws nose into wind)
# export SIM_TS06_C_L_BETA="-0.05"  # Dihedral effect (rolls wings level during sideslip)
# export SIM_TS06_C_Y_BETA="-0.3"   # Side force due to sideslip
# ==============================================================================

# ==============================================================================
# OPTIONAL PX4 AUTOPILOT PARAMETER OVERRIDES (PIDs, gains, tuning, etc.)
# Add any standard 'param set <PARAM_NAME> <VALUE>' commands to this array.
# They will be injected automatically into PX4 NuttShell (NSH) on startup.
# ==============================================================================
PX4_PARAMS=(
  # Multirotor Attitude Rate Gains (Dampened due to ultra-low physical inertias)
  #"param set MC_ROLLRATE_P 0.05"    # Default is 0.3 (Dampens violent roll oscillations)
  #"param set MC_PITCHRATE_P 0.08"   # Default is 0.3 (Dampens pitch oscillations)
  #"param set MC_YAWRATE_P 0.08"     # Default is 0.2

  # Fixed-Wing Rate Gains (Dampened to prevent control loop explosion in FW flight)
  "param set FW_RR_P 0.05"          # Default is 0.2 (Reduced roll rate gain)
  "param set FW_PR_P 0.12"          # Default is 0.5 (Tuned to prevent control loop self-excitation)
  "param set FW_PR_I 0.01"          # Prevent massive pitch integral windup during transition
  "param set FW_PR_D 0.005"         # Add active rate damping to kill high-frequency pitch oscillation
  "param set FW_YR_P 0.05"          # Default is 0.05 (Restore to standard damping)

  # Fixed-Wing Differential Thrust Scale Factors (Saves control loop from high motor leverage)
  "param set VT_FW_DIFTHR_S_R 0.10" # Default is 1.0 (Scale down aggressive roll differential thrust)
  "param set VT_FW_DIFTHR_S_P 0.15" # Default is 1.0 (Scale down aggressive pitch differential thrust)
  "param set VT_FW_DIFTHR_S_Y 0.05" # Default is 0.1 (Scale down aggressive yaw differential thrust)
)
# ==============================================================================

# Cleanup function to kill background processes on exit (Ctrl+C)
cleanup() {
    echo -e "\n[Launcher] Shutting down background processes..."
    # Disable the trap to prevent recursion
    trap - INT TERM EXIT
    if [ -n "$SIM_PID" ]; then
        kill "$SIM_PID" 2>/dev/null
    fi
    # Cleanly terminate any running PX4 processes inside the docker container
    docker exec px4-dev pkill -9 -f px4 2>/dev/null
    exit 0
}
trap cleanup INT TERM EXIT

# 1. Start Python Simulator in the background
echo "[Launcher] Starting Python Simulator (Model: ts06, Output: WebSocket)..."
cd "${SCRIPT_DIR}" || exit 1
$LAUNCH_SIM &
SIM_PID=$!

# Wait for simulator to open its ports and be ready
sleep 3

# 2. Start PX4 SITL inside the docker container and feed automated flight commands to its NSH shell
echo "[Launcher] Launching PX4 in 'px4-dev' container with automated Takeoff & Transition..."
(
  # Wait for PX4 to boot, initialize, and establish connection to lockstep simulator
  echo "[Automator] Waiting for connection & sensor initialization..." >&2
  sleep 8
  
  # Inject PX4 parameters if defined
  if [ ${#PX4_PARAMS[@]} -gt 0 ]; then
      echo "[Automator] Injecting custom PX4 PIDs & limits..." >&2
      for param in "${PX4_PARAMS[@]}"; do
          echo "$param"
          sleep 0.1
      done
  fi
  

  
  # Send commander transition to fixed-wing command
  echo "[Automator] Sending 'commander transition'..." >&2
  echo "commander transition"

  sleep 1

  # Send commander takeoff command
  echo "[Automator] Sending 'commander takeoff'..." >&2
  echo "commander takeoff"
  
  # Keep stdin pipe alive for PX4 shell
  cat
) | docker exec -i px4-dev make px4_sitl none_ts06

# Keep script running and wait for background processes
wait "$SIM_PID"
