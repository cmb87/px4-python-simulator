#!/bin/bash

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Define paths relative to the script directory
VENV_ACTIVATE="${SCRIPT_DIR}/../../.venv/bin/activate"

# Verify virtual environment exists
if [ ! -f "$VENV_ACTIVATE" ]; then
    echo "Error: Virtual environment activation script not found at: $VENV_ACTIVATE"
    exit 1
fi

# Set required environment variables
export SIM_GT_OUTPUT_MODE="flightgear_udp"
export SIM_VEHICLE_MODEL="x8"
export SIM_FG_UDP_HOST="127.0.0.1"
export SIM_FG_UDP_PORT="5503"

# Cleanup function to kill background processes on exit (Ctrl+C)
cleanup() {
    echo -e "\n[Launcher] Shutting down background processes..."
    # Disable the trap to prevent recursion
    trap - INT TERM EXIT
    if [ -n "$FG_PID" ]; then
        kill "$FG_PID" 2>/dev/null
    fi
    if [ -n "$SIM_PID" ]; then
        kill "$SIM_PID" 2>/dev/null
    fi
    exit 0
}
trap cleanup INT TERM EXIT

# # Start FlightGear
# echo "Starting FlightGear..."
# cd "${SCRIPT_DIR}/flightgear" || exit 1
# ./start_fg.sh &
# FG_PID=$!

# # Wait for FlightGear to start up and open its port
# echo "Waiting for FlightGear to initialize..."
# sleep 3

# Start Python Simulator
echo "Starting Python Simulator (Model: X8)..."
cd "${SCRIPT_DIR}" || exit 1
source "$VENV_ACTIVATE"
python src/main.py &
SIM_PID=$!

# Keep script running and wait for background processes
wait "$FG_PID" "$SIM_PID"
