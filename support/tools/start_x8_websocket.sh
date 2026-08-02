#!/bin/bash

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Define paths relative to the script directory
VENV_ACTIVATE="${REPO_ROOT}/.venv/bin/activate"

# Verify virtual environment exists
if [ ! -f "$VENV_ACTIVATE" ]; then
    echo "Error: Virtual environment activation script not found at: $VENV_ACTIVATE"
    exit 1
fi

# Set required environment variables
export SIM_GT_OUTPUT_MODE="websocket"
export SIM_VEHICLE_MODEL="x8"
export SIM_GT_WS_HOST="0.0.0.0"
export SIM_GT_WS_PORT="8765"

# Cleanup function to kill background processes on exit (Ctrl+C)
cleanup() {
    echo -e "\n[Launcher] Shutting down background processes..."
    # Disable the trap to prevent recursion
    trap - INT TERM EXIT
    if [ -n "$SIM_PID" ]; then
        kill "$SIM_PID" 2>/dev/null
    fi
    exit 0
}
trap cleanup INT TERM EXIT

# Start Python Simulator
echo "Starting Python Simulator (Model: x8, Output: WebSocket)..."
cd "${REPO_ROOT}" || exit 1
source "$VENV_ACTIVATE"
python src/main.py &
SIM_PID=$!

# Keep script running and wait for background processes
wait "$SIM_PID"
