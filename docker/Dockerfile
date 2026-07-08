FROM python:3.10-slim

WORKDIR /app

# Install system dependencies needed to build some packages (if any)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy packaging configuration and readme
COPY pyproject.toml README.md ./

# Copy source code and other required assets
COPY src/ ./src/

# Install the simulation tool locally
RUN pip install --no-cache-dir .

# Expose key ports:
# 4560 - MAVLink connection TCP port
# 8765 - Ground Truth WebSocket publisher port
EXPOSE 4560/tcp
EXPOSE 8765/tcp

# Run the simulator by default
CMD ["px4-python-sitl"]
