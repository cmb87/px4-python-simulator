#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The project root is one level up
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="px4-python-simulator"
IMAGE_TAG="latest"

echo "=================================================="
echo "Building Docker image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "Context: ${PROJECT_ROOT}"
echo "=================================================="

# Build the Docker image from the project root context
docker build -t "${IMAGE_NAME}:${IMAGE_TAG}" -f "${SCRIPT_DIR}/Dockerfile" "${PROJECT_ROOT}"

echo "=================================================="
echo "Build complete: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "=================================================="
