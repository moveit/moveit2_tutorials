#!/bin/bash

# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

# Script to call the isaac launch system (see python.sh in the ISAAC_SCRIPT_DIR)

CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
NEW_ARGS=()
ISAAC_SIM_PATH_OVERRIDE=""

# Parse arguments: extract custom --isaac-path if provided, and resolve script paths
while [[ $# -gt 0 ]]; do
    case "$1" in
        --isaac-path)
            ISAAC_SIM_PATH_OVERRIDE="$2"
            shift 2
            ;;
        --isaac-path=*)
            ISAAC_SIM_PATH_OVERRIDE="${1#*=}"
            shift
            ;;
        -*)
            NEW_ARGS+=("$1")
            shift
            ;;
        *)
            if [[ -f "$1" ]]; then
                NEW_ARGS+=("$(realpath "$1")")
            elif [[ -f "${CUR_SCRIPT_DIR}/$1" ]]; then
                NEW_ARGS+=("${CUR_SCRIPT_DIR}/$1")
            else
                NEW_ARGS+=("$1")
            fi
            shift
            ;;
    esac
done

# 1. Check if user provided an override or environment variable
CANDIDATE_DIRS=()
if [[ -n "$ISAAC_SIM_PATH_OVERRIDE" ]]; then
    CANDIDATE_DIRS+=("$ISAAC_SIM_PATH_OVERRIDE")
fi
if [[ -n "$ISAAC_SIM_PATH" ]]; then
    CANDIDATE_DIRS+=("$ISAAC_SIM_PATH")
fi
if [[ -n "$ISAAC_PATH" ]]; then
    CANDIDATE_DIRS+=("$ISAAC_PATH")
fi

# 2. Check standard installation paths and in-container paths
CANDIDATE_DIRS+=(
    "/isaac-sim"
    "$HOME/isaacsim"
    "/opt/nvidia/isaac-sim"
    "/opt/isaac-sim"
)

# 3. Check for versioned folders in ~/.local/share/ov/pkg
OV_PKG_DIR="$HOME/.local/share/ov/pkg"
if [[ -d "$OV_PKG_DIR" ]]; then
    for dir in $(ls -d -- "$OV_PKG_DIR"/isaac_sim-* "$OV_PKG_DIR"/isaac-sim-* 2>/dev/null); do
        CANDIDATE_DIRS+=("$dir")
    done
fi

# Check candidate directories for python.sh (excluding this script's directory to avoid recursion)
for dir in "${CANDIDATE_DIRS[@]}"; do
    if [[ -n "$dir" && "$dir" != "$CUR_SCRIPT_DIR" && -f "$dir/python.sh" ]]; then
        pushd "$dir" > /dev/null
        ./python.sh "${NEW_ARGS[@]}"
        RET=$?
        popd > /dev/null
        exit $RET
    fi
done

# 4. If no local installation was found, check if an Isaac Sim Docker container is running
DOCKER_CONTAINER="${ISAAC_DOCKER_CONTAINER:-isaac-sim}"
if command -v docker &> /dev/null; then
    RUNNING_CONTAINER=$(docker ps --filter "name=${DOCKER_CONTAINER}" --filter "status=running" --format "{{.Names}}" | head -n 1)
    if [[ -z "$RUNNING_CONTAINER" ]]; then
        RUNNING_CONTAINER=$(docker ps --filter "ancestor=nvcr.io/nvidia/isaac-sim" --filter "status=running" --format "{{.Names}}" | head -n 1)
    fi

    if [[ -n "$RUNNING_CONTAINER" ]]; then
        echo "Detected running Isaac Sim Docker container: '$RUNNING_CONTAINER'"
        echo "Forwarding execution into container..."

        CONTAINER_ARGS=()
        for arg in "${NEW_ARGS[@]}"; do
            if [[ -f "$arg" ]]; then
                fname=$(basename "$arg")
                docker cp "$arg" "${RUNNING_CONTAINER}:/tmp/${fname}"
                CONTAINER_ARGS+=("/tmp/${fname}")
            else
                CONTAINER_ARGS+=("$arg")
            fi
        done

        DOCKER_TTY_FLAG=""
        [ -t 0 ] && DOCKER_TTY_FLAG="-t"

        DOCKER_ENV_FLAGS=()
        if [[ -n "$DISPLAY" ]]; then
            DOCKER_ENV_FLAGS+=("-e" "DISPLAY=${DISPLAY}")
        fi

        docker exec -i $DOCKER_TTY_FLAG "${DOCKER_ENV_FLAGS[@]}" "${RUNNING_CONTAINER}" /isaac-sim/python.sh "${CONTAINER_ARGS[@]}"
        exit $?
    fi
fi

echo "No valid Isaac Sim installation or running container found."
echo ""
echo "Troubleshooting:"
echo "  - If Isaac Sim is installed locally, set ISAAC_SIM_PATH:"
echo "      export ISAAC_SIM_PATH=/path/to/isaacsim"
echo "    or pass:"
echo "      ./python.sh --isaac-path /path/to/isaacsim <script.py>"
echo "  - If running Isaac Sim in Docker, ensure the container is running and named 'isaac-sim' (or set ISAAC_DOCKER_CONTAINER):"
echo "      docker run --name isaac-sim --gpus all --network=host --ipc=host -e ACCEPT_EULA=Y nvcr.io/nvidia/isaac-sim:6.1.0  # or 4.5.0"
echo "    Alternatively, run ./python.sh directly from inside the container."
exit 1
