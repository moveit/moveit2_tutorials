#!/bin/bash

# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

# Script to call the isaac launch system (see python.sh in the ISAAC_SCRIPT_DIR)

NEW_OV_PKG_DIR=$HOME/isaacsim
OV_PKG_DIR=$HOME/.local/share/ov/pkg
ISAAC_SCRIPT_DIRS=()

# Prepend the path to all arguments passed in
CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
NEW_ARGS=""
for arg in "$@"; do
    NEW_ARGS="${NEW_ARGS} ${CUR_SCRIPT_DIR}/${arg}"
done

# Check if ~/isaacsim/python.sh exists
if [[ -f $NEW_OV_PKG_DIR/python.sh ]]; then
    # Use ~/isaacsim directly if python.sh exists
    pushd $NEW_OV_PKG_DIR
    ./python.sh $NEW_ARGS
    popd
    exit 0
fi

# Fallback to older locations if ~/isaacsim/python.sh does not exist

# Check for underscore-separated directory names (e.g., isaac_sim-*)
for ISAAC_SCRIPT_DIR in $(ls -d -- $OV_PKG_DIR/isaac_sim-* 2>/dev/null); do
    ISAAC_VER=${ISAAC_SCRIPT_DIR//$OV_PKG_DIR\/isaac_sim-/}
    if [[ "$ISAAC_VER" =~ ^(2022.2.0|2022.2.1|2023.1.0|2023.1.1)$ ]]; then
        ISAAC_SCRIPT_DIRS+=($ISAAC_SCRIPT_DIR)
    fi
done

# Check for dash-separated directory names (e.g., isaac-sim-*)
for ISAAC_SCRIPT_DIR in $(ls -d -- $OV_PKG_DIR/isaac-sim-* 2>/dev/null); do
    ISAAC_SCRIPT_DIRS+=($ISAAC_SCRIPT_DIR)
done

# Navigate to the most recent Isaac Sim directory and execute the script
if [[ ${#ISAAC_SCRIPT_DIRS[@]} -gt 0 ]]; then
    pushd ${ISAAC_SCRIPT_DIRS[${#ISAAC_SCRIPT_DIRS[@]}-1]}
    ./python.sh $NEW_ARGS
    popd
else
    echo "No valid Isaac Sim installation found."
    exit 1
fi
