#!/bin/bash

# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

# Script to call the isaac launch system (see python.sh in the ISAAC_SCRIPT_DIR)

ISAAC_SCRIPT_DIRS=($(ls -d -- $HOME/.local/share/ov/pkg/isaac_sim-*))

# Prepend the path to all arguments passed in
CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
NEW_ARGS=""
for arg in "$@"
do
    NEW_ARGS="${NEW_ARGS} ${CUR_SCRIPT_DIR}/${arg}"
done

pushd ${ISAAC_SCRIPT_DIRS[${#ISAAC_SCRIPT_DIRS[@]}-1]}
./python.sh $NEW_ARGS
popd
