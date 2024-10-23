#!/bin/bash

: """
Tries to build and invokes cluster_extraction
"""

set -e

MY_DIR=$(dirname -- $(realpath -- $0))
MY_NAME=$(basename --  $0)
BUILD_DIR="build"
PARAMS_FILE="params.json"

build_fn() {
    [ ! -e $BUILD_DIR ] && cmake -B $BUILD_DIR
    cd $BUILD_DIR && make -j $(($(nproc) + $(nproc)/2))
}

exec_fn() {
    if [ "$1" = "-h" ]; then
        echo "Usage: $MY_NAME exec <dataset_name>"
        exit 0
    fi

    [ ! "$1" ] && { echo -e "Dataset does not exist. See usage with -h option"; exit 1; }
    DATASET_DIR="$MY_DIR/$1"
    PARAMS_ABS_FILENAME="$MY_DIR/$PARAMS_FILE"

    ./$BUILD_DIR/cluster_extraction $DATASET_DIR $PARAMS_ABS_FILENAME
}

# Compilation
if [ "$1" = "build" ]; then
    build_fn
    exit 0
fi

# Invocation
if [ "$1" = "exec" ]; then
    exec_fn $2
    exit 0
fi


# Tuning
if [ "$1" = "tune" ]; then
    while [[ 0 ]]; do
        exec_fn $2
        echo "Press key to restart the program"
        read -n1 -s
    done
    exit 0
fi

echo "Usage: $MY_NAME (build|exec|tune)"
echo "  tune - Cyclic execution of cluster_extraction binary for params tuning"
echo "  exec - Execution-only of cluster_extraction binary"
echo "  build - Just compile cluster_extraction binary"
exit 0