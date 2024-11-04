#!/bin/bash

: """
Tries to build and run the tracker
"""

set -e

MY_DIR=$(dirname -- $(realpath -- $0))
MY_NAME=$(basename --  $0)
BUILD_DIR="build"

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

    ./$BUILD_DIR/main $DATASET_DIR
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


echo "Usage: $MY_NAME (build|exec)"
echo "  exec - Execution-only of cluster_extraction binary"
echo "  build - Just compile cluster_extraction binary"
exit 0