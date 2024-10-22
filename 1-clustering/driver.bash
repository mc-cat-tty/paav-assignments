#!/bin/bash

: """
Tries to build and invokes cluster_extraction
"""

set -e

MY_DIR=$(dirname -- $(realpath -- $0))
MY_NAME=$(basename --  $0)
BUILD_DIR="build"

build_fn() {
    [ ! -e $BUILD_DIR ] && cmake -B $BUILD_DIR
    cd $BUILD_DIR && make -j $(($(nproc)*1.5))
}

exec_fn() {
    if [ "$1" = "-h" ]; then
        echo "Usage: $MY_NAME exec <dataset_name>"
        exit 0
    fi

    DATASET_DIR="$MY_DIR/$1"
    [ ! -e $DATASET_DIR ] && echo "Dataset does not exist"

    ./cluster_extraction $DATASET_DIR
}

if [ "$1" = "-h" ]; then
    echo "Usage: $MY_NAME [exec|build]"
    echo "  exec - Execution-only of cluster_extraction binary"
    echo "  build - Just compile cluster_extraction binary"
    echo -e "\nPass no arguments to perform both compilation and execution"
    exit 0
fi

# Compilation
if [ "$1" != "exec" ]; then
    build_fn()
fi

# Invocation
if [ "$1" != "build" ]; then
    exec_fn()
fi