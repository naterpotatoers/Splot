#!/bin/bash

# Navigate to the cpp directory
cd "$(dirname "$0")"/cpp

# Create a build directory and navigate into it
mkdir -p build && cd build

# Run CMake to configure the project and generate a build system
cmake ..

# Build the project
cmake --build . -- -j4

# Navigate back to the project root directory
cd ../..
