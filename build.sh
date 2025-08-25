#!/bin/bash
#
#  Modified by: Tobias Madlberger*, 2025
#  * Corresponding author's email: tobias.madlberger@gmail.com
#

# Delete the build directory if it exists
if [ -d "build" ]; then
    rm -rf build
fi

mkdir build && cd build || exit

cmake -G "Unix Makefiles" -D "CMAKE_TOOLCHAIN_FILE=../CMake/GNU-ARM-Toolchain.cmake" ../
cmake --build . -- -j "$(nproc)"