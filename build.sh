#!/bin/bash
mkdir -p build && cdbuild

cmake -G "Unix Makefiles" -D "CMAKE_TOOLCHAIN_FILE=../CMake/GNU-ARM-Toolchain.cmake" -DBUILD_TYPE=RELEASE
make -j4