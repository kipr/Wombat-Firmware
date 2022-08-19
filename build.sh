#!/bin/bash
mkdir -p build && cd build
cmake -G "Unix Makefiles" -D "CMAKE_TOOLCHAIN_FILE=../CMake/GNU-ARM-Toolchain.cmake" ..
cmake -- build ..