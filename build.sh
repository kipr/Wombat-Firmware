#/bin/bash
mkdir build && cd build
cmake .. -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../CMake/GNU-ARM-Toolchain.cmake
make -j4