cmake_minimum_required (VERSION 2.8.12)

# set the build type
if(NOT CMAKE_BUILD_TYPE) 
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message("Debug build.")
elseif(CMAKE_BUILD_TYPE MATCHES Release)
  message("Release build.")
endif()

# set search path for CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake/Modules)

# Project specific settings
set(APPLICATION_NAME "wombat")

project (${APPLICATION_NAME} C CXX ASM)

# STM32-specific settings
set(DEVICE_FAMILY STM32F427_437xx)
set(STM32_PLATFORM "STM32F427xx" CACHE STRING "Target microprocessor platform")

# Speed of external resonator
set(HSE_VALUE 24000000)

add_definitions(-DSTM32F427xx)
add_definitions(-DSTM32F427_437xx)
add_definitions(-DUSE_STDPERIPH_DRIVER)

# Used linker file
get_filename_component(LINKER_SCRIPT linker/STM32F427VITx_FLASH.ld ABSOLUTE)

# Use Link time Optimization? WARNING: This might break the code
#set(ADDITIONAL_CORE_FLAGS -flto)

# add libraries
add_subdirectory(libs)

# add main project
add_subdirectory(Firmware)