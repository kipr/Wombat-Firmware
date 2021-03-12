Wombat Firmware
=======
> To view the history of this project, go to https://github.com/kipr/Wallaby-Firmware. This is a direct copy of the Wallaby2-V8 branch of that repository.

# General

This project provides firmware for the STM32F427VIT6 Microcontroller on the Wombat.

It is currently under active development by KIPR.

Open Source build provided with lots of :coffee: by [HTL Wiener Neustadt](https://robo4you.at/)

# Structure

* build
    * This folder contains the output wallaby.bin file, which can be flashed onto the microcontroller.
* CMake
    * Toolchain file, to compile the firmare using arm-gcc
* docs
    * Datasheets, Schematics and Bill of Materials for the Wombat Controller
* Firwmare
    * Actual firmware reading the sensors, controlling the motors, ...
* libs
    * STM32 Standard Peripheral Library used to interact with the different periphials of the microcrontroller
* linker
    * STM32F4 linker script

## Run/Build with Docker

> sudo service docker stop

> sudo service docker start

> sudo docker-compose up --build

This will execute all the builds and output the binary file into the build folder.

The build folder will be created during runtime.


## Run/Build Directly

(Requires PATH to be setup correctly for gcc/G++)

> sudo build.sh


### Author(s): 
 
Joshua Southerland (2015)

Zachary Sasser (2019)


#### Contributors:
 
Konstantin Lampalzer (2020) - Added Docker Support

Want to Contribute? Start Here!:
https://github.com/kipr/KIPR-Development-Toolkit
