# Raspberry Pi Pico Quad UART to USB Bridge

## Introduction
This project expands on Álvaro Fernández Rojas' 
[pico-uart-bridge](https://github.com/Noltari/pico-uart-bridge) project.
An additional 2 UARTS are added using the Raspberry Pi Pico's PIO peripherals.

## Requirements
* [Raspberry Pi Pico](https://www.raspberrypi.org/products/pico-series/pico-pi-zero)
* [Raspberry Pi Pico SDK version 1.3.0 (or later)](https://raspberrypi.github.io/pico-sdk-doxygen/)
* [CMake Version 3.13.0 (or later)](https://cmake.org/)
* [Make Version 4.3.0 (or later)](https://www.gnu.org/software/make/)
* [ARM GCC Compiler Toolchain Version 11.0 (or later)](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

## Build Instructions
* Clone this repository to your local machine.
* Ensure the environment variable `PICO_SDK_PATH` is set to the location of the Pico SDK.
* Build the project using CMake and Make.
    * Run `cmake -Bbuild` in the project root directory.
    * Run `cd build && make` to build the project.
## Project Status
* This project is a work in progress and make be broken at any given point.
* This project has not been tested thoroughly

### Current Limitations
* The project does not support a configurable PIO UART at the moment. Therefore to change the baud rate of a given
    PIO UART, you must rebuild the project and modify the BAUD_RATE macro in the source code.

## Project Milestones
 - [X] Add support for multiple UARTs
 - [ ] Add support for changing the baud rate on the PIO peripherals
 - [ ] Stress test 4 UART peripherals (2 hardware, 2 PIO) at 921600 baud