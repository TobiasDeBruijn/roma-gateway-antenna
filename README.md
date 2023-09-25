# stm32-cmake-template
Template for an STM32 C project with CMake support, while keeping support for CubeMX's HAL types.

## Features
- CMake build system
- Compatible with CubeMX & CubeIDE
>Note: CubeIDE completely ignores whatever you do in CMake, it does its own thing

## Compiling
Requirments:
- `gcc-arm-linux-gnueabihf`
- `openocd`
- `cmake`
- `make`

From the repository root:
1. `cmake -B build -DCMAKE_BUILD_TYPE=Release`
2. `cd build/`
3. `make`

## Uploading
From the repository root:
1. `openocd -f openocd-stlink.cfg -c "program build/smartbathroom-static-flusher.elf verify reset exit"`