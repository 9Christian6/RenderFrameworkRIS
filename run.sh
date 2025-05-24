#!/bin/bash

set -e

if [ ! -d "vcpkg" ]
then
    git clone https://github.com/microsoft/vcpkg
    ./vcpkg/bootstrap-vcpkg.sh
fi

if [ ! -d "build" ]
then
    ./vcpkg/vcpkg install sdl2 libpng libjpeg-turbo tiff yaml-cpp
    cmake -B build -S . -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_TOOLCHAIN_FILE=./vcpkg/scripts/buildsystems/vcpkg.cmake
fi

cmake --build build -j 4
./build/src/arty $*