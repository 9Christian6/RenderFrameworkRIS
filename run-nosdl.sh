#!/bin/bash

set -e

if [ ! -d "vcpkg" ]
then
    git clone https://github.com/microsoft/vcpkg
    ./vcpkg/bootstrap-vcpkg.sh
fi

if [ ! -d "build" ]
then
    ./vcpkg/vcpkg install libpng libjpeg-turbo tiff yaml-cpp
    cmake -B build -S . -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_TOOLCHAIN_FILE=./vcpkg/scripts/buildsystems/vcpkg.cmake -DDISABLE_GUI=1
fi

cmake --build build
./build/src/arty $*