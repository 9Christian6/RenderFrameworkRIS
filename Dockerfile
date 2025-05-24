FROM ubuntu:latest

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update; \
    apt-get install -y build-essential curl zip unzip tar git pkg-config cmake ninja-build

# Enable this to use clang instead (not recommended other than for testing)
# RUN apt-get install clang
# ENV CC=/usr/bin/clang CXX=/usr/bin/clang++ CMAKE_GENERATOR=Ninja

ENV CMAKE_GENERATOR=Ninja

COPY . /usr/src/arty
WORKDIR /usr/src/arty
RUN git clone https://github.com/microsoft/vcpkg; \
    ./vcpkg/bootstrap-vcpkg.sh; \
    ./vcpkg/vcpkg install libpng libjpeg-turbo tiff yaml-cpp; \
    cmake -B build -S . -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_TOOLCHAIN_FILE=./vcpkg/scripts/buildsystems/vcpkg.cmake -DDISABLE_GUI=1

ENTRYPOINT ["./run-nosdl.sh"]
CMD ["test/cornell_box/cornell_box.yml"]