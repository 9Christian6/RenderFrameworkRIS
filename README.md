# Arty

Arty is an educational rendering framework. It contains all the basic infrastructure to render from simple to complex scenes, and will be used throughout the _Realistic Image Synthesis_ course.

## Disclaimers

Please do not distribute or share the source code of this program. If you want to use a version control software, please do so privately. Sharing source code publicly will be considered cheating.

The test scenes are provided with authorization from their respective authors. If you want to redistribute the scenes, make sure to follow their licenses, if any.

## Building

If you are not an expert in C++, you can try the `run.sh` (Linux / macOS), `run-nosdl.sh` (no SDL GUI) and `run.bat` (Windows) scripts. Provided you have a C++ compiler, git, and [CMake](https://cmake.org/) installed, these will automatically download and compile all dependencies (via [vcpkg](https://github.com/microsoft/vcpkg)), configure and compile a release build, and run arty with the given parameters.

For example, on Unix systems:
```
./run.sh test/cornell_box/cornell_box.yml -s 8
```

And on Windows:
```
run.bat test/cornell_box/cornell_box.yml -s 8
```

If the script fails you can (a) try and fix it based on the error messages (likely some missing build tool like CMake) or (b) follow the more involved instructions below.

### Linux

The dependencies are:

- [CMake](https://cmake.org/download/)
- [SDL2](https://www.libsdl.org/download-2.0.php)
- [libpng](http://www.libpng.org/pub/png/libpng.html)
- [libjpeg](https://libjpeg-turbo.org/)
- [libtiff](/http://www.libtiff.org/)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

All of these should be available from your package manager. After installing them, run the following to create a build folder and compile in Release mode:

```
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

It also does not hurt to create a Debug build in a separate folder (to reduce recompile times):
```
cmake -B build-debug -S . -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug
```

### Windows

Builing arty with Visual Studio requires at least Visual Studio 2019 version 16.8!

On Windows, we recommend using [vcpkg](https://github.com/microsoft/vcpkg) to download and compile all dependencies. Run the following in the folder that contains `arty`:

```
git clone https://github.com/microsoft/vcpkg
cd vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg install sdl2 libpng libjpeg-turbo tiff yaml-cpp embree3 --triplet x64-windows
```

Note that, for a Windows build, using Embree is mandatory. To create a build folder and build in Release mode, do the following:

```
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake -DUSE_EMBREE
cmake --build build --config Release
```

The compiled executable will be in `build/src/Release/arty.exe`
Replace the `../vcpkg` with the full path to `vcpkg` if it is not in the folder above.


## Running

Arty comes with a set of test scenes. They are present in the `test` directory. Each scene is described in a YAML file (`.yml` extension). To run Arty, use the following command:

    arty <path-to-scene>.yml

To get the full list of options, type:

    arty -h

By default, Arty runs the debug renderer (eye-light shading only). The camera position can be controlled using the keyboard arrows. The keypad `+`/`-` keys control the translation speed. The camera direction is controlled with the mouse. To enable camera control, keep the left mouse button pressed while moving the cursor. The `R` key cycles through the different rendering algorithms. The implementation of these algorithms is missing and will be your task.

## Scene format

The scene files are described in [YAML](http://yaml.org/). Here is an example of scene:

```yaml
---
# List of OBJ files
meshes: ["model.obj"]
# Camera definition
camera: !perspective_camera {
    eye: [-0.45,  1.5, -1.0],      # Position of the camera
    center: [-0.45, -10.0, -100],  # Point to look at
    up: [0, 1, 0],                 # Up vector
    fov: 60.0                      # Field of view, in degrees
}
# List of lights (optional, emissive materials in the OBJ file will be converted to area lights)
lights: [
    !point_light {
        position: [0, 10, 0],
        color: [100, 100, 0]
    },
    !triangle_light {
        v0: [0, 1, 2],              # First vertex
        v1: [0, 1, 3],              # Second vertex
        v2: [1, 1, 2],              # Third vertex
        color: [100, 0, 100]
    }
]
```

## Conventions

The conventions in Arty are as follows:

- Functions to sample a direction should return **normalized** vectors
- Rays generated from the camera should have a **normalized** direction
- Materials and lights expect a **normalized** direction for sampling and evaluation
- **No redundant normalization should be done otherwise**

By enforcing those conventions, we ensure that the code is correct _and_ fast. In debug mode, you can check whether this convention is actually followed using the `assert_normalized` macro in `common.h`.

## Documentation

The source code is thoroughly documented, and [Doxygen](http://www.doxygen.org/) can generate structured HTML pages from all the inline comments in the source files.
