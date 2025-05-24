cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(Arty)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

option(DISABLE_GUI "Set to true to disable the SDL2-based GUI" OFF)
option(USE_STD_THREAD "Set to true to enable C++17/C++20 features as a fallback if OpenMP was not found." ON)
option(USE_EMBREE "Set to true to use Embree for scene traversal" OFF)

if (USE_EMBREE)
    find_package(Embree)
endif ()

if (MSVC)
    # MSVC does not report the correct __cplusplus value unless this flag is set
    add_compile_options("/Zc:__cplusplus")
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif ()

find_package(OpenMP 3.0 QUIET)
find_package(PNG 1.6 REQUIRED)
find_package(JPEG REQUIRED)
find_package(TIFF REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

include_directories(contrib/include/tinyexr)

if (NOT WIN32)
    add_definitions(-DCOLORIZE_LOG)
endif ()

# Request C++20, but fall back to C++17
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 17)

add_subdirectory(src)
