add_executable(arty
    main.cpp
    bvh.cpp
    bvh.h
    load_obj.cpp
    load_obj.h
    image.h
    image.cpp
    lights.h
    materials.h
    cameras.h
    intersect.h
    scene.h
    scene.cpp
    renderer.h
    samplers.h
    float4.h
    float3.h
    float2.h
    file_path.h
    common.h
    random.h
    options.h
    debug.h
    debug.cpp
    hash_grid.h
    algorithms/render_debug.cpp
    algorithms/render_pt.cpp
    
    algorithms/render_ppm.cpp)

target_link_libraries(arty PUBLIC PNG::PNG JPEG::JPEG TIFF::TIFF ${YAML_CPP_LIBRARIES})

if (OpenMP_FOUND)
    target_link_libraries(arty PUBLIC OpenMP::OpenMP_CXX)
else ()
    target_compile_definitions(arty PUBLIC -DUSE_STD_THREAD=1)
endif ()

if (USE_EMBREE)
    target_link_libraries(arty PUBLIC ${Embree_LIBRARY})
    target_include_directories(arty PUBLIC ${Embree_INCLUDE_DIR})
    target_compile_definitions(arty PUBLIC -DEMBREE)
endif ()

if (NOT DISABLE_GUI)
    find_package(SDL2 CONFIG REQUIRED)
    target_link_libraries(arty PUBLIC SDL2::SDL2)
else ()
    target_compile_definitions(arty PUBLIC -DDISABLE_GUI)
endif ()
