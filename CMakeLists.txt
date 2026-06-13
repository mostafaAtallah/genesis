cmake_minimum_required(VERSION 3.16)

project(genesis LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BOX2D_BUILD_TESTS OFF CACHE BOOL "Disable Box2D tests" FORCE)
set(BOX2D_BUILD_SAMPLES OFF CACHE BOOL "Disable Box2D samples" FORCE)
set(BOX2D_BUILD_DOCS OFF CACHE BOOL "Disable Box2D docs" FORCE)

add_executable(genesis
    src/main.cpp
    src/CompoundObject.cpp
    src/FunctionalSystem.cpp
    src/ResourceNode.cpp
)

target_include_directories(genesis
    PRIVATE
        include
)

if (EXISTS "C:/box2d-main/CMakeLists.txt")
    add_subdirectory("C:/box2d-main" "${CMAKE_BINARY_DIR}/box2d")
    target_link_libraries(genesis PRIVATE box2d::box2d)
    target_include_directories(genesis PRIVATE "C:/box2d-main/include")
else()
    message(FATAL_ERROR "Box2D source not found at C:/box2d-main")
endif()

find_path(RAYLIB_INCLUDE_DIR
    NAMES raylib.h
    HINTS
        $ENV{RAYLIB_ROOT}/include
        C:/raylib/include
)

find_library(RAYLIB_LIBRARY
    NAMES raylib
    HINTS
        $ENV{RAYLIB_ROOT}/lib
        C:/raylib/lib
)

if (NOT RAYLIB_INCLUDE_DIR OR NOT RAYLIB_LIBRARY)
    message(FATAL_ERROR "Raylib not found. Set RAYLIB_ROOT or install Raylib in C:/raylib.")
endif()

target_include_directories(genesis
    PRIVATE
        ${RAYLIB_INCLUDE_DIR}
)

target_link_libraries(genesis
    PRIVATE
        ${RAYLIB_LIBRARY}
)

if (WIN32)
    target_link_libraries(genesis
        PRIVATE
            winmm
            gdi32
            shell32
            opengl32
    )
endif()

if (MSVC)
    target_compile_options(genesis PRIVATE /W4)
else()
    target_compile_options(genesis PRIVATE -Wall -Wextra -Wpedantic)
endif()
