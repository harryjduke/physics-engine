# --- SETUP ---
cmake_minimum_required(VERSION 3.21)

# Build with shared libraries by default
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# Setup vcpkg
if (NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    if (DEFINED ENV{VCPKG_ROOT})
        cmake_path(SET CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
        message(STATUS "Using toolchain file from VCPKG_ROOT: ${VCPKG_ROOT}")
    endif ()
else ()
    message(STATUS "Using provided CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")
endif ()

# Setup project
project(physics-engine)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

# Setup game
set(GAME_NAME "MyGame")

# Setup build output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${physics-engine_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${physics-engine_BINARY_DIR}/bin")

# --- DEPENDENCIES ---
if (EXISTS "${CMAKE_SOURCE_DIR}/libs")
    set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/libs")
    message(STATUS "Local /libs folder will be checked for dependencies.")
endif ()

find_package(SDL2 CONFIG REQUIRED)
find_package(SDL2_image CONFIG REQUIRED)

# --- SUBDIRECTORIES ---
add_subdirectory(engine)
add_subdirectory(game)

# Set startup project (for visual studio solution generation)  ---
if (CMAKE_GENERATOR MATCHES "Visual Studio")
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${GAME_NAME})
endif ()