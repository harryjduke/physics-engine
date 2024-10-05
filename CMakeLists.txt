cmake_minimum_required(VERSION 3.28)

project(CI517_GameEngine)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

# Build with shared libraries by default
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# Setup build output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CI517_GameEngine_BINARY_DIR}/build_results/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CI517_GameEngine_BINARY_DIR}/build_results/bin")

# --- FETCH PACKAGES ---
include(FetchContent)
FetchContent_Declare(
        SDL2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-2.30.7
)
FetchContent_Declare(
        SDL2_image
        GIT_REPOSITORY https://github.com/libsdl-org/SDL_image.git
        GIT_TAG release-2.8.2
)
FetchContent_MakeAvailable(SDL2 SDL2_image)

# --- ADD SUBDIRECTORIES ---
add_subdirectory(engine)
add_subdirectory(game)
