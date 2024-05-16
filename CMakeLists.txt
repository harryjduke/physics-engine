cmake_minimum_required(VERSION 3.28)
set(CMAKE_CXX_STANDARD 17)

project(CI517_GameEngine)

# --- SDL2 SETUP ---
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdl2)
set(SDL2_PATH "libs/SDL2/x86_64-w64-mingw32")
set(SDL2_IMAGE_PATH "libs/SDL2_image/x86_64-w64-mingw32")

# Find SDL2 libraries
find_package(SDL2 REQUIRED)
find_package(SDL2_image REQUIRED)

# --- ADD SUBDIRECTORIES ---
add_subdirectory(engine)
add_subdirectory(game)