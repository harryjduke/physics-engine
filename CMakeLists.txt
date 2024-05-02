cmake_minimum_required(VERSION 3.28)
project(CI517_GameEngine)

set(CMAKE_CXX_STANDARD 17)

add_executable(CI517_GameEngine src/main.cpp)

# --- SDL2 SETUP ---
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdl2)
set(SDL2_PATH "libs/SDL2/x86_64-w64-mingw32")

# Find SDL2 libraries
find_package(SDL2 REQUIRED)

# Link SDL2::Main to our project
target_link_libraries(${PROJECT_NAME} SDL2::Main)