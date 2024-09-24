<<<<<<< HEAD
set(CMAKE_VERBOSE_MAKEFILE ON)
cmake_minimum_required(VERSION 3.6)

# the project name is MyGame, rename as needed
project(MyGame CXX)

set(CMAKE_CXX_STANDARD 11)

if(WIN32)
    # use bundled version to save ourselves a lot of trouble
    set(SDL2MAIN_LIBRARY "${CMAKE_SOURCE_DIR}/libs/SDL2/lib/x64/SDL2main.lib")

    set(SDL2_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/libs/SDL2/include")
    set(SDL2_LIBRARY "${CMAKE_SOURCE_DIR}/libs/SDL2/lib/x64/SDL2.lib")

    set(SDL2_IMAGE_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/libs/SDL2_image/include")
    set(SDL2_IMAGE_LIBRARIES "${CMAKE_SOURCE_DIR}/libs/SDL2_image/lib/x64/SDL2_image.lib")
    
    set(SDL2_MIXER_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/libs/SDL2_mixer/include")
    set(SDL2_MIXER_LIBRARIES "${CMAKE_SOURCE_DIR}/libs/SDL2_mixer/lib/x64/SDL2_mixer.lib")
    
    set(SDL2_TTF_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/libs/SDL2_ttf/include")
    set(SDL2_TTF_LIBRARIES "${CMAKE_SOURCE_DIR}/libs/SDL2_ttf/lib/x64/SDL2_ttf.lib")
    
    # copy dynamic libraries
    file(COPY 
            "${CMAKE_SOURCE_DIR}/libs/SDL2/lib/x64/SDL2.dll"
            "${CMAKE_SOURCE_DIR}/libs/SDL2_image/lib/x64/libjpeg-9.dll"
            "${CMAKE_SOURCE_DIR}/libs/SDL2_image/lib/x64/libpng16-16.dll"
            "${CMAKE_SOURCE_DIR}/libs/SDL2_image/lib/x64/libtiff-5.dll"
            "${CMAKE_SOURCE_DIR}/libs/SDL2_image/lib/x64/libwebp-7.dll"
            "${CMAKE_SOURCE_DIR}/libs/SDL2_image/lib/x64/SDL2_image.dll"
            "${CMAKE_SOURCE_DIR}/libs/SDL2_image/lib/x64/zlib1.dll"
            "${CMAKE_SOURCE_DIR}/libs/SDL2_mixer/lib/x64/SDL2_mixer.dll"
            "${CMAKE_SOURCE_DIR}/libs/SDL2_mixer/lib/x64/libmpg123-0.dll"
            "${CMAKE_SOURCE_DIR}/libs/SDL2_ttf/lib/x64/libfreetype-6.dll"
            "${CMAKE_SOURCE_DIR}/libs/SDL2_ttf/lib/x64/SDL2_ttf.dll"
            DESTINATION ${CMAKE_BINARY_DIR})
else()
    # let FindSDL2.cmake figure out where it is on Mac/Linux
    # Put the FindSDL2.cmake in ./cmake and tell CMake to find it there
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

    set(SDL2MAIN_LIBRARY "")

    find_package(SDL2 REQUIRED)
    find_package(SDL2_image REQUIRED)
    # find_package(SDL2_ttf REQUIRED)
endif()

# include SDL header files
include_directories(${SDL2_INCLUDE_DIR}
                    ${SDL2_IMAGE_INCLUDE_DIR}
                    ${SDL2_MIXER_INCLUDE_DIR}
                    ${SDL2_TTF_INCLUDE_DIR})

# load user source and header files
file(GLOB_RECURSE SOURCE_FILES "src/*.h" "src/*.cpp")
add_executable(${PROJECT_NAME} WIN32 ${SOURCE_FILES})

# make assets directory in build
#file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/assets)

# copy game assets into build/assets directory
#file(COPY
#        "assets/assetName.png"
#        DESTINATION ${CMAKE_BINARY_DIR}/assets)

# SDL2MAIN_LIBRARY is needed for Windows specific main function.
target_link_libraries(${PROJECT_NAME}
        ${SDL2MAIN_LIBRARY}
        ${SDL2_LIBRARY}
        ${SDL2_IMAGE_LIBRARIES}
        ${SDL2_MIXER_LIBRARIES}
        ${SDL2_TTF_LIBRARIES})
=======
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
>>>>>>> 7340bce (Base engine setup)
