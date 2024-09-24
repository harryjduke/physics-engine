# Create the 'Engine' library target
add_library(Engine
        src/XCube2d.cpp
        include/XCube2d.h
        src/AbstractGame.cpp
        include/AbstractGame.h
        include/utils/EngineCommon.h
        include/utils/GameMath.h
        src/EventEngine.cpp
        include/EventEngine.h
        src/GraphicsEngine.cpp
        include/GraphicsEngine.h
        src/PhysicsEngine.cpp
        include/PhysicsEngine.h
)

# Specify the include directories for the 'Engine' target
target_include_directories(Engine PUBLIC include)

# Link external libraries to the 'Engine' target
target_link_libraries(Engine
        PUBLIC
        SDL2::SDL2main
        SDL2::SDL2
        SDL2_image::SDL2_image
)

# Define __DEBUG macro for Debug builds
if(CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_definitions(Engine PUBLIC __DEBUG)
endif()