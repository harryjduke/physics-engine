# Create a library target named 'Engine' from the source files
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

# Link the SDL2 libraries to the 'Engine' target
target_link_libraries(Engine SDL2::Main SDL2::Image)

# Define __DEBUG if we are in the Debug build
if(CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_definitions(Engine PUBLIC __DEBUG)
endif()