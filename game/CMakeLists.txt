# Create the 'MyGame' executable target
add_executable(MyGame
        src/main.cpp
        src/MyGame.cpp
        include/MyGame.h)

# Specify the include directories for the 'MyGame' target
target_include_directories(MyGame PRIVATE include)

# Link the 'Engine' library to the 'MyGame' target
target_link_libraries(MyGame Engine)