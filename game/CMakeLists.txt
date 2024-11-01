# Create the 'MyGame' executable target
add_executable(${GAME_NAME}
        src/main.cpp
        src/MyGame.cpp
        src/MyGame.h)

# Specify the include directories for the 'MyGame' target
target_include_directories(${GAME_NAME} PRIVATE include)

# Link the 'Engine' library to the 'MyGame' target
target_link_libraries(${GAME_NAME} Engine)