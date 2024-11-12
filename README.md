# Physics Engine

A physics subsystem built on the [XCube2D] game engine, written in C++
using [SDL2]. This subsystem implements both linear and angular rigid body dynamics along with
collision detection and response. 

![physics-engine_demo](https://github.com/user-attachments/assets/a6392518-67f1-4fea-b134-31343d658452)

## Dependencies

- [Git]
- C++ compiler ([g++]/[MSVC])
- [CMake]
- [SDL2]
- [SDL2_image]

## Build Instructions

Follow the instructions for your specific operating system: [Windows](#windows) or [Linux](#linux).
You can also follow the [vcpkg] instructions for [Windows](#windows) on other platforms.

#### Windows

1. **Clone the Repository:**
    ```shell
    git clone https://github.com/harryjduke/physics-engine.git
    cd physics-engine
    ``` 
2. **Install prerequisites**
    - [Install Git](https://git-scm.com/downloads/win) if not already installed.
    - [Install CMake](https://cmake.org/download/) if not already installed
    - Choose A C++ compiler:
        - [MSVC], which comes with Visual Studio
        - [g++], which can be installed through [MinGW]
3. **Install library dependencies:** to install dependencies on Windows, you can either use [vcpkg] to manage
   dependencies automatically (recommended) or manually install them as shown below.

    - **Install with [vcpkg] (recommended):**
        1. Install [vcpkg] by cloning the repository and running the bootstrap script.
            ```shell
            git clone https://github.com/microsoft/vcpkg.git
            .\vcpkg\bootstrap.bat
            ```
        2. When you generate the [CMake] project (see step 4) use
           the`-DCMAKE_TOOLCHAIN_FILE="<path to vcpkg>/scripts/buildsystems/vcpkg.cmake"` argument to use [vcpkg].
            ```shell
            cmake .. -DCMAKE_TOOLCHAIN_FILE=".\vcpkg\scripts\buildsystems\vcpkg.cmake"
            ```
           > **Note:** Alternatively, you can set the VCPKG_ROOT environment variable to the vcpkg directory to
           automatically locate [vcpkg] without manually setting the toolchain file.
    - **Manual installation:**
        1. Download the latest SDL2 and SDL2_image development packages from
           the [SDL2 releases page](https://github.com/libsdl-org/SDL/releases/latest)
           and [SDL2_image releases page](https://github.com/libsdl-org/SDL_image/releases/latest) on GitHub:
            - For MSVC: download `SDL2-devel-<version>-VC.zip` and `SDL2_image-devel-<version>-VC.zip`.
            - For MinGW: download `SDL2-devel-<version>-mingw.zip` and `SDL2_image-devel-<version>-mingw.zip`.
        2. Extract both downloaded archives to a `.\libs` folder in the project (e.g., `<project root>/libs/SDL2`
           and `<project root>/libs/SDL2_image`). CMake will automatically detect packages here.
           > **Note**: If you prefer to install the libraries in a different location, use
           the `-DCMAKE_PREFIX_PATH="<path to SDL2>"` argument when generating the project (see step 4).
           >```shell
            >cmake .. -DCMAKE_PREFIX_PATH="C:\libs"
            >```
4. Generate the [CMake] project by creating a `.\build` directory in the project root and running the cmake command from
   within the folder, using `..` to use the parent directory as the project source directory.
   Pass any arguments (e.g., `-DCMAKE_TOOLCHAIN_FILE` or `-DCMAKE_PREFIX_PATH`) here.
   > **Note**: If you are using a single-config generator (not [MSVC]) you will need to pass `-DCMAKE_BUILD_TYPE=Debug`
   as only the 'Debug' version displays any visuals currently.
   >```shell
    >cmake .. -DCMAKE_BUILD_TYPE=Debug
    >```

    ```shell
     mkdir build
     cd build
     cmake ..
     ```
5. Build the project by running `cmake --build .` in the `.\build` directory.
   If you generated the project with [MSVC] you can instead open the solution in Visual Studio and build it from there.
   > **Note**: If you are using a single-config generator (not [MSVC]) you do not need to pass `--config Debug` and the
   executable will be at `.\bin\MyGame.exe`.

    ```shell
    #build
    cmake --build . --config Debug
   
    #run
    .\bin\Debug\MyGame.exe
    ```

#### Linux

1. On Debian/Ubuntu-based distributions, use the following command to install dependencies:
    ```shell
    sudo apt install git build-essential cmake libsdl2-dev libsdl2-image-dev
    ```

2. Clone and build the project:
    ```shell
    # clone this repository
    git clone https://github.com/harryjduke/physics-engine.git
    cd physics-engine

    # build
    mkdir build
    cd build
    cmake ..
    cmake --build .

    # run
    ./bin/MyGame.exe
    ```

## Usage

- The game is created by extending the `AbstractGame` class. `AbstractGame::runMainLoop()` starts the game.
- To create a `RigidBody` pass a shared pointer to the `RigidBody` to the `PhysicsEngine::registerRigidBody()` method.
  A pointer to the game's `PhysicsEngine` is stored in `AbstractGame::physicsEngine`.
```cpp
const auto rigidBody = std::make_shared<RigidBody>(Vector2F{1000, 400}, 50, 50);
physicsEngine->registerRigidBody(rigidBody);
```

## Licence

Distributed under the [GPLv2](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html) license.
See [LICENCE.txt](LICENSE) for more information.

## Credits

- **[XCube2D](https://github.com/AlmasB/xcube2d)** - The base game engine for this subsystem.
- **Collision Detection**: 
  - [Collision Detection Using the Separating Axis Theorem - Kah Shiu Chong](https://code.tutsplus.com/collision-detection-using-the-separating-axis-theorem--gamedev-169t)
  - [Linear Collision Resolution in 2D Game Physics - pikuma](https://www.youtube.com/watch?v=1L2g4ZqmFLQ)
- **Physics Calculations**: 
  - [Physics articles (Parts 1, 2 & 3) - Chris Hecker](https://www.chrishecker.com/Rigid_Body_Dynamics)
- **Moment of Inertia**: 
  - [Moments of Inertia for Triangles and other Polygons - Robert Fotino](https://fotino.me/moment-of-inertia-algorithm/)
  - [Calculating the moment of inertia of a triangle - Matt Anderson](https://www.youtube.com/watch?v=yEa8npNVejg)

[XCube2D]: https://github.com/AlmasB/xcube2d

[SDL2]: https://github.com/libsdl-org/SDL

[SDL2_image]: https://github.com/libsdl-org/SDL_image

[Git]: https://git-scm.com/

[g++]: https://gcc.gnu.org/

[MSVC]: https://visualstudio.microsoft.com/vs/features/cplusplus/

[CMake]: https://cmake.org/

[vcpkg]: https://github.com/microsoft/vcpkg

[MinGW]: https://www.mingw-w64.org/
