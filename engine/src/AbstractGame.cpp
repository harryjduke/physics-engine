
//
//  AbstractGame.cpp
//  GameEngineBase
// all code resources are taken from https://github.com/AlmasB/xcube2d/tree/master
//

#include "AbstractGame.h"

#include <iostream>

#include "utils/EngineCommon.h"

using namespace std;

AbstractGame::AbstractGame() : running(true), paused(false), gameTime(0.0)
{
    std::shared_ptr<XCube2Engine> engine = XCube2Engine::getInstance();

    graphicsEngine = engine->getGraphicsEngine();
    eventEngine = engine->getEventEngine();
    physicsEngine = engine->getPhysicsEngine();
}

AbstractGame::~AbstractGame()
{
#ifdef __DEBUG
    debug("AbstractGame::~AbstractGame() started");
#endif

    // kill Game class' instance pointers
    // so that engine is isolated from the outside world
    // before shutting down
    graphicsEngine.reset();
    eventEngine.reset();
    physicsEngine.reset();

    // kill engine
    XCube2Engine::quit();

#ifdef __DEBUG
    debug("AbstractGame::~AbstractGame() finished");
    debug("The game finished and cleaned up successfully. Press Enter to exit");
    getchar();
#endif
}

int AbstractGame::runMainLoop() {
#ifdef __DEBUG
    debug("Entered Main Loop");
#endif

    while (running)
    {
        graphicsEngine->setFrameStart();
        eventEngine->pollEvents();

        if (eventEngine->isPressed(Key::ESC) || eventEngine->isPressed(Key::QUIT))
            running = false;

        handleKeyEvents();
        handleMouseEvents();

        if (!paused) {
            update();
            updatePhysics();
            gameTime += 0.016;	// 60 times a sec
        }

        graphicsEngine->clearScreen();
        render();
        graphicsEngine->showScreen();

        graphicsEngine->adjustFPSDelay(16);	// atm hardcoded to ~60 FPS
    }

#ifdef __DEBUG
    debug("Exited Main Loop");
#endif

    return 0;
}

void AbstractGame::handleMouseEvents() {
    if (eventEngine->isPressed(Mouse::BTN_LEFT)) onLeftMouseButton();
    if (eventEngine->isPressed(Mouse::BTN_RIGHT)) onRightMouseButton();
}

void AbstractGame::onLeftMouseButton() {

}

void AbstractGame::onRightMouseButton() {

}

void AbstractGame::updatePhysics() {
    physicsEngine->update();
}
