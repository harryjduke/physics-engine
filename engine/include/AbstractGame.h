//
//  AbstractGame.h
//  GameEngineBase
//
// all code resources are taken from https://github.com/AlmasB/xcube2d/tree/master

#ifndef ABSTRACT_GAME_H
#define ABSTRACT_GAME_H

#include "XCube2d.h"

#include <cstdio>

class AbstractGame {
private:
    void handleMouseEvents();
    void updatePhysics();

protected:
    AbstractGame();
    virtual ~AbstractGame();

    /* Engine systems */
    std::shared_ptr<GraphicsEngine> graphicsEngine;
    std::shared_ptr<EventEngine> eventEngine;
    std::shared_ptr<PhysicsEngine> physicsEngine;

    /* Main loop control */
    bool running;
    bool paused;
    double gameTime;

    virtual void update() = 0;
    virtual void render() = 0;

    //Define input handlers
    virtual void handleKeyEvents() = 0;
    virtual void onLeftMouseButton();
    virtual void onRightMouseButton();


public:
    int runMainLoop();
};

#endif