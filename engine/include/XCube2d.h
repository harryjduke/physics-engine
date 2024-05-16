//
//  XCube2d.h
//  GameEngineBase
//
// all code resources are taken from https://github.com/AlmasB/xcube2d/tree/master
//

#ifndef X_CUBE_2D_H
#define X_CUBE_2D_H

#include <cstdio>
#include <string>
#include <vector>
#include <memory>

#include "GraphicsEngine.h"
#include "EventEngine.h"
#include "PhysicsEngine.h"

const int _ENGINE_VERSION_MAJOR = 0;
const int _ENGINE_VERSION_MINOR = 1;

class XCube2Engine {
private:
    static std::shared_ptr<XCube2Engine> instance;

    // Initialize subsystems
    std::shared_ptr<GraphicsEngine> graphicsEngine;
    std::shared_ptr<EventEngine> eventEngine;
    std::shared_ptr<PhysicsEngine> physicsEngine;

    XCube2Engine();
public:
    /**
    * @return the instance of game engine
    * @exception throws EngineException if init of any submodules failed
    */
    static std::shared_ptr<XCube2Engine> getInstance();
    ~XCube2Engine();

    /** @return The graphics engine subsystem instance */
    std::shared_ptr<GraphicsEngine> getGraphicsEngine() {return graphicsEngine;}
    /** @return The event engine subsystem instance */
    std::shared_ptr<EventEngine> getEventEngine() {return eventEngine;}
    /** @return The physics engine subsystem instance */
    std::shared_ptr<PhysicsEngine> getPhysicsEngine() {return physicsEngine;}

    /**
    * Quits the engine, closes all the subsystems
    *
    * All subsequent calls to any of subsystems will have undefined behaviour
    */
    static void quit();


};

typedef XCube2Engine XEngine;

#endif