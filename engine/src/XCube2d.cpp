
//
//  XCube2d.cpp
//  GameEngineBase
//
// all code resources are taken from https://github.com/AlmasB/xcube2d/tree/master
//

#include "../include/XCube2d.h"
#include <iostream>
#include "utils/EngineCommon.h"

using namespace std;

std::shared_ptr<XCube2Engine> XCube2Engine::instance = nullptr;

XCube2Engine::XCube2Engine() 
{
    std::cout << "Initializing X-CUBE 2D v" << _ENGINE_VERSION_MAJOR << "." << _ENGINE_VERSION_MINOR << std::endl;

    #ifdef __DEBUG
        #if defined(_WIN32)
            debug("WIN32");
        #elif defined(__linux__)
            debug("LINUX");
        #elif defined(__APPLE__)
            debug("MACOSX");
        #endif
    #endif

    graphicsEngine = std::shared_ptr<GraphicsEngine>(new GraphicsEngine());

#ifdef __DEBUG
    debug("GraphicsEngine() successful");
#endif

    eventEngine = std::shared_ptr<EventEngine>(new EventEngine());

#ifdef __DEBUG
    debug("EventEngine() successful");
#endif

    physicsEngine = std::shared_ptr<PhysicsEngine>(new PhysicsEngine());

#ifdef __DEBUG
    debug("PhysicsEngine() successful");
#endif


}

XCube2Engine::~XCube2Engine() 
{
#ifdef __DEBUG
    debug("XCube2Engine::~XCube2Engine() started");
#endif

    graphicsEngine.reset();
    eventEngine.reset();
    physicsEngine.reset();

#ifdef __DEBUG
    debug("XCube2Engine::~XCube2Engine() finished");
#endif
}

void XCube2Engine::quit() 
{
    if (instance)
        instance.reset();
}

std::shared_ptr<XCube2Engine> XCube2Engine::getInstance() 
{
    if (!instance)
        instance = std::shared_ptr<XCube2Engine>(new XCube2Engine());
    return instance;
}