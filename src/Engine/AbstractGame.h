//
//  AbstractGame.h
//  GameEngineBase
//
// all code resources are taken from https://github.com/AlmasB/xcube2d/tree/master


#include <stdio.h>
#include "XCube2d.h"


class AbstractGame {
private:
    /* Main loop control */
    bool running;
    bool paused;
    double gameTime;
protected:
    AbstractGame();
    virtual ~AbstractGame();

public:
    int runMainLoop();
};

