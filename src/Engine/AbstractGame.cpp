
//
//  AbstractGame.cpp
//  GameEngineBase
// all code resources are taken from https://github.com/AlmasB/xcube2d/tree/master
//

#include "AbstractGame.h"
#include <iostream>
using namespace std;

AbstractGame::AbstractGame() : running(false), paused(false), gameTime(0.0)
{
    cout << "Abstract Game started" << endl;
}

AbstractGame::~AbstractGame()
{
    cout << "Abstract Game ended" << endl;
}

int AbstractGame::runMainLoop() {
#ifdef __DEBUG
    debug("Entered Main Loop");
#endif

    while (running)
    {
        std::cout << "main loop";
    }

#ifdef __DEBUG
    debug("Exited Main Loop");
#endif

    return 0;
}
