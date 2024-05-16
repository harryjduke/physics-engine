#include <iostream>
#include "MyGame.h"

int main (int argc, char* args[])
{
    try
    {
        MyGame game;
        game.runMainLoop();
    }
    catch (std::exception &e)
    {
        std::cout << "error: " << std::endl;
    }
    return 0;
}

