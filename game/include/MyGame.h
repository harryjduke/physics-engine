#pragma once
//
//  MyGame.h
//  GameEngine
//
// all code resources are taken from https://github.com/AlmasB/xcube2d/tree/master
//


#include <cstdio>
#include "AbstractGame.h"

struct GameKey {
    Point2 pos;
    bool isAlive{};
};

class MyGame : public AbstractGame
{
private:
    SDL_Rect box, box2;
    PhysicsObject physicsObject1, physicsObject2;

    std::vector<std::shared_ptr<GameKey>> gameKeys;

    /* GAMEPLAY */
    int score, numKeys, lives;
    bool gameWon;

    void handleKeyEvents() override;
    void update() override;
    void render() override;
    void renderUI();
public:
    MyGame();
    ~MyGame();
};

