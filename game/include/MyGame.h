/*
 * @file MyGame.h
 * @author Harry
 * @date 16/05/2024
 * @copyright (c) 2024 Harry Duke
 */

#ifndef MY_GAME_H
#define MY_GAME_H

#include <cstdio>
#include "AbstractGame.h"

class MyGame : public AbstractGame
{
private:
    std::shared_ptr<PhysicsObject> physicsObject1, physicsObject2;
    double objectSpawnCooldown;
    double lastSpawnedObjectTime;


    /* GAMEPLAY */
    void handleKeyEvents() override;
    void update(float deltaTime) override;
    void render() override;
    void renderUI();
public:
    MyGame();
    ~MyGame();

};

#endif