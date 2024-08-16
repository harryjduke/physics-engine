/*
 * @file MyGame.cpp
 * @author Harry
 * @date 16/05/2024
 * @copyright (c) 2024 Harry Duke
 */

#include "MyGame.h"

#include <memory>

#include "utils/GameMath.h"

MyGame::MyGame() : AbstractGame(),
    physicsObject1(std::make_shared<RigidBody>(Vector2F{1000, 400}, 2000, 60, true)),
    physicsObject2(std::make_shared<RigidBody>(Vector2F{100, 100}, 200, 60, true, 1)),
    objectSpawnCooldown(.2),
    lastSpawnedObjectTime{}
{
    // Setup graphics engine
    graphicsEngine->setVerticalSync(true);

    // Setup physics engine
#ifdef __DEBUG
    physicsEngine->setDoDebug(true);
#endif

    physicsObject1->rotate(PI);
    physicsEngine->registerRigidBody(physicsObject1);
    physicsEngine->registerRigidBody(physicsObject2);
}
MyGame::~MyGame() = default;

void MyGame::handleKeyEvents()
{
    float speed = 3.f;

    if (eventEngine->isPressed(BTN_LEFT) && gameTime - lastSpawnedObjectTime > objectSpawnCooldown)
    {
        lastSpawnedObjectTime = gameTime;
        physicsEngine->registerRigidBody(std::make_shared<RigidBody>(static_cast<Vector2F>(
                eventEngine->getMousePos()), 50, 50, 10.f, getRandom(0, 2 * PI)));
    }

}

void MyGame::update(float deltaTime)
{

}

void MyGame::render()
{
#ifdef __DEBUG
    physicsEngine->drawDebugDEBUG();
#endif
}

void MyGame::renderUI()
{

}
