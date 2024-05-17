/*
 * @file MyGame.cpp
 * @author Harry
 * @date 16/05/2024
 * @copyright (c) 2024 Harry Duke
 */

#include "MyGame.h"

#include <memory>

MyGame::MyGame() : AbstractGame(),
    physicsObject1(std::make_shared<PhysicsObject>(Point2{45, 45}, 50, 50)),
    physicsObject2(std::make_shared<PhysicsObject>(Point2{1000, 400}, 2000, 60, true))
{
    graphicsEngine->setVerticalSync(true);

    physicsEngine->setGravity(98.f);
    physicsEngine->setDoDebug(true);
    physicsEngine->registerObject(physicsObject1);
    //physicsEngine->registerObject(physicsObject2);
}
MyGame::~MyGame() = default;

void MyGame::handleKeyEvents()
{
    float speed = 3.f;

    if (eventEngine->isPressed(BTN_LEFT))
    {
        physicsEngine->registerObject(std::make_shared<PhysicsObject>(eventEngine->getMousePos(), 50, 50));
    }

}

void MyGame::update(float deltaTime)
{

}

void MyGame::render()
{
#ifdef __DEBUG
    physicsEngine->drawDebug();
#endif
}

void MyGame::renderUI()
{

}
