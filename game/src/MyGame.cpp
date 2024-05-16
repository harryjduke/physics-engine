
//
//  MyGame.cpp
//  GameEngine
//
// all code resources are taken from https://github.com/AlmasB/xcube2d/tree/master
//


#include "MyGame.h"

MyGame::MyGame() : AbstractGame(), numKeys(5), box{20, 20, 50, 50}, box2{0, 370, 2000, 60},
    physicsObject1({45, 45}, 50, 50), physicsObject2({1000, 400}, 2000, 60), score(0), lives(3), gameWon(false)
{
    graphicsEngine->setVerticalSync(true);

    physicsEngine->setGravity(2, 2);

    // Draw points on random locations
    for (int i = 0; i < numKeys; i++)
    {
        std::shared_ptr<GameKey> k = std::make_shared<GameKey>();
        k->isAlive = true;
        k->pos = Point2(getRandom(0, 750), getRandom(0, 550));
        gameKeys.push_back(k);
    }
}
MyGame::~MyGame() {

}

void MyGame::handleKeyEvents()
{
    float speed = 3.f;

    if (eventEngine->isPressed(Key::W))
    {
        physicsObject1.applyForceVertical(15);
    }

    if (eventEngine->isPressed(Key::S))
    {
        physicsObject1.applyForceVertical(-speed);
    }

    if (eventEngine->isPressed(Key::A))
    {
        physicsObject1.applyForceHorizontal(speed);
    }

    if (eventEngine->isPressed(Key::D))
    {
        physicsObject1.applyForceHorizontal(-speed);
    }

}

void MyGame::update()
{



    if (!physicsObject1.isColliding(physicsObject2))
    {
        physicsObject1.applyGravity(physicsEngine);
    }

    for (const auto& key : gameKeys)
    {

        if (key->isAlive &&
            box.x <= key->pos.x &&
            box.x + box.w >= key->pos.x &&
            box.y <= key->pos.y &&
            box.y + box.h >= key->pos.y)
        {
            score += 200;
            key->isAlive = false;
            numKeys--;
        }
    }

    if (numKeys == 0)
    {
        gameWon = true;
    }

}

void MyGame::render()
{
    graphicsEngine->setDrawColor(SDL_COLOR_RED);
    graphicsEngine->drawRect(box);
    graphicsEngine->setDrawColor(SDL_COLOR_GREEN);
    graphicsEngine->fillRect(box2.x, box2.y, box2.w, box2.h);
    graphicsEngine->setDrawColor(SDL_COLOR_YELLOW);
    for (const auto& key : gameKeys)
    {
        if(key->isAlive)
        {
            graphicsEngine->drawCircle(key->pos, 5);
        }
    }
}

void MyGame::renderUI() {
    graphicsEngine->setDrawColor(SDL_COLOR_AQUA);
    std::string scoreStr = std::to_string(score);


    if (gameWon)
        std::cout << "you won: " << score;
}
