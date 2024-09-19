//  This file is part of CI517_GameEngine <https://github.com/harryjduke/CI517_GameEngine>.
//  Copyright (c) 2024 Harry Duke <harryjduke@gmail.com>
//
//  This program is distributed under the terms of the GNU General Public License version 2.
//  You should have received a copy of the GNU General Public License along with this program.
//  If not, see <https://github.com/harryjduke/CI517_GameEngine/blob/main/LICENSE> or <https://www.gnu.org/licenses/>.

#ifndef MY_GAME_H
#define MY_GAME_H

#include <cstdio>
#include "AbstractGame.h"

class MyGame : public AbstractGame
{
private:
    std::shared_ptr<RigidBody> physicsObject1, physicsObject2;
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