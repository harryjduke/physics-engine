// This file is part of physics-engine <https://github.com/harryjduke/physics-engine>.
// Copyright (c) 2024 Harry Duke <harryjduke@gmail.com>
// This file includes modifications made by Harry Duke.
//
// This program is distributed under the terms of the GNU General Public License version 2.
// You should have received a copy of the GNU General Public License along with this program.
// If not, see <https://github.com/harryjduke/physics-engine/blob/main/LICENSE> or <https://www.gnu.org/licenses/>.

#ifndef MY_GAME_H
#define MY_GAME_H

#include <cstdio>

#include "AbstractGame.h"

class MyGame final : public AbstractGame
{
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
    ~MyGame() override;

};

#endif