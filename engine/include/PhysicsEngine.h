#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <vector>
#include <memory>

#include "utils/GameMath.h"

static const float DEFAULT_GRAVITY = 9.8f;

struct PhysicsObject
{
    Point2 center;
    float width;
    float height;
    float inverseMass;
    Vector2f velocity;
    Vector2f acceleration;
    void setMass(float mass);

    PhysicsObject(const Point2 & center, float w, float h, float mass = 1.f);
    PhysicsObject(const Point2 & center, float w, float h, bool isStatic);
};

class PhysicsEngine
{
    friend class XCube2Engine;

private:
    Vector2f gravity;
    bool doDebug{};

    PhysicsEngine();
    std::vector<std::shared_ptr<PhysicsObject>> objects;

public:

    /**
     * Set the acceleration due to gravity
     * Note that gravity is naturally a positive value as the origin for SDL is top-left.
     * @param gravityValue the acceleration due to gravity on the vertical axis
     */
    void setGravity(float gravityValue);
    /**
     * Set the acceleration due to gravity.
     * Note that gravity is naturally a positive value as the origin for SDL is top-left.
     * @param gravityValue The acceleration due to gravity as a 2D vector.
     */
    void setGravity(Vector2f gravityValue);

    /**
     * Set whether the graphics engine should render debug for all PhysicsObjects
     * @param value The value to set doDebug to
     */
    void setDoDebug(bool value) {doDebug = value;}

    void update(float deltaTime);

    void registerObject(const std::shared_ptr<PhysicsObject>&);

#ifdef __DEBUG
    void drawDebug();
#endif
};

#endif

