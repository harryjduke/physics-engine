#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <vector>
#include <memory>

#include "utils/GameMath.h"

static const float DEFAULT_GRAVITY = 980.f; // cm/s

/**
 * @brief A struct containing the data required for a physics based object
 * This object has no functionality and is instead designed to be operated on by the PhysicsEngine. PhysicsObjects must
 * be registered with the PhysicsEngine to be included in the physics simulation
 */
struct PhysicsObject
{
    PhysicsObject(const Point2f & center, float w, float h, float mass = 1.f);
    PhysicsObject(const Point2f & center, float w, float h, bool isStatic);

    /** The position of the center of this object relative to the origin (top-left) */
    Point2f center;

    /** The width of this object */
    float width;

    /** The height of this object */
    float height;

    /** The inverse of this objects mass */
    float inverseMass;

    /** The current velocity */
    Vector2f velocity;

    /** The current acceleration */
    Vector2f acceleration;

    /**
     * Whether this object is currently colliding with another object for testing purposes (to check for collisions
     * between objects use PhysicsEngine::isColliding)
     */
    bool isColliding;

    /**
     * Set the inverseMass of this object from a (positive, non-zero) mass value
     * @param mass  the mass to set the object to @attention MUST be greater than zero, any values less than or equal to
     * zero will not be assigned
     */
    void setMass(float mass);

    /**
     * @return A Rectf with the x and y set to the upper left corner of the object and the width and height
     * set to it's width and height
     */
    [[nodiscard]] Rectf getRect() const;
};

class PhysicsEngine
{
    friend class XCube2Engine;
public:
    PhysicsEngine();

    /**
     * Set the acceleration due to gravity
     * Note that gravity is naturally a positive value as the origin for SDL is top-left.
     * @param gravityValue  the acceleration due to gravity on the vertical axis
     */
    void setGravity(float gravityValue);
    /**
     * Set the acceleration due to gravity.
     * Note that gravity is naturally a positive value as the origin for SDL is top-left.
     * @param gravityValue  The acceleration due to gravity as a 2D vector.
     */
    void setGravity(Vector2f gravityValue);

    /**
     * Set whether the graphics engine should render debug for all PhysicsObjects
     * @param value The value to set doDebug to
     */
    void setDoDebug(bool value) {doDebug = value;}

    /**
     * Update method that runs the physics simulation to be called every frame from the game. Applies velocity to each
     * PhysicsObject registered with the PhysicsEngine, applies gravity and handles collisions
     * @param deltaTime  The time (in seconds) since the last frame
     */
    void update(float deltaTime);

    /**
     * Registers the given PhysicsObject with this PhysicsEngine so that it can be included in the physics simulation
     * @param object  The object to register with this PhysicsEngine
     */
    void registerObject(const std::shared_ptr<PhysicsObject>& object);

    /**
     * Tests if two objects are currently colliding
     * @param object1, object2  The objects to check for collision between
     * @return True if the objects are colliding
     */
    static bool isColliding(const std::shared_ptr<PhysicsObject>& object1, const std::shared_ptr<PhysicsObject>& object2);

#ifdef __DEBUG
    /**
     * A method, only run in the Debug build and when doDebug is true, that draws the collisions of all the
     * PhysicsObjects registered with this PhysicsEngine
     */
    void drawDebug();
#endif

private:

    /** The acceleration due to gravity to apply to registered PhysicsObjects */
    Vector2f gravity;

    /** If true and the build is 'Debug' this PhysicsEngine will draw debug graphics for registered PhysicsObjects */
    bool doDebug;

    /** A vector of all registered PhysicsObjects*/
    std::vector<std::shared_ptr<PhysicsObject>> objects;

    void resolveCollision(std::shared_ptr<PhysicsObject> object1, std::shared_ptr<PhysicsObject> object2);
};

#endif

