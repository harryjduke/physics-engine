#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <vector>
#include <memory>

#include "utils/GameMath.h"

static const float DEFAULT_GRAVITY = 980.f; // cm/s

/**
 * @brief A struct containing the data required for a physics based rigid body.
 * This struct has no functionality beyond that to interact with it's data and is instead designed to be operated on by
 * the PhysicsEngine. RigidBodies must be registered with the PhysicsEngine to be included in the physics simulation.
 */
struct RigidBody
{
    RigidBody(const Vector2F & center, float width, float height, float mass = 1.f, float rotation = 0.f, float restitution = 0.5f);
    RigidBody(const Vector2F & center, float width, float height, bool isStatic, float rotation = 0.f, float restitution = 0.5f);
    RigidBody(const Vector2F& center, const std::vector<Vector2F>& vertices, float mass = 1.f, float restitution = 0.5f);
    RigidBody(const Vector2F& center, const std::vector<Vector2F>& vertices, bool isStatic, float restitution = 0.5f);

    /** The position of the center of this object relative to the origin (top-left) */
    Vector2F center;

    /** A vector of this RigidBody's vertices stored as positions relative to the RigidBody's center */
    std::vector<Vector2F> vertices;

    /** The inverse of this RigidBody's mass */
    float inverseMass;

    /** The restitution (or bounciness) of this RigidBody */
    float restitution;

    /** The current velocity */
    Vector2F velocity;

    /** The current acceleration */
    Vector2F acceleration;

    /**
     * Set the inverseMass of this RigidBody from a (positive, non-zero) mass value
     * @param mass  the mass to set the RigidBody to @attention MUST be greater than zero, any values less than or equal to
     * zero will not be assigned
     */
    void setMass(float mass);

    /**
     * Perform a rotation on this RigidBody by a given angleRadians, rotating all the vertices around the center
     * @param angleRadians  The angleRadians in radians to rotate the RigidBody by
     */
    void rotate(float angleRadians);


    [[nodiscard]] std::vector<Vector2F> getNormals();

    /**
     * Get an axis-aligned bounding box containing this RigidBody, this is not the exact collision but anything that
     * does NOT collide with this bounding box will NOT collide with the RigidBody
     * @return A RectF bounding box containing this RigidBody. Will return a default RectF if there are no vertices to
     * this RigidBody
     */
    [[nodiscard]] RectF getBoundingBox() const;


    /**
     * Gets this RigidBody's vertices relative to the world origin
     * @return A vector of this RigidBody's vertices relative to the world origin
     */
    [[nodiscard]] std::vector<Vector2F> getVerticesWorld() const;

#ifdef __DEBUG
    /**
     * DEBUG ONLY
     * Whether this RigidBody is currently colliding with another RigidBody for testing purposes (to check for
     * collisions between RigidBodies use PhysicsEngine::isCollidingDEBUG)
     */
    bool isCollidingDEBUG;
#endif
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
    void setGravity(Vector2F gravityValue);

    /**
     * Update method that runs the physics simulation to be called every frame from the game. Applies velocity to each
     * RigidBody registered with the PhysicsEngine, applies gravity and handles collisions
     * @param deltaTime  The time (in seconds) since the last frame
     */
    void update(float deltaTime);

    /**
     * Registers the given RigidBody with this PhysicsEngine so that it can be included in the physics simulation
     * @param rigidBody  The RigidBody to register with this PhysicsEngine
     */
    void registerRigidBody(const std::shared_ptr<RigidBody>& rigidBody);

    /**
     * Tests if two RigidBodies are currently colliding and returns the collision vector or zero if there was no
     * collision
     * @param rigidBodyA, rigidBodyB  The RigidBodies to check for collision between
     * @return The collision vector with the direction being the collision normal and the magnitude being the
     * penetration depth. Zero if there was no collision
     */
    static Vector2F getCollision(const std::shared_ptr<RigidBody>& rigidBodyA, const std::shared_ptr<RigidBody>& rigidBodyB);

#ifdef __DEBUG
    /**
     * Set whether the graphics engine should render debug for all RigidBodies
     * @param value The value to set doDebugDEBUG to
     */
    void setDoDebug(bool value) { doDebugDEBUG = value;}

    /**
     * DEBUG ONLY
     * A method, only run in the Debug build and when doDebugDEBUG is true, that draws the collisions of all the
     * RigidBodies registered with this PhysicsEngine
     */
    void drawDebugDEBUG();
#endif

private:

    /** The acceleration due to gravity to apply to registered RigidBodies */
    Vector2F gravity;

    /** A vector of all registered RigidBodies*/
    std::vector<std::shared_ptr<RigidBody>> rigidBodies;

    /**
     * Tests for a collision (with PhysicsEngine::getCollision) between the two given RigidBodies and, if they are
     * colliding, move them apart to negate the overlap and apply a velocity to each based on the restitution of the
     * RigidBodies
     * @param rigidBodyA, rigidBodyB  The RigidBodies to resolve a collision between
     * @return  True if a collision was resolved, false if there was no collision
     */
    static bool resolveCollision(const std::shared_ptr<RigidBody>& rigidBodyA,
                                 const std::shared_ptr<RigidBody>& rigidBodyB);

#ifdef __DEBUG
    /** If true and the build is 'Debug' this PhysicsEngine will draw debug graphics for registered RigidBodies */
    bool doDebugDEBUG;

    /** Polygons (as vector<Vector2F) put in this vector will be rendered on the next frame and removed */
    std::vector<std::vector<Vector2F>> debugQueue;
#endif

};

#endif

