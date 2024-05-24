#include "PhysicsEngine.h"

#include <algorithm>
#include <cmath>

#include "XCube2d.h"

/* PHYSICS OBJECT */

RigidBody::RigidBody(const Vector2F & centerOfMass, float width, float height, float mass, float rotation, float restitution) :
        centerOfMass(centerOfMass),
        collisionMesh{
                {-width / 2, -height / 2},
                {width / 2,  -height / 2},
                {width / 2,  height / 2},
                {-width / 2, height / 2}},
        inverseMass(mass != 0 ? 1.f/mass : 1.f),
        restitution(restitution),
        velocity{},
        acceleration{},
        angularVelocity{},
        angularAcceleration{}
#ifdef __DEBUG
        , isCollidingDEBUG{}
#endif
{
    momentOfInertia = PhysicsEngine::calculateMomentOfInertia(collisionMesh, 1/inverseMass);
    rotate(rotation);
}

RigidBody::RigidBody(const Vector2F & centerOfMass, float width, float height, bool isStatic, float rotation, float restitution) :
        centerOfMass(centerOfMass),
        collisionMesh{
                {-width / 2, -height / 2},
                {width / 2,  -height / 2},
                {width / 2,  height / 2},
                {-width / 2, height / 2}},
        inverseMass(isStatic ? 0.f : 1.f ),
        restitution(restitution),
        velocity{},
        acceleration{},
        angularVelocity{},
        angularAcceleration{}
#ifdef __DEBUG
        , isCollidingDEBUG{}
#endif
{
    momentOfInertia = PhysicsEngine::calculateMomentOfInertia(collisionMesh, 1/inverseMass);
    rotate(rotation);
}

RigidBody::RigidBody(const Vector2F& centerOfMass, const std::vector<Vector2F>& collisionMesh, float mass, float restitution) :
        centerOfMass(centerOfMass),
        collisionMesh(collisionMesh),
        inverseMass(mass > 0 ? 1.f/mass : 1.f),
        restitution(restitution),
        velocity{},
        acceleration{},
        angularVelocity{},
        angularAcceleration{}
#ifdef __DEBUG
        , isCollidingDEBUG{}
#endif
{
    momentOfInertia = PhysicsEngine::calculateMomentOfInertia(collisionMesh, 1/inverseMass);
}

RigidBody::RigidBody(const Vector2F& centerOfMass, const std::vector<Vector2F>& collisionMesh, bool isStatic, float restitution) :
        centerOfMass(centerOfMass),
        collisionMesh(collisionMesh),
        inverseMass(isStatic ? 0.f : 1.f ),
        restitution(restitution),
        velocity{},
        acceleration{},
        angularVelocity{},
        angularAcceleration{}
#ifdef __DEBUG
        , isCollidingDEBUG{}
#endif
{
    momentOfInertia = PhysicsEngine::calculateMomentOfInertia(collisionMesh, 1/inverseMass);
}

void RigidBody::setMass(float mass) { if (mass > 0) inverseMass = 1 / mass; }

void RigidBody::rotate(float angleRadians) {
    // Calculate sine and cosine of the angleRadians
    float sinAngle = std::sin(angleRadians);
    float cosAngle = std::cos(angleRadians);

    for (auto& vertex : collisionMesh){
        vertex = { // Rotate the vertex
            vertex.x * cosAngle - vertex.y * sinAngle,
            vertex.x * sinAngle + vertex.y * cosAngle
        };
    }
}

std::vector<Vector2F> RigidBody::getNormals() {
    std::vector<Vector2F> normals{};
    for (auto it = collisionMesh.begin(); it != collisionMesh.end(); ++it)
    {
        Line2f edge;
        if (std::next(it) != collisionMesh.end()) edge = {*it, *next(it)};
        else edge = {*it, *collisionMesh.begin()};

        normals.push_back(edge.getNormal());
    }
    // OPTIMISE: cache the normals in a private member
    return normals;
}

RectF RigidBody::getBoundingBox() const {
    if (collisionMesh.empty()) return RectF{};

    RectF boundingBox{collisionMesh[0].x, collisionMesh[0].y, 0.0f, 0.0f }; // Initialize width and height to 0

    for (auto it = std::next(collisionMesh.begin()); it != collisionMesh.end(); it++) {
        boundingBox.x = std::min(boundingBox.x, it->x);
        boundingBox.y = std::min(boundingBox.y, it->y);
    }

    for (auto vertex : collisionMesh) {
        boundingBox.w = std::max(boundingBox.w, vertex.x - boundingBox.x); // Update width based on x
        boundingBox.h = std::max(boundingBox.h, vertex.y - boundingBox.y); // Update height based on y
    }

    return boundingBox + centerOfMass;
}

std::vector<Vector2F> RigidBody::getCollisionMeshWorld() const {
    std::vector<Vector2F> collisionMeshWorld = collisionMesh;

    for (auto& vertex : collisionMeshWorld)
    {
        vertex += centerOfMass;
    }
    return collisionMeshWorld;
}

/* PHYSICS ENGINE */

PhysicsEngine::PhysicsEngine() :
        gravity(Vector2F(0, DEFAULT_GRAVITY))
#ifdef __DEBUG
        ,doDebugDEBUG{}
#endif
{ }

void PhysicsEngine::update(float deltaTime)
{
    for (const auto& rigidBody : rigidBodies)
    {
#ifdef __DEBUG
        rigidBody->isCollidingDEBUG = false;
#endif
        // Calculate forces
        Vector2F force{};
        float torque{};

        // Apply acceleration (a = f/m)
        rigidBody->acceleration = force * rigidBody->inverseMass;
        // Apply angular acceleration (α = t/I)
        rigidBody->angularAcceleration = torque / rigidBody->momentOfInertia;

        // Add gravity (as gravity is a constant acceleration, we don't need to calculate the force)
        if (rigidBody->inverseMass != 0) // If the rigidBody is not static
            rigidBody->acceleration += gravity;

        // Update velocity and angular velocity using Euler integration.
        // v(n+1) = v(n) + h * a(n) where n is the previous frame, n+1 is this frame and h is delta time
        // This is an approximation that assumes the acceleration is constant over this frame
        rigidBody->velocity += rigidBody->acceleration * deltaTime;
        rigidBody->angularVelocity += rigidBody->angularAcceleration * deltaTime;

        // Update position and rotation using Euler integration.
        // r(n+1) = r(n) + h * v(n) where n is the previous frame, n+1 is this frame and h is delta time
        // This is an approximation that assumes the velocity is constant over this frame
        rigidBody->centerOfMass += rigidBody->velocity * deltaTime;
        rigidBody->rotate( rigidBody->angularVelocity * deltaTime );
    }

    // After updating all positions, check for collisions
    for (int i = 0; i < rigidBodies.size(); ++i)
    {
        for (int j = i + 1; j < rigidBodies.size(); ++j)
        {
            // Broad phase collision test
            if (rigidBodies[i]->getBoundingBox().intersects(rigidBodies[j]->getBoundingBox()))
            {
                bool didCollide = resolveCollision(rigidBodies[i], rigidBodies[j]);
#ifdef __DEBUG
                if (didCollide && !(rigidBodies[i] == rigidBodies[0] || rigidBodies[j] == rigidBodies[0]))
                {
                    debugQueue.push_back(rigidBodies[i]->getCollisionMeshWorld());
                    debugQueue.push_back(rigidBodies[j]->getCollisionMeshWorld());
                }
#endif
            }
        }
    }
}

void PhysicsEngine::registerRigidBody(const std::shared_ptr<RigidBody> &rigidBody) { rigidBodies.push_back(rigidBody); }


PhysicsEngine::CollisionInfo PhysicsEngine::getCollision(const std::shared_ptr<RigidBody>& rigidBodyA,
                                          const std::shared_ptr<RigidBody>& rigidBodyB)
{
    // Create a vector of all the normals as unit vectors | OPTIMISE: filter out any axis with the same gradient
    std::vector<Vector2F> normalsA = rigidBodyA->getNormals();
    std::vector<Vector2F> normalsB = rigidBodyB->getNormals();
    std::vector<Vector2F> axes;
    axes.insert(axes.end(), normalsA.begin(), normalsA.end());
    axes.insert(axes.end(), normalsB.begin(), normalsB.end());
    for (auto& axis : axes) axis = axis.getUnitVector(); // OPTIMISE: only get the unit vector if it is needed to calculate penetration

    Vector2F penetrationVector{std::numeric_limits<float>::max()};

    bool isColliding = false;

    for (auto axis : axes)
    {
        std::vector<Vector2F> rigidBodyACollisionMesh = rigidBodyA->getCollisionMeshWorld();
        float rigidBodyAMinProjection = rigidBodyACollisionMesh[0].dot(axis);
        float rigidBodyAMaxProjection = rigidBodyACollisionMesh[0].dot(axis);
        for (auto it = rigidBodyACollisionMesh.begin() + 1; it != rigidBodyACollisionMesh.end(); ++it)
        {
            rigidBodyAMinProjection = std::min(rigidBodyAMinProjection, it->dot(axis));
            rigidBodyAMaxProjection = std::max(rigidBodyAMaxProjection, it->dot(axis));
        }

        std::vector<Vector2F> rigidBodyBCollisionMesh = rigidBodyB->getCollisionMeshWorld();
        float rigidBodyBMinProjection = rigidBodyBCollisionMesh[0].dot(axis);
        float rigidBodyBMaxProjection = rigidBodyBCollisionMesh[0].dot(axis);
        for (auto it = rigidBodyBCollisionMesh.begin() + 1; it != rigidBodyBCollisionMesh.end(); ++it)
        {
            rigidBodyBMinProjection = std::min(rigidBodyBMinProjection, it->dot(axis));
            rigidBodyBMaxProjection = std::max(rigidBodyBMaxProjection, it->dot(axis));
        }

        float penetrationMagnitude{};
        if (rigidBodyBMaxProjection <= rigidBodyAMinProjection || rigidBodyAMaxProjection <= rigidBodyBMinProjection)
            return {Vector2F{}, Vector2F{}, false};
        else
            penetrationMagnitude = std::min(rigidBodyBMaxProjection - rigidBodyAMinProjection,
                                            rigidBodyAMaxProjection - rigidBodyBMinProjection);

        // Find the penetration with the smallest magnitude to work out how the objects should be separated
        if (penetrationMagnitude < penetrationVector.getMagnitude()) {
            penetrationVector = axis * penetrationMagnitude;

            // Multiple axis can give the same result despite not pointing the correct way ( i.e. parallel edges will
            // have opposite normals along the same axis) so reverse the direction if it is not pointing the right way
            Vector2F relativeVelocity = rigidBodyA->velocity - rigidBodyB->velocity;
            if (relativeVelocity.dot(penetrationVector) > 0.0f) {
                penetrationVector = -penetrationVector;
            }

        }

        isColliding = true;
    }


    return {penetrationVector, {}, isColliding};
}

float PhysicsEngine::calculateMomentOfInertia(const std::vector<Vector2F>& collisionMesh, float mass) {

    // TODO: This function needs to be reworked
    if (mass == std::numeric_limits<float>().infinity()) return mass;
    // Calculate the area of the mesh
    float area{};
    for (auto it = collisionMesh.begin() + 1; it != collisionMesh.end() - 1; ++it)
    {
        area += (*std::next(it) - *collisionMesh.begin()).cross(*it - *collisionMesh.begin()) / 2;
    }
    area = std::abs(area);

    // Calculate the density
    float density = mass / area;

    // Calculate the moment of inertia
    float newMomentOfInertia{};
    for (auto it = collisionMesh.begin() + 1; it != collisionMesh.end() - 1; ++it)
    {
        // Get the points of the triangle
        Vector2F p1 = *collisionMesh.begin();
        Vector2F p2 = *it;
        Vector2F p3 = *std::next(it);

        //Get the vectors of the triangle
        Vector2F v1 = p2 - p1;
        Vector2F v2 = p3 - p1;

        // Find the width of the base of the triangle
        float width = v1.getMagnitude();
        // Get the area to calculate the height from A = wh / 2
        float triangleArea = std::abs( v2.cross(v1) / 2 );
        float height = ( 2 * triangleArea ) / width;

        // Find a fourth point to split the triangle into two right-angled triangles
        Vector2F p4 = p1 + v1 * (v2.dot(v1)/powf(width, 2));
        float width1 = p1.getDistanceTo(p4);
        float width2 = p4.getDistanceTo(p2);

        // Calculate the moments of inertia around p3 using the equation for a right-angled triangle
        float i1 = density * width1 * height * ((powf(height, 2) / 4) + (powf(width1, 2) / 12));
        float i2 = density * width2 * height * ((powf(height, 2) / 4) + (powf(width2, 2) / 12));

        // Use the parallel axis theorem to shift the moment of inertia to the origin (0, 0)
        Vector2F centroid1 = {
                (p2.x + p3.x + p4.x) / 3,
                (p2.y + p3.y + p4.y) / 3,
        };
        Vector2F centroid2 = {
                (p1.x + p3.x + p4.x) / 3,
                (p1.y + p3.y + p4.y) / 3,
        };
        float mass1 = 0.5f * width1 * height * density;
        float mass2 = 0.5f * width2 * height * density;

        float i1cm = i1 - (mass1 * powf(centroid1.getDistanceTo(p3), 2));
        float i2cm = i2 - (mass2 * powf(centroid2.getDistanceTo(p3), 2));

        float momentOfInertia1 = i1cm + (mass1 * powf(centroid1.getDistanceTo({0, 0}), 2));
        float momentOfInertia2 = i2cm + (mass2 * powf(centroid2.getDistanceTo({0, 0}), 2));

        if ((p1 - p3).cross(p4 - p3) > 0) {
            newMomentOfInertia += momentOfInertia1;
        } else {
            newMomentOfInertia -= momentOfInertia1;
        }
        if ((p4 - p3).cross(p2 - p3) > 0) {
            newMomentOfInertia += momentOfInertia2;
        } else {
            newMomentOfInertia -= momentOfInertia2;
        }
    }

    return std::abs(newMomentOfInertia);
}

void PhysicsEngine::setGravity(float gravityValue) { gravity = Vector2F(0, gravityValue); }

void PhysicsEngine::setGravity(Vector2F gravityValue) { gravity = gravityValue; }

bool PhysicsEngine::resolveCollision(const std::shared_ptr<RigidBody>& rigidBodyA,
                                     const std::shared_ptr<RigidBody>& rigidBodyB)
{
    CollisionInfo collisionInfo = getCollision(rigidBodyA, rigidBodyB);
    if (!collisionInfo.isColliding) return false; // No collision

    // OPTIMISE: Some values calculated in getCollision can be cached for use here such as the collision normal
    Vector2F collisionVector = collisionInfo.penetrationVector;
    Vector2F collisionNormal = collisionInfo.penetrationVector.getUnitVector();
    Vector2F collisionPoint = collisionInfo.collisionPoint;

    Vector2F rA{};// = collisionPoint - rigidBodyA->centerOfMass;
    Vector2F rB{};// = collisionPoint - rigidBodyB->centerOfMass;

    // Calculate the impulse magnitude (the acceleration to apply to each object to respond to the collision)
    float rA_perp_dot_n = rA.x * collisionNormal.y - rA.y * collisionNormal.x;
    float rB_perp_dot_n = rB.x * collisionNormal.y - rB.y * collisionNormal.x;
    //Vector2F relativeVelocity = (rigidBodyA->velocity + rigidBodyA->angularVelocity * rA_perp_dot_n) - (rigidBodyB->velocity + rigidBodyB->angularVelocity * rB_perp_dot_n);
    Vector2F relativeVelocity = rigidBodyA->velocity - rigidBodyB->velocity;
    float elasticity = std::min(rigidBodyA->restitution, rigidBodyB->restitution);


    // OPTIMISE: I might be able to avoid getting the unit vector of the collision vector here if I add back the n.n that was cancelled out in the denominator
    float denominator = rigidBodyA->inverseMass + rigidBodyB->inverseMass;// +
                        (rA_perp_dot_n * rA_perp_dot_n) / rigidBodyA->momentOfInertia +
                        (rB_perp_dot_n * rB_perp_dot_n) / rigidBodyB->momentOfInertia;

    float impulseMagnitude = -(1 + elasticity) * relativeVelocity.dot(collisionNormal) / denominator;

    Vector2F impulseVector = collisionNormal * impulseMagnitude;

    // Calculate how much each RigidBody will be moved to negate the collision based off each RigidBody's velocity
    float totalVelocityMagnitude = rigidBodyA->velocity.getMagnitude() + rigidBodyB->velocity.getMagnitude();
    float velocityProportionA = (totalVelocityMagnitude != 0) ? rigidBodyA->velocity.getMagnitude() /
                                                                    totalVelocityMagnitude : 0.5f;
    float velocityProportionB = (totalVelocityMagnitude != 0) ? rigidBodyB->velocity.getMagnitude() /
                                                                   totalVelocityMagnitude : 0.5f;

    // Adjust the positions to negate collision and respond to it
    rigidBodyA->angularVelocity += (rA_perp_dot_n * impulseMagnitude) / rigidBodyA->momentOfInertia;
    rigidBodyB->angularVelocity -= (rB_perp_dot_n * impulseMagnitude) / rigidBodyB->momentOfInertia;

    rigidBodyA->velocity += impulseVector * rigidBodyA->inverseMass;
    rigidBodyB->velocity -= impulseVector * rigidBodyB->inverseMass;

    rigidBodyA->centerOfMass += collisionVector * velocityProportionA;
    rigidBodyB->centerOfMass -= collisionVector * velocityProportionB;


    return true;
}

#ifdef __DEBUG
void PhysicsEngine::drawDebugDEBUG()
{
    if (doDebugDEBUG)
    {
        auto graphicsEngine =  XCube2Engine::getInstance()->getGraphicsEngine();

        // Draw all RigidBodies
        for (const auto& rigidBody : rigidBodies) {

            // Draw the polygon outline
            graphicsEngine->setDrawColor(SDL_COLOR_RED);
            graphicsEngine->drawPolygon(rigidBody->getCollisionMeshWorld());
            // Draw a + for the centerOfMass
            graphicsEngine->drawLine({Vector2F{rigidBody->centerOfMass.x - 5, rigidBody->centerOfMass.y}, Vector2F{rigidBody->centerOfMass.x + 5, rigidBody->centerOfMass.y}});
            graphicsEngine->drawLine({Vector2F{rigidBody->centerOfMass.x, rigidBody->centerOfMass.y - 5}, Vector2F{rigidBody->centerOfMass.x, rigidBody->centerOfMass.y + 5}});
        }
        // Draw all shapes in the debugQueue
        for (const auto& polygon : debugQueue)
        {
            graphicsEngine->setDrawColor(SDL_COLOR_GREEN);
            graphicsEngine->drawPolygon(polygon);
        }
        debugQueue.clear();
    }
}

#endif
