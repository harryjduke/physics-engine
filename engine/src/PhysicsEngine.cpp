#include "PhysicsEngine.h"

#include <algorithm>
#include <cmath>

#include "XCube2d.h"

/* PHYSICS OBJECT */

RigidBody::RigidBody(const Vector2F & center, float width, float height, float mass, float rotation, float restitution) :
        center(center),
        vertices{
        {-width / 2, -height / 2},
        {width / 2,  -height / 2},
        {width / 2,  height / 2},
        {-width / 2, height / 2}},
        inverseMass(mass != 0 ? 1.f/mass : 1.f),
        restitution(restitution)
#ifdef __DEBUG
        ,isCollidingDEBUG{}
#endif
{
    rotate(rotation);
}

RigidBody::RigidBody(const Vector2F & center, float width, float height, bool isStatic, float rotation, float restitution) :
        center(center),
        vertices{
        {-width / 2, -height / 2},
        {width / 2,  -height / 2},
        {width / 2,  height / 2},
        {-width / 2, height / 2}},
        inverseMass(isStatic ? 0.f : 1.f ),
        restitution(restitution)
#ifdef __DEBUG
        ,isCollidingDEBUG{}
#endif
{
    rotate(rotation);
}

RigidBody::RigidBody(const Vector2F& center, const std::vector<Vector2F>& vertices, float mass, float restitution) :
        center(center),
        vertices(vertices),
        inverseMass(mass > 0 ? 1.f/mass : 1.f),
        restitution(restitution)
#ifdef __DEBUG
        ,isCollidingDEBUG{}
#endif
{}

RigidBody::RigidBody(const Vector2F& center, const std::vector<Vector2F>& vertices, bool isStatic, float restitution) :
        center(center),
        vertices(vertices),
        inverseMass(isStatic ? 0.f : 1.f ),
        restitution(restitution)
#ifdef __DEBUG
        ,isCollidingDEBUG{}
#endif
{}

void RigidBody::setMass(float mass) { if (mass > 0) inverseMass = 1 / mass; }

void RigidBody::rotate(float angleRadians) {
    // Calculate sine and cosine of the angleRadians
    float sinAngle = std::sin(angleRadians);
    float cosAngle = std::cos(angleRadians);

    for (auto& vertex : vertices){
        vertex = { // Rotate the vertex
            vertex.x * cosAngle - vertex.y * sinAngle,
            vertex.x * sinAngle + vertex.y * cosAngle
        };
    }
}

std::vector<Vector2F> RigidBody::getNormals() {
    std::vector<Vector2F> normals{};
    for (auto it = vertices.begin(); it != vertices.end(); ++it)
    {
        Line2f edge;
        if (std::next(it) != vertices.end()) edge = {*it, *next(it)};
        else edge = {*it, *vertices.begin()};

        normals.push_back(edge.getNormal());
    }
    // OPTIMISE: cache the normals in a private member
    return normals;
}

RectF RigidBody::getBoundingBox() const {
    if (vertices.empty()) return RectF{};

    RectF boundingBox{vertices[0].x, vertices[0].y, 0.0f, 0.0f }; // Initialize width and height to 0

    for (auto it = std::next(vertices.begin()); it != vertices.end(); it++) {
        boundingBox.x = std::min(boundingBox.x, it->x);
        boundingBox.y = std::min(boundingBox.y, it->y);
    }

    for (auto vertex : vertices) {
        boundingBox.w = std::max(boundingBox.w, vertex.x - boundingBox.x); // Update width based on x
        boundingBox.h = std::max(boundingBox.h, vertex.y - boundingBox.y); // Update height based on y
    }

    return boundingBox + center;
}

std::vector<Vector2F> RigidBody::getVerticesWorld() const {
    std::vector<Vector2F> verticesWorld = vertices;

    for (auto& vertex : verticesWorld)
    {
        vertex += center;
    }
    return verticesWorld;
}

/* PHYSICS ENGINE */

PhysicsEngine::PhysicsEngine() :
        gravity(Vector2F(0, DEFAULT_GRAVITY))
#ifdef __DEBUG
        ,doDebugDEBUG{}
#endif
{ }

void PhysicsEngine::setGravity(float gravityValue) { gravity = Vector2F(0, gravityValue); }
void PhysicsEngine::setGravity(Vector2F gravityValue) { gravity = gravityValue; }

void PhysicsEngine::registerRigidBody(const std::shared_ptr<RigidBody> &rigidBody) { rigidBodies.push_back(rigidBody); }


void PhysicsEngine::update(float deltaTime)
{
    for (const auto& rigidBody : rigidBodies)
    {
#ifdef __DEBUG
        rigidBody->isCollidingDEBUG = false;
#endif
        // Calculate forces
        Vector2F force{};

        // Apply acceleration (a = f/m)
        rigidBody->acceleration = force * rigidBody->inverseMass;
        // Add gravity (as gravity is a constant acceleration, we don't need to calculate the force)
        if (rigidBody->inverseMass != 0) // If the rigidBody is not static
            rigidBody->acceleration += gravity;

        // Update velocity using Euler integration.
        // v(n+1) = v(n) + h * a(n) where n is the previous frame, n+1 is this frame and h is delta time
        // This is an approximation that assumes the acceleration is constant over this frame
        rigidBody->velocity += rigidBody->acceleration * deltaTime;
        // Update position using Euler integration.
        // r(n+1) = r(n) + h * v(n) where n is the previous frame, n+1 is this frame and h is delta time
        // This is an approximation that assumes the velocity is constant over this frame
        rigidBody->center += rigidBody->velocity * deltaTime;
    }

    // After updating all positions, check for collisions
    for (int i = 0; i < rigidBodies.size(); ++i)
    {
        for (int j = i + 1; j < rigidBodies.size(); ++j)
        {
            // OPTIMISE: Add broad phase collision test
            if ( resolveCollision(rigidBodies[i], rigidBodies[j])
                && !(rigidBodies[i] == rigidBodies[0] || rigidBodies[j] == rigidBodies[0]))
            {
#ifdef __DEBUG
                debugQueue.push_back(rigidBodies[i]->getVerticesWorld());
                debugQueue.push_back(rigidBodies[j]->getVerticesWorld());
#endif
            }
        }
    }
}

Vector2F PhysicsEngine::getCollision( const std::shared_ptr<RigidBody>& rigidBodyA,
                                      const std::shared_ptr<RigidBody>& rigidBodyB )
{
    // Create a vector of all the normals as unit vectors | OPTIMISE: filter out any axis with the same gradient
    std::vector<Vector2F> normalsA = rigidBodyA->getNormals();
    std::vector<Vector2F> normalsB = rigidBodyB->getNormals();
    std::vector<Vector2F> axes;
    axes.insert(axes.end(), normalsA.begin(), normalsA.end());
    axes.insert(axes.end(), normalsB.begin(), normalsB.end());
    for (auto& axis : axes) axis = axis.getUnitVector(); // OPTIMISE: only get the unit vector if it is needed to calculate penetration

    Vector2F penetrationVector{std::numeric_limits<float>::max()};

    // Check if the RigidBodies are separated on each axis
    for (auto axis : axes)
    {
        // Get the min and max projections for rigidBodyA
        std::vector<Vector2F> rigidBodyAVertices = rigidBodyA->getVerticesWorld();
        float rigidBodyAMinProjection = rigidBodyAVertices[0].getDotProduct(axis);
        float rigidBodyAMaxProjection = rigidBodyAVertices[0].getDotProduct(axis);
        for (auto it = rigidBodyAVertices.begin() + 1; it != rigidBodyAVertices.end(); ++it)
        {
            rigidBodyAMinProjection = std::min(rigidBodyAMinProjection, it->getDotProduct(axis));
            rigidBodyAMaxProjection = std::max(rigidBodyAMaxProjection, it->getDotProduct(axis));
        }

        // Get the min and max projections for rigidBodyB
        std::vector<Vector2F> rigidBodyBVertices = rigidBodyB->getVerticesWorld();
        float rigidBodyBMinProjection = rigidBodyBVertices[0].getDotProduct(axis);
        float rigidBodyBMaxProjection = rigidBodyBVertices[0].getDotProduct(axis);
        for (auto it = rigidBodyBVertices.begin() + 1; it != rigidBodyBVertices.end(); ++it)
        {
            rigidBodyBMinProjection = std::min(rigidBodyBMinProjection, it->getDotProduct(axis));
            rigidBodyBMaxProjection = std::max(rigidBodyBMaxProjection, it->getDotProduct(axis));
        }

        float penetrationMagnitude{};
        if (rigidBodyBMaxProjection < rigidBodyAMinProjection || rigidBodyAMaxProjection < rigidBodyBMinProjection)
            // If there is a separation on any axis then the RigidBodies cannot be colliding so exit early with default Vector2F
            return Vector2F{};
        else
            penetrationMagnitude = std::min(rigidBodyBMaxProjection - rigidBodyAMinProjection,
                                            rigidBodyAMaxProjection - rigidBodyBMinProjection);

        // Find the penetration with the smallest magnitude to work out how the objects should be separated
        if (penetrationMagnitude < penetrationVector.getMagnitude()) {
            penetrationVector = axis * penetrationMagnitude;

            // Multiple axis can give the same result despite not pointing the correct way ( i.e. parallel edges will
            // have opposite normals along the same axis) so reverse the direction if it is not pointing the right way
            Vector2F relativeVelocity = rigidBodyA->velocity - rigidBodyB->velocity;
            if (relativeVelocity.getDotProduct(penetrationVector) > 0.0f) {
                penetrationVector = -penetrationVector;
            }
        }

    }

    // There was no separation on any axis so the RigidBodies must be colliding, return the smallest penetration
    return penetrationVector;
}

bool PhysicsEngine::resolveCollision(const std::shared_ptr<RigidBody>& rigidBodyA,
                                     const std::shared_ptr<RigidBody>& rigidBodyB)
{
    Vector2F collisionVector = getCollision(rigidBodyA, rigidBodyB);
    // OPTIMISE: Some values calculated in getCollision can be cached for use here such as the collision normal

    if (collisionVector == Vector2F{}) return false; // No collision

    // Calculate the impulse magnitude (the acceleration to apply to each object to respond to the collision)
    Vector2F relativeVelocity = rigidBodyA->velocity - rigidBodyB->velocity;
    float elasticity = std::min(rigidBodyA->restitution, rigidBodyB->restitution);
    float impulseMagnitude = -(1 + elasticity) * relativeVelocity.getDotProduct(collisionVector.getUnitVector()) /
                                            (rigidBodyA->inverseMass + rigidBodyB->inverseMass);

    Vector2F impulseVector = collisionVector.getUnitVector() * impulseMagnitude;

    // Calculate how much each RigidBody will be moved to negate the collision based off each RigidBody's velocity
    float totalVelocityMagnitude = rigidBodyA->velocity.getMagnitude() + rigidBodyB->velocity.getMagnitude();
    float velocityProportionA = (totalVelocityMagnitude != 0) ? rigidBodyA->velocity.getMagnitude() /
                                                                    totalVelocityMagnitude : 0.5f;
    float velocityProportionB = (totalVelocityMagnitude != 0) ? rigidBodyB->velocity.getMagnitude() /
                                                                   totalVelocityMagnitude : 0.5f;

    // Adjust the positions to negate collision and respond to it
    rigidBodyA->center += collisionVector * velocityProportionA;
    rigidBodyA->velocity += impulseVector * rigidBodyA->inverseMass;
    rigidBodyB->center -= collisionVector * velocityProportionB;
    rigidBodyB->velocity -= impulseVector * rigidBodyB->inverseMass;

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
            graphicsEngine->drawPolygon(rigidBody->getVerticesWorld());
            // Draw a + for the center
            graphicsEngine->drawLine({Vector2F{rigidBody->center.x - 5, rigidBody->center.y},Vector2F{rigidBody->center.x + 5, rigidBody->center.y}});
            graphicsEngine->drawLine({Vector2F{rigidBody->center.x, rigidBody->center.y - 5},Vector2F{rigidBody->center.x, rigidBody->center.y + 5}});
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
