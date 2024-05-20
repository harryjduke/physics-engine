#include "PhysicsEngine.h"

#include <algorithm>
#include <cmath>

#include "XCube2d.h"

/* PHYSICS OBJECT */

RigidBody::RigidBody(const Vector2F & center, float width, float height, float mass, float rotation) :
        center(center),
        vertices{
        {-width / 2, -height / 2},
        {width / 2,  -height / 2},
        {width / 2,  height / 2},
        {-width / 2, height / 2}},
        inverseMass(mass > 0 ? 1.f/mass : 1.f)
#ifdef __DEBUG
        ,isCollidingDEBUG{}
#endif
{
    rotate(rotation);
}

RigidBody::RigidBody(const Vector2F & center, float width, float height, bool isStatic, float rotation) :
        center(center),
        vertices{
        {-width / 2, -height / 2},
        {width / 2,  -height / 2},
        {width / 2,  height / 2},
        {-width / 2, height / 2}},
        inverseMass(isStatic ? 0.f : 1.f )
#ifdef __DEBUG
        ,isCollidingDEBUG{}
#endif
{
    rotate(rotation);
}

RigidBody::RigidBody(const Vector2F& center, const std::vector<Vector2F>& vertices, float mass) :
        center(center),
        vertices(vertices),
        inverseMass(mass > 0 ? 1.f/mass : 1.f)
#ifdef __DEBUG
        ,isCollidingDEBUG{}
#endif
{}

RigidBody::RigidBody(const Vector2F& center, const std::vector<Vector2F>& vertices, bool isStatic) :
        center(center),
        vertices(vertices),
        inverseMass(isStatic ? 0.f : 1.f )
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

        // Update velocity (v = u + at)
        rigidBody->velocity += rigidBody->acceleration * deltaTime;
        // Update position (s = ut)
        rigidBody->center += rigidBody->velocity * deltaTime;
    }

    // After updating all positions, check for collisions
    for (int i = 0; i < rigidBodies.size(); ++i)
    {
        for (int j = i + 1; j < rigidBodies.size(); ++j)
        {
            // OPTIMISE: Add broad phase collision test
            if (isColliding(rigidBodies[i], rigidBodies[j]))
            {
#ifdef __DEBUG
                rigidBodies[i]->isCollidingDEBUG = true;
                rigidBodies[j]->isCollidingDEBUG = true;
#endif
            }
        }
    }
}

bool PhysicsEngine::isColliding(const std::shared_ptr<RigidBody>& rigidBody1,
                                const std::shared_ptr<RigidBody>& rigidBody2)
{
    //return object1->getBoundingBox().intersects(object2->getBoundingBox());

    // Create a vector of all the normals as unit vectors | OPTIMISE filter out any axis with the same gradient
    std::vector<Vector2F> normals1 = rigidBody1->getNormals();
    std::vector<Vector2F> normals2 = rigidBody2->getNormals();
    std::vector<Vector2F> axes;
    axes.insert(axes.end(), normals1.begin(), normals1.end());
    axes.insert(axes.end(), normals2.begin(), normals2.end());
    for (auto& axis : axes) axis = axis.getUnitVector();

    // Check if the RigidBodies are separated on each axis
    for (auto axis : axes)
    {
        // Get the min and max projections for rigidBody1
        std::vector<Vector2F> rigidBody1Vertices = rigidBody1->getVerticesWorld();
        float rigidBody1MinProjection = rigidBody1Vertices[0].getDotProduct(axis);
        float rigidBody1MaxProjection = rigidBody1Vertices[0].getDotProduct(axis);
        for (auto it = rigidBody1Vertices.begin() + 1; it != rigidBody1Vertices.end(); ++it)
        {
            rigidBody1MinProjection = std::min(rigidBody1MinProjection, it->getDotProduct(axis));
            rigidBody1MaxProjection = std::max(rigidBody1MaxProjection, it->getDotProduct(axis));
        }

        // Get the min and max projections for rigidBody2
        std::vector<Vector2F> rigidBody2Vertices = rigidBody2->getVerticesWorld();
        float rigidBody2MinProjection = rigidBody2Vertices[0].getDotProduct(axis);
        float rigidBody2MaxProjection = rigidBody2Vertices[0].getDotProduct(axis);
        for (auto it = rigidBody2Vertices.begin() + 1; it != rigidBody2Vertices.end(); ++it)
        {
            rigidBody2MinProjection = std::min(rigidBody2MinProjection, it->getDotProduct(axis));
            rigidBody2MaxProjection = std::max(rigidBody2MaxProjection, it->getDotProduct(axis));
        }

        // If there is a separation on any axis then the RigidBodies cannot be colliding so exit early with false
        if (rigidBody2MaxProjection < rigidBody1MinProjection || rigidBody1MaxProjection < rigidBody2MinProjection)
            return false;
    }

    return true; // There was no separation on any axis so the RigidBodies must be colliding
}

void PhysicsEngine::resolveCollision(std::shared_ptr<RigidBody> rigidBody1, std::shared_ptr<RigidBody> rigidBody2)
{

}

#ifdef __DEBUG
void PhysicsEngine::drawDebugDEBUG()
{
    if (doDebugDEBUG)
    {
        auto graphicsEngine =  XCube2Engine::getInstance()->getGraphicsEngine();

        //int i = 0;
        for (const auto& rigidBody : rigidBodies)
        {
            graphicsEngine->setDrawColor(SDL_COLOR_RED);
            if (rigidBody->isCollidingDEBUG) graphicsEngine->setDrawColor(SDL_COLOR_GREEN);
            //graphicsEngine->drawRect(static_cast<Rectangle2I>(rigidBody->getBoundingBox()).getSDLRect());
            //graphicsEngine->setDrawColor(SDL_COLOR_AQUA);
            graphicsEngine->drawPolygon(rigidBody->getVerticesWorld());
        }
    }
}
#endif
