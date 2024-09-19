//  This file is part of CI517_GameEngine <https://github.com/harryjduke/CI517_GameEngine>.
//  Copyright (c) 2024 Harry Duke <harryjduke@gmail.com>
//
//  This program is distributed under the terms of the GNU General Public License version 2.
//  You should have received a copy of the GNU General Public License along with this program.
//  If not, see <https://github.com/harryjduke/CI517_GameEngine/blob/main/LICENSE> or <https://www.gnu.org/licenses/>.

#include "PhysicsEngine.h"

#include <algorithm>
#include <cmath>

#include "XCube2d.h"

/* RIGID BODY */

RigidBody::RigidBody(const Vector2F &centerOfMass, const float width, const float height, const float mass,
                     const float rotation, const float restitution) :
        centerOfMass(centerOfMass),
        inverseMass(mass != 0 ? 1.f/mass : 1.f),
        restitution(restitution),
        velocity{},
        acceleration{},
        angularVelocity{},
        angularAcceleration{},
        collisionMesh{},
        momentOfInertia{}
#ifdef __DEBUG
        , isCollidingDEBUG{}
#endif
{
    setCollisionMesh({{-width / 2, -height / 2},
                      {width / 2,  -height / 2},
                      {width / 2,  height / 2},
                      {-width / 2, height / 2}});
    rotate(rotation);
}

RigidBody::RigidBody(const Vector2F &centerOfMass, const float width, const float height, const bool isStatic,
                     const float rotation, const float restitution) :
        centerOfMass(centerOfMass),
        inverseMass(isStatic ? 0.f : 1.f ),
        restitution(restitution),
        velocity{},
        acceleration{},
        angularVelocity{},
        angularAcceleration{},
        collisionMesh{},
        momentOfInertia{}
#ifdef __DEBUG
        , isCollidingDEBUG{}
#endif
{
    setCollisionMesh({{-width / 2, -height / 2},
                      {width / 2,  -height / 2},
                      {width / 2,  height / 2},
                      {-width / 2, height / 2}});
    rotate(rotation);
}

RigidBody::RigidBody(const Vector2F &centerOfMass, const std::vector<Vector2F>& collisionMesh, const float mass,
                     const float restitution) :
        centerOfMass(centerOfMass),
        inverseMass(mass > 0 ? 1.f/mass : 1.f),
        restitution(restitution),
        velocity{},
        acceleration{},
        angularVelocity{},
        angularAcceleration{},
        collisionMesh{},
        momentOfInertia{}
#ifdef __DEBUG
        , isCollidingDEBUG{}
#endif
{
    setCollisionMesh(collisionMesh);
}

RigidBody::RigidBody(const Vector2F &centerOfMass, const std::vector<Vector2F> &collisionMesh, const bool isStatic,
                     const float restitution) :
        centerOfMass(centerOfMass),
        inverseMass(isStatic ? 0.f : 1.f ),
        restitution(restitution),
        velocity{},
        acceleration{},
        angularVelocity{},
        angularAcceleration{},
        collisionMesh{},
        momentOfInertia{}
#ifdef __DEBUG
        ,isCollidingDEBUG{}
#endif
{
    setCollisionMesh(collisionMesh);
}

void RigidBody::setMass(float mass) { if (mass > 0) inverseMass = 1 / mass; }

const std::vector<Vector2F> &RigidBody::getCollisionMesh() const {
    return collisionMesh;
}

void RigidBody::setCollisionMesh(std::vector<Vector2F> newCollisionMesh) {
    float newMomentOfInertia;
    try {
        Vector2F centroid = PhysicsEngine::calculateCentroid(newCollisionMesh);
        // Adjust vertices relative to the new centroid
        for (Vector2F& vertex : newCollisionMesh) vertex -= centroid;

        newMomentOfInertia = PhysicsEngine::calculateMomentOfInertia(newCollisionMesh, inverseMass);
    }
    catch (const EngineException& e) {
        throw EngineException("Failed to set collision mesh", e.what());
    }
    collisionMesh = std::move(newCollisionMesh);
    momentOfInertia = newMomentOfInertia;
}

std::vector<Vector2F> RigidBody::getCollisionMeshWorld() const {
    std::vector<Vector2F> collisionMeshWorld = collisionMesh;
    for (auto& vertex : collisionMeshWorld) vertex += centerOfMass;
    return collisionMeshWorld;
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

    RectF boundingBox{collisionMesh[0].x, collisionMesh[0].y, 0.0f, 0.0f };
    for (auto it = std::next(collisionMesh.begin()); it != collisionMesh.end(); ++it) {
        boundingBox.x = std::min(boundingBox.x, it->x);
        boundingBox.y = std::min(boundingBox.y, it->y);
    }
    for (const Vector2F vertex : collisionMesh) {
        boundingBox.w = std::max(boundingBox.w, vertex.x - boundingBox.x);
        boundingBox.h = std::max(boundingBox.h, vertex.y - boundingBox.y);
    }
    return boundingBox + centerOfMass;
}

void RigidBody::rotate(const float angleRadians) {
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

float RigidBody::getMomentOfInertia() const {
    return momentOfInertia;
}

/* PHYSICS ENGINE */


PhysicsEngine::PhysicsEngine() :
        gravity(Vector2F(0, DEFAULT_GRAVITY))
#ifdef __DEBUG
        ,doDebugDEBUG{}
#endif
{ }

void PhysicsEngine::update(const float deltaTime)
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
        rigidBody->angularAcceleration = torque / rigidBody->getMomentOfInertia();

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
                if (CollisionInfo collisionInfo = getCollision(rigidBodies[i], rigidBodies[j]);
                    collisionInfo.isColliding) {
                    resolveCollision(rigidBodies[i], rigidBodies[j], collisionInfo);
                    /**
#ifdef __DEBUG
                    if (collisionInfo.isColliding)
                    {
                        debugQueue.push_back({collisionInfo.collisionPoint - Vector2F{5, 0},
                                              collisionInfo.collisionPoint + Vector2F{5, 0}});
                        debugQueue.push_back({collisionInfo.collisionPoint - Vector2F{0, 5},
                                              collisionInfo.collisionPoint + Vector2F{0, 5}});
                        debugQueue.push_back({rigidBodies[j]->centerOfMass, {collisionInfo.penetrationVector * 10.f + rigidBodies[j]->centerOfMass}});
                        if (!(rigidBodies[i] == rigidBodies[0] || rigidBodies[j] == rigidBodies[0]))
                        {
                            debugQueue.push_back(rigidBodies[i]->getCollisionMeshWorld());
                            debugQueue.push_back(rigidBodies[j]->getCollisionMeshWorld());
                        }
                    }
#endif
                    **/
                }
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

    Vector2F relativeVelocityAB = rigidBodyB->velocity - rigidBodyA->velocity;

    float timeSinceCollision = std::numeric_limits<float>::max();
    Vector2F penetrationVector{std::numeric_limits<float>::max()};
    Vector2F collisionAxis{};
    Vector2F collisionPoint;
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

        if (rigidBodyBMaxProjection > rigidBodyAMinProjection && rigidBodyAMaxProjection > rigidBodyBMinProjection)
        {
            float penetrationMagnitude = std::min(rigidBodyBMaxProjection - rigidBodyAMinProjection,
                                            rigidBodyAMaxProjection - rigidBodyBMinProjection);
            float timeSinceCollisionOnCurrentAxis = std::abs(penetrationMagnitude / relativeVelocityAB.dot(axis));

            // Find the penetration with the smallest magnitude to work out how the objects should be separated
            if (timeSinceCollisionOnCurrentAxis < timeSinceCollision)
            {
                timeSinceCollision = timeSinceCollisionOnCurrentAxis;
                penetrationVector = axis * penetrationMagnitude;
                collisionAxis = axis;

                // Multiple axis can give the same result despite not pointing the correct way ( i.e. parallel edges will
                // have opposite normals along the same axis) so reverse the direction if it is not pointing the right way
                Vector2F relativeVelocity = rigidBodyA->velocity - rigidBodyB->velocity;
                if (relativeVelocity.dot(penetrationVector) > 0.0f) {
                    penetrationVector = -penetrationVector;
                }

            }

            isColliding = true;
        }
        else return CollisionInfo{};
    }

    auto getMinMaxProjectionsAlongAxis =
            [](const Vector2F &axis, const std::vector<Vector2F> &collisionMesh,
               std::vector<Vector2F> &minVertices, std::vector<Vector2F> &maxVertices,
               float &minProjection, float &maxProjection)
            {
                minVertices = maxVertices = {collisionMesh[0]};
                minProjection = maxProjection = collisionMesh[0].dot(axis);

                for (int i = 1; i < collisionMesh.size(); ++i) {
                    Vector2F vertex = collisionMesh[i];
                    float projection = vertex.dot(axis);

                    if (std::abs(projection - minProjection) <= std::max(std::abs(projection), std::abs(minProjection)) * SDL_FLT_EPSILON * 10)
                        minVertices.push_back(vertex);
                    else if (projection < minProjection)
                    {
                        minVertices = {vertex};
                        minProjection = projection;
                    }


                    if (std::abs(projection - maxProjection) <= std::max(std::abs(projection), std::abs(maxProjection)) * SDL_FLT_EPSILON * 10)
                        maxVertices.push_back(vertex);
                    else if (projection > maxProjection) {
                        maxVertices = {vertex};
                        maxProjection = projection;
                    }
                }
            };

    std::vector<Vector2F> rigidBodyACollisionMesh = rigidBodyA->getCollisionMeshWorld();
    std::vector<Vector2F> rigidBodyBCollisionMesh = rigidBodyB->getCollisionMeshWorld();
    std::vector<Vector2F> rigidBodyAMinVertices, rigidBodyAMaxVertices, rigidBodyBMinVertices, rigidBodyBMaxVertices;
    float rigidBodyAMinProjection, rigidBodyAMaxProjection, rigidBodyBMinProjection, rigidBodyBMaxProjection;

    getMinMaxProjectionsAlongAxis(collisionAxis, rigidBodyACollisionMesh,
                                  rigidBodyAMinVertices, rigidBodyAMaxVertices,
                                  rigidBodyAMinProjection, rigidBodyAMaxProjection);
    getMinMaxProjectionsAlongAxis(collisionAxis, rigidBodyBCollisionMesh,
                                  rigidBodyBMinVertices, rigidBodyBMaxVertices,
                                  rigidBodyBMinProjection, rigidBodyBMaxProjection);

    std::vector<Vector2F> rigidBodyACollidingVertices;
    float rigidBodyACollidingProjection;
    std::vector<Vector2F> rigidBodyBCollidingVertices;
    float rigidBodyBCollidingProjection;
    if (rigidBodyAMaxProjection - rigidBodyBMinProjection > rigidBodyBMaxProjection - rigidBodyAMinProjection)
    {
        rigidBodyACollidingVertices = rigidBodyAMinVertices;
        rigidBodyACollidingProjection = rigidBodyAMinProjection;
        rigidBodyBCollidingVertices = rigidBodyBMaxVertices;
        rigidBodyBCollidingProjection = rigidBodyBMaxProjection;
    }
    else
    {
        rigidBodyACollidingVertices = rigidBodyAMaxVertices;
        rigidBodyACollidingProjection = rigidBodyAMaxProjection;
        rigidBodyBCollidingVertices = rigidBodyBMinVertices;
        rigidBodyBCollidingProjection = rigidBodyBMinProjection;
    }
    if (rigidBodyACollidingVertices.size() == 1)
        collisionPoint = rigidBodyACollidingVertices[0];
    else if (rigidBodyBCollidingVertices.size() == 1)
        collisionPoint = rigidBodyBCollidingVertices[0];
    else
    {
        Vector2F collisionAxisPerpendicular = {-collisionAxis.y, collisionAxis.x};
        std::vector<float> edgeProjections = {
                rigidBodyACollidingVertices.front().dot(collisionAxisPerpendicular),
                rigidBodyACollidingVertices.back().dot(collisionAxisPerpendicular),
                rigidBodyBCollidingVertices.front().dot(collisionAxisPerpendicular),
                rigidBodyBCollidingVertices.back().dot(collisionAxisPerpendicular)
        };

        sort(edgeProjections.begin(), edgeProjections.end());

        collisionPoint = collisionAxisPerpendicular * ((edgeProjections[1] + edgeProjections[2]) / 2) +
                         collisionAxis * ((rigidBodyACollidingProjection + rigidBodyBCollidingProjection) / 2);
    }

    return {isColliding, penetrationVector, collisionPoint, timeSinceCollision};
}

bool PhysicsEngine::resolveCollision(const std::shared_ptr<RigidBody>& rigidBodyA,
                                     const std::shared_ptr<RigidBody>& rigidBodyB,
                                     const CollisionInfo &collisionInfo)
{
    if (collisionInfo.penetrationVector == Vector2F{}) return false; // Early exit for invalid collisionVector

    // OPTIMISE: Some values calculated in getCollision can be cached for use here such as the collision normal


    // Calculate the impulse (the velocity to apply to each object to respond to the collision)
    // OPTIMISE: I might be able to avoid getting the unit vector of the collision vector here if I add back the n.n that was cancelled out in the denominator
    const Vector2F collisionNormal = collisionInfo.penetrationVector.getUnitVector();
    const float elasticity = std::min(rigidBodyA->restitution, rigidBodyB->restitution);
    const Vector2F relativeCollisionPointAP = collisionInfo.collisionPoint - rigidBodyA->centerOfMass;
    const Vector2F relativeCollisionPointBP = collisionInfo.collisionPoint - rigidBodyB->centerOfMass;
    const float relativeCollisionPointAPPerpendicularDotN = relativeCollisionPointAP.x * collisionNormal.y -
                                                            relativeCollisionPointAP.y * collisionNormal.x;
    const float relativeCollisionPointBPPerpendicularDotN = relativeCollisionPointBP.x * collisionNormal.y -
                                                            relativeCollisionPointBP.y * collisionNormal.x;
    const Vector2F relativeVelocityAB =
        (rigidBodyB->velocity + rigidBodyB->angularVelocity * relativeCollisionPointBPPerpendicularDotN) -
            (rigidBodyA->velocity + rigidBodyA->angularVelocity * relativeCollisionPointAPPerpendicularDotN);
    const float impulseMagnitude = -(1 + elasticity) * relativeVelocityAB.dot(collisionNormal) /
                                   (
                                       rigidBodyA->inverseMass + rigidBodyB->inverseMass +
                                       powf(relativeCollisionPointAPPerpendicularDotN, 2) /
                                       rigidBodyA->getMomentOfInertia() +
                                       powf(relativeCollisionPointBPPerpendicularDotN, 2) /
                                       rigidBodyB->getMomentOfInertia()
                                    );
    const Vector2F impulseVector = collisionNormal * impulseMagnitude;

    // Apply the impulse to the velocities
    rigidBodyA->angularVelocity -= (relativeCollisionPointAPPerpendicularDotN * impulseMagnitude) /
                                    rigidBodyA->getMomentOfInertia();
    rigidBodyB->angularVelocity += (relativeCollisionPointBPPerpendicularDotN * impulseMagnitude) /
                                    rigidBodyB->getMomentOfInertia();

    rigidBodyA->velocity -= impulseVector * rigidBodyA->inverseMass;
    rigidBodyB->velocity += impulseVector * rigidBodyB->inverseMass;

    // Calculate how much each RigidBody will be moved to negate the collision based off each RigidBody's velocity
    const float totalVelocityMagnitude = rigidBodyA->velocity.getMagnitude() + rigidBodyB->velocity.getMagnitude();
    const float velocityProportionA = (totalVelocityMagnitude != 0) ? rigidBodyA->velocity.getMagnitude() /
                                                                      totalVelocityMagnitude : 0.5f;
    const float velocityProportionB = (totalVelocityMagnitude != 0) ? rigidBodyB->velocity.getMagnitude() /
                                                                      totalVelocityMagnitude : 0.5f;
    // Adjust the positions to negate collision and respond to it
    rigidBodyA->centerOfMass += collisionInfo.penetrationVector * velocityProportionA;
    rigidBodyB->centerOfMass -= collisionInfo.penetrationVector * velocityProportionB;

    return true;
}

Vector2F PhysicsEngine::calculateCentroid(const std::vector<Vector2F>& polygonVertices) {
    if (polygonVertices.size() < 3) {
        throw EngineException("Invalid Polygon", "Centroid calculation requires at least 3 polygonVertices.");
    }

    Vector2F centroid{};
    float signedArea = 0.0f;
    size_t numVertices = polygonVertices.size();

    for (size_t i = 0; i < numVertices; ++i) {
        size_t j = (i + 1) % numVertices;

        float A = polygonVertices[i].x * polygonVertices[j].y - polygonVertices[j].x * polygonVertices[i].y;
        signedArea += A;
        centroid.x += (polygonVertices[i].x + polygonVertices[j].x) * A;
        centroid.y += (polygonVertices[i].y + polygonVertices[j].y) * A;
    }

    if (std::abs(signedArea) < std::numeric_limits<float>::epsilon()) {
        throw EngineException("Degenerate Polygon", "Polygon has zero or near-zero area.");
    }

    centroid /= (3.0f * signedArea);
    return centroid;
}

float PhysicsEngine::calculateMomentOfInertia(const std::vector<Vector2F>& polygonVertices, float inverseMass) {
    if (polygonVertices.size() < 3) {
        throw EngineException("Invalid Polygon", "Moment of inertia calculation requires at least 3 vertices.");
    }
    if (inverseMass == 0) return std::numeric_limits<float>::max();

    float area{};
    float momentOfInertiaAtP1{};

    // Loop through all the points in the collision mesh except the first and last to separate it into triangles
    for (auto it = polygonVertices.begin() + 1; it != polygonVertices.end() - 1; ++it) {

        // Get the points of the triangle
        Vector2F p1 = *it; // Current vertex
        Vector2F p2 = *std::next(it); // Next vertex
        Vector2F p3 = *polygonVertices.begin(); // First vertex (reference)

        // Get the vectors of the triangle
        Vector2F v1 = p2 - p1;
        Vector2F v2 = p3 - p1;

        const float width = v1.getMagnitude();
        const float height = v1.cross(v2) / width;

        // Project v2 onto v1 to split the width into two, making two right-angled triangles, A (p1, p4, p3) and B (p4, p2, p3)
        Vector2F p4 = p1 + v1 * (v2.dot(v1) / powf(width, 2));
        const float widthA = p1.getDistanceTo(p4);
        const float widthB = p2.getDistanceTo(p4);

        // Calculate areas of the right-angled triangles
        const float areaA = (widthA * height) / 2;
        const float areaB = (widthB * height) / 2;

        // Calculate the moment of inertia at
        auto calculateMomentOfInertiaOfRightAngledTriangle = [&
                ](const float triangleWidth, const float triangleHeight) {
            return ((triangleWidth * powf(triangleHeight, 3)) / 4 +
                   (powf(triangleWidth, 3) * triangleHeight) / 12);
        };

        // Decide whether to add or subtract the moments of inertia on whether the triangle is negative or positive space
        if ((p1 - p3).cross(p4 - p3) > 0) {
            momentOfInertiaAtP1 += calculateMomentOfInertiaOfRightAngledTriangle(widthA, height);
            area += areaA;
        } else {
            momentOfInertiaAtP1 -= calculateMomentOfInertiaOfRightAngledTriangle(widthA, height);
            area -= areaA;
        }
        if ((p4 - p3).cross(p2 - p3) > 0) {
            momentOfInertiaAtP1 += calculateMomentOfInertiaOfRightAngledTriangle(widthB, height);
            area += areaB;
        } else {
            momentOfInertiaAtP1 -= calculateMomentOfInertiaOfRightAngledTriangle(widthB, height);
            area -= areaB;
        }
    }
    momentOfInertiaAtP1 = std::abs(momentOfInertiaAtP1 * ((1/inverseMass) / area));

    // Get the moment of inertia at the center of inverseMass using parallel axis theorem and return it
    return momentOfInertiaAtP1 - polygonVertices[0].getDistanceSquaredTo(Vector2F()) / inverseMass;
}

void PhysicsEngine::setGravity(float gravityValue) { gravity = Vector2F(0, gravityValue); }

void PhysicsEngine::setGravity(Vector2F gravityValue) { gravity = gravityValue; }

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
