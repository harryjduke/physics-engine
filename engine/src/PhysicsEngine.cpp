#include "PhysicsEngine.h"

#include "XCube2d.h"

/* PHYSICS OBJECT */

PhysicsObject::PhysicsObject(const Point2 & center, float w, float h, float mass):
    center(center), width(w), height(h), inverseMass(1.f/mass) { }

PhysicsObject::PhysicsObject(const Point2 & center, float w, float h, bool isStatic):
        center(center), width(w), height(h), inverseMass(isStatic ? 0.f : 1.f ) { }

void PhysicsObject::setMass(float mass) { inverseMass = 1/mass; }

//bool PhysicsObject::isColliding(const PhysicsObject & other) const
//{
//    Rectf r1 = { center.x - hlX, center.y - hlY, lX, lY };
//    Rectf r2 = { other.center.x - other.hlX, other.center.y - other.hlY, other.lX, other.lY };
//
//    return r1.intersects(r2);
//}

/* PHYSICS ENGINE */

PhysicsEngine::PhysicsEngine() : gravity(Vector2f(0, DEFAULT_GRAVITY)) { }

void PhysicsEngine::setGravity(float gravityValue) { gravity = Vector2f(0, gravityValue); }
void PhysicsEngine::setGravity(Vector2f gravityValue) { gravity = gravityValue; }

void PhysicsEngine::registerObject(const std::shared_ptr<PhysicsObject>& obj) { objects.push_back(obj); }


void PhysicsEngine::update(float deltaTime)
{
    for (const auto& object : objects)
    {
        // Calculate forces
        Vector2f force{};

        // Apply acceleration (a = f/m)
        object->acceleration = force * object->inverseMass;
        // Add gravity (as gravity is a constant acceleration, we don't need to calculate the force)
        if (object->inverseMass != 0) // If the object is not static
            object->acceleration += gravity;

        // Update velocity (v = u + at)
        object->velocity += object->acceleration * deltaTime;
        // Update position (s = ut)
        object->center += object->velocity * deltaTime;
    }
}

#ifdef __DEBUG
void PhysicsEngine::drawDebug()
{
    if (doDebug)
    {
        auto graphicsEngine =  XCube2Engine::getInstance()->getGraphicsEngine();
        graphicsEngine->setDrawColor(SDL_COLOR_RED);

        //int i = 0;
        for (const auto& object : objects)
        {
            SDL_Rect debugRect;
            debugRect.x = object->center.x - object->width/2;
            debugRect.y = object->center.y - object->height/2;
            debugRect.w = object->width;
            debugRect.h = object->height;
            graphicsEngine->drawRect(debugRect);

            //i++;
            //printf("Object%d velocity: %f", i, object->velocity.y);
            //std::cout << std::endl;
        }
    }
}
#endif
