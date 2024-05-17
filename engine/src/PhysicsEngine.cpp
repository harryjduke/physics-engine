#include "PhysicsEngine.h"

#include "XCube2d.h"

/* PHYSICS OBJECT */

PhysicsObject::PhysicsObject(const Point2f & center, float w, float h, float mass):
    center(center), width(w), height(h), inverseMass(1.f/mass), isColliding{} { }

PhysicsObject::PhysicsObject(const Point2f & center, float w, float h, bool isStatic):
        center(center), width(w), height(h), inverseMass(isStatic ? 0.f : 1.f ), isColliding{} { }

void PhysicsObject::setMass(float mass) { if (mass > 0) inverseMass = 1/mass; }

Rectf PhysicsObject::getRect() const {
    return
    {
        static_cast<float>(center.x) - width/2,
        static_cast<float>(center.y) - height/2,
        width,
        height
    };
}

/* PHYSICS ENGINE */

PhysicsEngine::PhysicsEngine() : gravity(Vector2f(0, DEFAULT_GRAVITY)), doDebug{} { }

void PhysicsEngine::setGravity(float gravityValue) { gravity = Vector2f(0, gravityValue); }
void PhysicsEngine::setGravity(Vector2f gravityValue) { gravity = gravityValue; }

void PhysicsEngine::registerObject(const std::shared_ptr<PhysicsObject>& object) { objects.push_back(object); }


void PhysicsEngine::update(float deltaTime)
{
    for (const auto& object : objects)
    {
        object->isColliding = false;
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

    // After updating all positions, check for collisions
    for (int i = 0; i < objects.size(); ++i)
    {
        for (int j = i + 1; j < objects.size(); ++j)
        {
            if (isColliding(objects[i], objects[j]))
            {
                objects[i]->isColliding = true;
                objects[j]->isColliding = true;
            }
        }
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
            if (object->isColliding) graphicsEngine->setDrawColor(SDL_COLOR_GREEN);
            SDL_Rect debugRect =  object->getRect().getSDLRect();
            graphicsEngine->drawRect(debugRect);

            //i++;
            //printf("Object%d velocity: %f", i, object->velocity.y);
            //std::cout << std::endl;
        }
    }
}

bool PhysicsEngine::isColliding(const std::shared_ptr<PhysicsObject>& object1,
                                const std::shared_ptr<PhysicsObject>& object2)
{
    return object1->getRect().intersects(object2->getRect());
}

void PhysicsEngine::resolveCollision(std::shared_ptr<PhysicsObject> object1, std::shared_ptr<PhysicsObject> object2)
{

}


#endif
