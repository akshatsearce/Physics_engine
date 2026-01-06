#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Vec2.h"
#include "Shape.h"

class RigidBody
{

public:
    Vec2 position;
    Vec2 velocity;
    Vec2 force;

    float rotation;
    float angularVelocity;

    float mass;
    float inverseMass;
    float restitution;
    float friction;

    Shape* shape;

    RigidBody(Shape* shape, float x, float y, float mass, float restitution, float friction = 0.1f);

    void ApplyForce(const Vec2& f);
    void Integrate(float dt);

};


#endif