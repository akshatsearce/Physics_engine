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

    Shape* shape;

    RigidBody(Shape* shape, float x, float y, float mass, float restitution);

    void ApplyForce(const Vec2& f);
    void Integrate(float dt);

};


#endif