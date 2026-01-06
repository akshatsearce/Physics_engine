#ifndef COLLISION_H
#define COLLISION_H

#include "RigidBody.h"


struct Manifold{
    RigidBody* bodyA;
    RigidBody* bodyB;

    Vec2 normal;
    float penetration;
    bool isColliding;

};

class Collision{
    public:

    static Manifold CheckCircleCollision(RigidBody* bodyA, RigidBody* bodyB);

    static Manifold CheckBoxCollision(RigidBody* a, RigidBody* b);

    static void ResolveCollision(const Manifold& m);

    static void PositionalCorrection(const Manifold& m);
};

#endif