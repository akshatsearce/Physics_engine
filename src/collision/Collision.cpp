#include "Collision.h"
#include "Circle.h"
#include <algorithm>
#include <cmath>
#include <iostream>

Manifold Collision::CheckCircleCollision(RigidBody* bodyA, RigidBody* bodyB){

    Circle* cA = dynamic_cast<Circle*>(bodyA->shape);
    Circle* cB = dynamic_cast<Circle*>(bodyB->shape);

    Vec2 normal = bodyB->position - bodyA->position;

    float distanceBetween = normal.Lenght();
    float sumRadius = cA->radius + cB->radius;

    bool isColliding = false;

    if(distanceBetween <= sumRadius) {
        isColliding = true;
    }

    Manifold m;
    m.bodyA = bodyA;
    m.bodyB = bodyB;
    m.isColliding = isColliding;

    if (distanceBetween != 0) {
        m.penetration = sumRadius - distanceBetween;
        m.normal = normal / distanceBetween; 
    } else {
        m.penetration = sumRadius;
        m.normal = Vec2(1, 0); 
    }

    return m;

}

void Collision::ResolveCollision(const Manifold& m){

    RigidBody* a = m.bodyA;
    RigidBody* b = m.bodyB;

    Vec2 vr = b->velocity - a->velocity;

    float velAlongNormal = Vec2::DotProduct(vr, m.normal);

    if(velAlongNormal > 0 ) return;

    float e = std::min(a->restitution , b->restitution);

    float j = -(1+e)*velAlongNormal;
    j /= (a->inverseMass + b->inverseMass); 

    Vec2 impluse = m.normal*j;

    a->velocity -= impluse*a->inverseMass;
    b->velocity += impluse*b->inverseMass;

};

void Collision::PositionalCorrection(const Manifold& m){

    const float percent = 0.2f;
    const float slop = 0.01f;

    RigidBody* A = m.bodyA;
    RigidBody* B = m.bodyB;

    float invMassA = A->inverseMass;
    float invMassB = B->inverseMass;

    if (invMassA + invMassB == 0) return;

    float depth = std::max(m.penetration - slop, 0.0f);
    Vec2 correction = m.normal * (depth / (invMassA + invMassB)) * percent;

    A->position -= correction * invMassA;
    B->position += correction * invMassB;

}