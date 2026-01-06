#include "Collision.h"
#include "Circle.h"
#include "Box.h"
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



Manifold Collision::CheckBoxCollision(RigidBody* bodyA, RigidBody* bodyB){

    Manifold m;
    m.bodyA = bodyA;
    m.bodyB = bodyB;

    m.isColliding = false;
    
    Box* boxA = dynamic_cast<Box*>(bodyA->shape);
    Box* boxB = dynamic_cast<Box*>(bodyB->shape);

    Vec2 normal = bodyB->position - bodyA->position;

    float x_extend = boxA->halfWidth + boxB->halfWidth;
    float x_overlap = x_extend - std::abs(normal.x);

    if(x_overlap > 0){

        float y_extend = boxA->halfHeight + boxB->halfHeight;
        float y_overlap = y_extend - std::abs(normal.y);

        if(y_overlap >0){

            if (x_overlap < y_overlap) {
                if (normal.x < 0)
                    m.normal = Vec2(-1, 0);
                else
                    m.normal = Vec2(1, 0);
                
                m.penetration = x_overlap;
            } else {
                if (normal.y < 0)
                    m.normal = Vec2(0, -1);
                else
                    m.normal = Vec2(0, 1);
                    
                m.penetration = y_overlap;
            }

            m.isColliding = true;
        }

    }
    return m;
}


Manifold Collision::CheckBoxCircleCollision(RigidBody* boxBody, RigidBody* circleBody){

    Manifold m;
    m.bodyA = boxBody;
    m.bodyB = circleBody;
    m.isColliding = false;

    Box* box = dynamic_cast<Box*>(boxBody->shape);
    Circle* circle = dynamic_cast<Circle*>(circleBody->shape);

    // position of circle relative to box center (in box local coords)
    Vec2 local = circleBody->position - boxBody->position;

    Vec2 closest = local;

    closest.x = std::clamp(closest.x , -box->halfWidth , box->halfWidth);
    closest.y = std::clamp(closest.y, -box->halfHeight, box->halfHeight);

    bool inside = false;

        if (local.x == closest.x && local.y == closest.y) {
        inside = true;
           if (std::abs(local.x) > std::abs(local.y)) {
               closest.x = (closest.x > 0) ? box->halfWidth : -box->halfWidth;
           } else {
               closest.y = (closest.y > 0) ? box->halfHeight : -box->halfHeight;
           }
    }

        Vec2 normal = local - closest;
        float d = normal.LenghtSquared();
        float r = circle->radius;

    if (d > r * r && !inside) {
        return m;
    }

    d = std::sqrt(d);

    if (inside) {
        m.penetration = r + d;
        // ensure normal is set even if distance is zero
        if (d > 0.000001f) m.normal = normal / d;
        else m.normal = Vec2(1, 0);
    } else {
        m.normal = normal / d;
        m.penetration = r - d;
    }

    m.isColliding = true;
    return m;

}


void Collision::ResolveCollision(const Manifold& m){

    RigidBody* A = m.bodyA;
    RigidBody* B = m.bodyB;

    Vec2 vr = B->velocity - A->velocity;

    float velAlongNormal = Vec2::DotProduct(vr, m.normal);

    if(velAlongNormal > 0 ) return;

    float e = std::min(A->restitution , B->restitution);

    float j = -(1+e)*velAlongNormal;
    j /= (A->inverseMass + B->inverseMass); 

    Vec2 impluse = m.normal*j;

    A->velocity -= impluse*A->inverseMass;
    B->velocity += impluse*B->inverseMass;

    vr = B->velocity - A->velocity;
    Vec2 tangent = vr - (m.normal*(Vec2::DotProduct(vr, m.normal)));

    if (tangent.LenghtSquared() > 0.0001f) {
        tangent = tangent / tangent.Lenght();
    } else {
        return; 
    }

    float jt = -Vec2::DotProduct(vr, tangent);
    jt /= (A->inverseMass + B->inverseMass);

    float mu = std::sqrt(A->friction * A->friction + B->friction * B->friction);

    Vec2 frictionImpulse;
    if (std::abs(jt) < j * mu) {
        frictionImpulse = tangent * jt;
    } else {
        frictionImpulse = tangent * -j * mu; 
    }

    A->velocity -= frictionImpulse * A->inverseMass;
    B->velocity += frictionImpulse * B->inverseMass;

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
