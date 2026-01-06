#include "RigidBody.h"

RigidBody::RigidBody(Shape* shape, float x, float y, float mass, float restitution, float friction) {
    this->shape = shape;
    this->position = Vec2(x, y);
    this->velocity = Vec2(0, 0);
    this->force = Vec2(0, 0);
    this->mass = mass;
    this->restitution = restitution;
    this->friction = friction;

    if (mass != 0.0f) {
        this->inverseMass = 1.0f / mass;
    } else {
        this->inverseMass = 0.0f; // Infinite mass cannot be moved
    }
}

void RigidBody::ApplyForce(const Vec2& f){
    this->force += f;
}

void RigidBody::Integrate(float dt){

    if(this->inverseMass == 0.0f) return;
    // f= ma -> a = f* im
    Vec2 acceleration = force * inverseMass;

    velocity += acceleration * dt;

    position += velocity * dt;

    force = Vec2(0.0f, 0.0f);

}
