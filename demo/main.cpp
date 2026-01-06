#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>

// Include your headers
#include "Vec2.h"
#include "Circle.h"
#include "Box.h"  
#include "RigidBody.h"
#include "Collision.h"

int main() {
    sf::RenderWindow window(sf::VideoMode({800, 600}), "Box Stacking Demo");
    window.setFramerateLimit(60);

    Vec2 gravity(0.0f, 500.0f);
    float dt = 1.0f / 60.0f;

    std::vector<RigidBody*> bodies;

    // 1. Create the Floor (A large static BOX)
    // Width: 800, Height: 50. Position: Bottom center (400, 580)
    Box* floorShape = new Box(800.0f, 50.0f);
    RigidBody* floor = new RigidBody(floorShape, 400.0f, 580.0f, 0.0f, 0.5f);
    bodies.push_back(floor);

    // 2. Create a Stack of Boxes
    // We create 5 boxes, one above the other
    for (int i = 0; i < 5; i++) {
        Box* boxShape = new Box(50.0f, 50.0f); // 50x50 squares
        
        // Position them higher and higher (Y decreases as we go up)
        // Y = 500, 440, 380... (leaving some gap to let them fall)
        float startY = 500.0f - (i * 60.0f);
        
        RigidBody* box = new RigidBody(boxShape, 400.0f, startY, 10.0f, 0.2f);
        bodies.push_back(box);
    }

    while (window.isOpen()) {
        while (const auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
            // Reset on Click: Throw a box at the stack!
            else if (const auto* mouse = event->getIf<sf::Event::MouseButtonPressed>()) {
                // Create a CIRCLE projectile
                Circle* projectileShape = new Circle(20.0f);
                RigidBody* projectile = new RigidBody(projectileShape, 50.0f, 300.0f, 5.0f, 0.8f);
                
                // Shoot it at the box stack
                projectile->velocity = Vec2(500.0f, -100.0f);
                bodies.push_back(projectile);
            }
        }

        // --- Physics Step ---
        for (auto b : bodies) {
            if (b->inverseMass != 0.0f) {
                b->ApplyForce(gravity * b->mass);
            }
            b->Integrate(dt);
        }

        // --- Collision Step ---
        for (int k = 0; k < 10; k++) { 
            
            for (size_t i = 0; i < bodies.size(); i++) {
                for (size_t j = i + 1; j < bodies.size(); j++) {
                    
                    RigidBody* A = bodies[i];
                    RigidBody* B = bodies[j];
                    Manifold m;

                    // Check collisions
                    if (A->shape->type == ShapeType::BOX && B->shape->type == ShapeType::BOX) {
                        m = Collision::CheckBoxCollision(A, B);
                    }
                    else if (A->shape->type == ShapeType::CIRCLE && B->shape->type == ShapeType::CIRCLE) {
                        m = Collision::CheckCircleCollision(A, B);
                    }
                    else if (A->shape->type == ShapeType::BOX && B->shape->type == ShapeType::CIRCLE) {
                        m = Collision::CheckBoxCircleCollision(A, B);
                    }
                    else if (A->shape->type == ShapeType::CIRCLE && B->shape->type == ShapeType::BOX) {
                        m = Collision::CheckBoxCircleCollision(B, A);
                        m.normal = m.normal * -1.0f;
                    }

                    if (m.isColliding) {
                        Collision::ResolveCollision(m);
                        Collision::PositionalCorrection(m);
                    }
                }
            }
        }

        // --- Render Step ---
        window.clear(sf::Color::Black);

        for (auto b : bodies) {
            // Draw BOXES
            if (b->shape->type == ShapeType::BOX) {
                Box* box = dynamic_cast<Box*>(b->shape);
                sf::RectangleShape sfShape({box->width, box->height});
                
                sfShape.setOrigin({box->halfWidth, box->halfHeight});
                sfShape.setPosition({b->position.x, b->position.y});

                // Color logic
                if (b->inverseMass == 0.0f) sfShape.setFillColor(sf::Color(100, 100, 100)); // Grey Floor
                else sfShape.setFillColor(sf::Color::Red); // Red Stack

                window.draw(sfShape);
            }
            // Draw CIRCLES (if you added any)
            else if (b->shape->type == ShapeType::CIRCLE) {
                Circle* c = dynamic_cast<Circle*>(b->shape);
                sf::CircleShape sfShape(c->radius);
                sfShape.setOrigin({c->radius, c->radius});
                sfShape.setPosition({b->position.x, b->position.y});
                sfShape.setFillColor(sf::Color::Cyan);
                window.draw(sfShape);
            }
        }

        window.display();
    }

    // Cleanup memory logic here (omitted for brevity)
    return 0;
}