#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>

// Keep these includes the same
#include "Vec2.h"
#include "Circle.h"
#include "RigidBody.h"
#include "Collision.h"

int main() {
    // FIX 1: VideoMode now takes a Vector size ({width, height})
    sf::RenderWindow window(sf::VideoMode({800, 600}), "Physics Engine Demo");
    window.setFramerateLimit(60);

    Vec2 gravity(0.0f, 10.0f);
    float dt = 1.0f / 60.0f;

    std::vector<RigidBody*> bodies;

    // --- Create Floor ---
    Circle* floorShape = new Circle(5000.0f);
    RigidBody* floor = new RigidBody(floorShape, 400.0f, 5550.0f, 0.0f, 0.5f);
    bodies.push_back(floor);

    // --- Create Ball ---
    Circle* ballShape = new Circle(20.0f);
    RigidBody* ball = new RigidBody(ballShape, 400.0f, 100.0f, 5.0f, 0.8f);
    bodies.push_back(ball);

    // --- Create Rock ---
    Circle* rockShape = new Circle(30.0f);
    RigidBody* rock = new RigidBody(rockShape, 300.0f, 50.0f, 20.0f, 0.1f);
    bodies.push_back(rock);

    while (window.isOpen()) {
        // FIX 2: pollEvent works differently in SFML 3.0
        // It returns an "optional", so we check it in the while loop directly
        while (const auto event = window.pollEvent()) {
            
            // FIX 3: Event handling syntax changed
            // We check "is<Type>()" to see what happened
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
            // Check if mouse button was pressed
            else if (const auto* mouse = event->getIf<sf::Event::MouseButtonPressed>()) {
                // Reset ball (Just a fun interaction)
                ball->position = Vec2(400.0f, 100.0f);
                ball->velocity = Vec2(100.0f, -200.0f);
            }
        }

        // --- Physics Update ---
        for (auto b : bodies) {
            if (b->inverseMass != 0.0f) {
                b->ApplyForce(gravity * b->mass);
            }
        }

        for (auto b : bodies) {
            b->Integrate(dt);
        }

        for (size_t i = 0; i < bodies.size(); i++) {
            for (size_t j = i + 1; j < bodies.size(); j++) {
                Manifold m = Collision::CheckCircleCollision(bodies[i], bodies[j]);
                if (m.isColliding) {
                    Collision::ResolveCollision(m);
                    Collision::PositionalCorrection(m);
                }
            }
        }

        // --- Rendering ---
        window.clear(sf::Color::Black);

        for (auto b : bodies) {
            if (b->shape->type == ShapeType::CIRCLE) {
                Circle* c = dynamic_cast<Circle*>(b->shape);
                sf::CircleShape sfShape(c->radius);
                
                // FIX 4: setOrigin and setPosition now require braces {x, y}
                sfShape.setOrigin({c->radius, c->radius});
                sfShape.setPosition({b->position.x, b->position.y});

                if (b->inverseMass == 0.0f) sfShape.setFillColor(sf::Color(100, 100, 100));
                else sfShape.setFillColor(sf::Color::Cyan);

                window.draw(sfShape);
            }
        }

        window.display();
    }

    // Cleanup
    for (auto b : bodies) {
        delete b->shape;
        delete b;
    }

    return 0;
}