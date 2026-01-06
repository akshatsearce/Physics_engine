#ifndef CIRCLE_H
#define CIRCLE_H

#include "Shape.h"

class Circle : public Shape{
    public:
        float radius;

    Circle(float r): Shape(ShapeType::CIRCLE){
        radius = r;
    }
};


#endif