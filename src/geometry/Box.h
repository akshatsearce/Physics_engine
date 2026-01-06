#ifndef BOX_H
#define BOX_H

#include "Shape.h"

class Box : public Shape{

    public:
    float breath, height;

    Box(float b, float h): Shape(ShapeType::BOX){
        breath = b;
        height = h;
    }

};

#endif