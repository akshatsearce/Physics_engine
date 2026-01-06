#ifndef BOX_H
#define BOX_H

#include "Shape.h"

class Box : public Shape{

    public:
    float width, height;
    float halfWidth, halfHeight;

    Box(float w, float h): Shape(ShapeType::BOX){
        width = w;
        height = h;
        halfWidth = w*0.5f;
        halfHeight = h*0.5f;
    }

};

#endif