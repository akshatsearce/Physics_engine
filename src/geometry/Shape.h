#ifndef SHAPE_H
#define SHAPE_H

enum ShapeType{
    CIRCLE,
    BOX,
    TRIANGLE
};

class Shape{
    public:

    ShapeType type;
    Shape(ShapeType t){
        type =t;
    }

    virtual ~Shape(){};
};

#endif