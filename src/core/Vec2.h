#ifndef VEC2_H
#define VEC2_H

#include <cmath>

struct Vec2
{
    float x,y;

    Vec2(){
        this->x=0.0f;
        this->y=0.0f;
    }

    Vec2(float x, float y){
        this->x=x;
        this->y=y;
    }

    Vec2 operator + (const Vec2& v) const{
        return Vec2(x + v.x , y + v.y);
    }

    Vec2 operator - (const Vec2& v) const{
        return Vec2(x- v.x, y-v.y);
    }

    Vec2 operator * (const float& c) const{
        return Vec2(x*c, y*c);
    }

    Vec2 operator / (const float& c) const{
        return Vec2(x/c, y/c);
    }

    void operator += (const Vec2& v){
        this->x += v.x;
        this->y += v.y;
    }

    void operator -= (const Vec2& v){
        this->x -= v.x;
        this->y -= v.y;
    }

    static float DotProduct(const Vec2& a, const Vec2& b){
        return (a.x* b.x) + (a.y * b.y);
    }

    float LenghtSquared() const{
        return x*x + y*y;
    }

    float Lenght() const{
        return std::sqrt(x*x + y*y);
    }

};



#endif