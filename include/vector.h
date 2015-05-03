#ifndef VECTOR_H
#define VECTOR_H
#endif

#include <math.h>
#include <stdio.h>

//****************************************************
// Vector
//****************************************************

class Vector {
    public:
        float x, y, z;
        float len();
        Vector normalize();
        void clamp();

        Vector& operator+=(const Vector&);
        Vector& operator-=(const Vector&);
        Vector& operator*=(const float);
        Vector& operator/=(const float);
        
        static void print(Vector v);
        static Vector cross(Vector a, Vector b);
        static float dot(Vector a, Vector b);
        static Vector point_multiply(Vector a, Vector b);
        static bool equal(Vector, Vector);
        
        Vector();
        Vector(float, float, float); 
};

Vector operator+(Vector, const Vector&);
Vector operator-(Vector, const Vector&);
Vector operator*(Vector, const float);
Vector operator*(const float, Vector&);
Vector operator/(Vector, const float);