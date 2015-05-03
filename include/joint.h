#ifndef JOINT_H
#define JOINT_H
#endif

#ifndef VECTOR_H
#include "vector.h"
#endif

//****************************************************
// Link
//****************************************************

class Joint {
    public:
        // Instance variables
        float length;
        Vector rotation;
        Joint inboard, outboard;

        // Constructors
        Joint();
        Joint(float);
}