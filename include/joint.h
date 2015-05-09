#ifndef JOINT_H
#define JOINT_H
#endif

#ifndef VECTOR_H
#include "vector.h"
#endif

//****************************************************
// Joint
//****************************************************

class Joint {
    public:
        // Instance variables
        float length;
        Vector rotation;

        // Constructors
        Joint();
        Joint(float, Vector);
};