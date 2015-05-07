#include "joint.h"

//****************************************************
// Joint
//****************************************************

Joint::Joint() {
    length = 0;
    rotation = Vector(0, 0 ,0);
}

Joint::Joint(float len, Vector rot) {
    length = len;
    rotation = rot;
}