#include "joint.h"

//****************************************************
// Joint
//****************************************************

Joint::Joint() {
    length = 0;
    rotation = Vector(1, 0 ,0);
    inboard = outboard = NULL;
}

Joint::Joint(float len, Vector rot, Joint* in, Joint* out) {
    length = len;
    rotation = rot;
    inboard = in;
    outboard = out;
}