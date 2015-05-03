#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#define _USE_MATH_DEFINES
#include <math.h>

#ifndef JOINT_H
#include "joint.h"
#endif

#ifndef VECTOR_H
#include "vector.h"
#endif

//****************************************************
// Global Variables
//****************************************************

int windowWidth = 700;
int windowHeight = 700;
int windowID;
unsigned int mode = GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH;;

float lpos[] = {1000, 1000, 1000, 0};

std::vector<Joint> jointlist;
Vector goal;

//****************************************************
// OpenGL Functions
//****************************************************

void junk() {
    int i;
    i = pthread_getconcurrency();
    i++;
};

void init(){
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);

    glLightfv(GL_LIGHT0, GL_POSITION, lpos);
    glLoadIdentity();
}

void display() {
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);    
    glLoadIdentity();
    gluLookAt(0, 0, -3,
            0, 0, 0,
            0, 1, 0);

    int numJoints = jointlist.size();

    for (int i = 0; i < numJoints; i++) {
        Joint current = jointlist.at(i);
        Vector rot = current.rotation;
        float angle = rot.len() * 180 / M_PI;
        Joint* prev = current.inboard;
        Vector prevrot;
        float prevangle;
        glPushMatrix();
            while (prev) {
                prevrot = prev->rotation;
                prevangle = prevrot.len() * 180 / M_PI;
                glRotatef(prevangle, prevrot.x, prevrot.y, prevrot.z);
                glTranslatef(prev->length, 0, 0);
                prev = prev->inboard;
            }
            glRotatef(angle, rot.x, rot.y, rot.z);
            glPushMatrix();
                glTranslatef(0.1, 0, 0);
                glRotatef(90, 0, 1, 0);
                glColor3f(1, 1, 1);
                gluCylinder(gluNewQuadric(), 0.1, 0.1, current.length - 0.2, 50, 50);
            glPopMatrix();
            glColor3f(1, 0, 0);
            glutSolidSphere(0.1, 50, 50);
        glPopMatrix();
    }

    glFlush();
    glutSwapBuffers();
}

void reshape(int w, int h) {
    float height = (h == 0) ? 1.0 : (float) h;

    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90, w / height, 1, 1000);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 27:
            glutDestroyWindow(windowID);
            break;
    }

    if (glutGetWindow())
        glutPostRedisplay();
}

void special(int key, int x, int y) {
    switch (key) {
    }
    glutPostRedisplay();
}

//****************************************************
// Input Parsing & Main
//****************************************************

int main(int argc, char *argv[]) {
    goal = Vector();
    Joint root = Joint(1, Vector(0, 0, M_PI / 2), NULL, NULL);
    Joint arm1 = Joint(2, Vector(), &root, NULL);
    root.outboard = &arm1;
    jointlist.push_back(root);
    jointlist.push_back(arm1);

    glutInit(&argc, argv);
    glutInitDisplayMode(mode);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(0, 0);
    windowID = glutCreateWindow(argv[0]);

    init();
    glutIdleFunc(glutPostRedisplay);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special);    

    glutMainLoop();

    return 0;
}
