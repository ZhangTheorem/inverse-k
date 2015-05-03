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

//****************************************************
// Global Variables
//****************************************************

int windowWidth = 700;
int windowHeight = 700;
int windowID;
unsigned int mode = GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH;;

float lpos[] = {1000, 1000, 1000, 0};

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

    glFlush();
    glutSwapBuffers();
}

void reshape(int w, int h) {
    float width = (w == 0) ? 1.0 : (float) w;
    float height = (h == 0) ? 1.0 : (float) h;

    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(fmin(-width/height, -1.0), fmax(width/height, 1.0),
            fmin(-height/width, -1.0), fmax(height/width, 1.0),
            0.01, 20.0);

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
