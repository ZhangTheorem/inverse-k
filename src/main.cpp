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

#include <Eigen3/Eigen/Core>
#include <Eigen3/Eigen/SVD>

#define _USE_MATH_DEFINES
#include <math.h>

#ifndef JOINT_H
#include "joint.h"
#endif

#ifndef VECTOR_H
#include "vector.h"
#endif

using namespace Eigen;

//****************************************************
// Global Variables
//****************************************************

int windowWidth = 1440;
int windowHeight = 900;
int windowID;
unsigned int mode = GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH;;

float lpos[] = {1000, 1000, 1000, 0};

std::vector<Joint> jointlist;
std::vector<Joint> jointlist2;
std::vector<Joint> newjoints;
Vector systemend;
Vector newend;
Vector goal;
Vector fakeGoal;
float stepsize = 1;
float t = 0;
bool test = true;

//****************************************************
// Inverse Kinematics Solver
//****************************************************

Vector goalFunction(float t) {
    //return Vector(1.25 + 1.25*cos(t), 1.25*sin(t), 2.5*sin(t/2));
    //return Vector(3 * sin(t), 3*sin(t)*cos(t), 0);
    //return Vector(1.5 + 1.5*cos(t), 1.5*sin(t), 3*sin(t/2));
    //return Vector(1 + cos(t), sin(t), 2*sin(t/2));
    //return Vector(2*sin(t/2) - 1, cos(t), sin(t) - 1);
    //return Vector(sin(t), 2*sin(t/2), 1 + cos(t));
    //return Vector(3.5*cos(t), 3.5*sin(t), 0);
    //return Vector(t,t,t);
    //return Vector(0,0,0);
}

MatrixXf pseudoinvert(MatrixXf m) {
    JacobiSVD<MatrixXf> svd(m, ComputeThinU | ComputeThinV);

    float error = 1e-7;
    MatrixXf matrixS;
    matrixS = svd.singularValues().asDiagonal();

    for (int i = 0; i < m.cols() && i < m.rows(); i++) {
        if (matrixS(i, i) > error) {
            matrixS(i, i) = 1.0 / matrixS(i, i);
        }
    }

    MatrixXf inv;
    inv = svd.matrixV() * matrixS * svd.matrixU().transpose();
    return inv;
}

MatrixXf rotation_matrix(Vector rot) {
    MatrixXf rtn(3, 3);
    rtn << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;

    float angle = rot.len();
    Vector rotaxis = rot.normalize();

    MatrixXf K(3, 3);
    K << 0, -rotaxis.z, rotaxis.y,
         rotaxis.z, 0, -rotaxis.x,
         -rotaxis.y, rotaxis.x, 0;

    rtn += sin(angle) * K;
    rtn += (1 - cos(angle)) * K * K;

    return rtn;
}

Vector3f end_effector(std::vector<Joint> list) {
    int numJoints = list.size();
    Vector3f end(0, 0, 0);
    for (int i = numJoints - 1; i >= 0; i--) {
        Joint current = list.at(i);
        Vector3f translate(current.length, 0, 0);
        end += translate;
        end = rotation_matrix(current.rotation) * end;
    }
    return end;
}

Vector3f jacobian_column(std::vector<Joint> oldList, int k) {
    int numJoints = oldList.size();
    float h = 0.0001;
    std::vector<Joint> newList;
    for (int i = 0; i < numJoints; i++) {
        newList.push_back(oldList.at(i));
    }
    if (k % 3 == 0) {
        newList[k / 3].rotation.x += h;
    } else if (k % 3 == 1) {
        newList[k / 3].rotation.y += h;
    } else {
        newList[k / 3].rotation.z += h;
    }
    Vector3f oldEnd = end_effector(oldList);
    Vector3f shiftEnd = end_effector(newList);
    return (shiftEnd - oldEnd) / h;
}

MatrixXf numeric_jacobian(std::vector<Joint> list) {
    int numJoints = list.size();
    MatrixXf jacobian(3, 3 * numJoints);

    for (int i = 0; i < 3 * numJoints; i ++) {
        Vector3f column = jacobian_column(list, i);
        jacobian(0, i) = column(0);
        jacobian(1, i) = column(1);
        jacobian(2, i) = column(2);
    }

    return jacobian;
}

MatrixXf analytic_jacobian(std::vector<Joint> list) {
    int numJoints = list.size();
    MatrixXf jacobian(3, 0);

    for (int i = 0; i < numJoints; i++) {
        Vector3f localp(0, 0, 0);
        for (int j = numJoints - 1; j >= i; j--) {
            Joint inter = list.at(j);
            Vector3f translate(inter.length, 0, 0);
            localp += translate;
            localp = rotation_matrix(inter.rotation) * localp;
        }
        MatrixXf localjacobian(3, 3);
        localjacobian << 0, -localp(2), localp(1),
                         localp(2), 0, -localp(0),
                         -localp(1), localp(0), 0;
        for (int k = i - 1; k >= 0; k--) {
            Joint prev = list.at(k);
            localjacobian = rotation_matrix(prev.rotation) * localjacobian;
        }
        localjacobian *= -1.0;
        // for (int l = 0; l < 3; l++) {
        //     for (int m = 0; m < 3; m++) {
        //         if (localjacobian(l, m) != 0) {
        //             localjacobian(l, m) *= -1;
        //         }
        //     }
        // }
        MatrixXf temp(3, jacobian.cols() + 3);
        temp << jacobian, localjacobian;
        jacobian = temp;
    }

    return jacobian;
}

void update_system(std::vector<Joint> list) {
    int numJoints = list.size();
    Vector diff = (fakeGoal - systemend) * stepsize;
    // diff = diff.normalize() * stepsize;
    Vector3f diff_c(diff.x, diff.y, diff.z);
    // MatrixXf jacobian(3, 0);

    // for (int i = 0; i < numJoints; i++) {
    //     Vector3f localp(0, 0, 0);
    //     for (int j = numJoints - 1; j >= i; j--) {
    //         Joint inter = list.at(j);
    //         Vector3f translate(inter.length, 0, 0);
    //         localp += translate;
    //         localp = rotation_matrix(inter.rotation) * localp;
    //     }
    //     MatrixXf localjacobian(3, 3);
    //     localjacobian << 0, -localp(2), localp(1),
    //                      localp(2), 0, -localp(0),
    //                      -localp(1), localp(0), 0;
    //     for (int k = i - 1; k >= 0; k--) {
    //         Joint prev = list.at(k);
    //         localjacobian = rotation_matrix(prev.rotation) * localjacobian;
    //     }
    //     localjacobian *= -1.0;
    //     // for (int l = 0; l < 3; l++) {
    //     //     for (int m = 0; m < 3; m++) {
    //     //         if (localjacobian(l, m) != 0) {
    //     //             localjacobian(l, m) *= -1;
    //     //         }
    //     //     }
    //     // }
    //     MatrixXf temp(3, jacobian.cols() + 3);
    //     temp << jacobian, localjacobian;
    //     jacobian = temp;
    // }

    MatrixXf jacobian = numeric_jacobian(list);

    // if (test)
    //     std::cout << jacobian << std::endl;

    // test = false;

    // MatrixXf pseudo;
    // pseudo = pseudoinvert(jacobian);

    JacobiSVD<MatrixXf> svd(jacobian, ComputeThinU | ComputeThinV);

    MatrixXf dr;
    // dr = pseudo * diff_c;

    dr = svd.solve(diff_c);

    for (int i = 0; i < numJoints; i++) {
        Joint newjoint = list.at(i);
        newjoint.rotation += Vector(dr(i*3, 0), dr(i*3 + 1, 0), dr(i*3 + 2, 0));
        // if ((newjoint.rotation.len() - M_PI) < 1e-7) {
        //     float reparam = (1 - 2 * M_PI) / newjoint.rotation.len();
        //     newjoint.rotation = reparam * newjoint.rotation;
        // }
        newjoints[i] = newjoint;
    }

    Vector3f tempend(0, 0, 0);
    for (int i = numJoints - 1; i >= 0; i--) {
        Joint current = newjoints.at(i);
        Vector3f translate(current.length, 0, 0);
        tempend += translate;
        tempend = rotation_matrix(current.rotation) * tempend;
    }
    newend = Vector(tempend(0), tempend(1), tempend(2));
}

//****************************************************
// OpenGL Functions
//****************************************************

void junk() {
    int i;
    i = pthread_getconcurrency();
    i++;
};

void init(){
    int numJoints = jointlist.size();
    Vector3f tempend(0, 0, 0);
    for (int i = numJoints - 1; i >= 0; i--) {
        Joint current = jointlist.at(i);
        newjoints.push_back(current);
        Vector3f translate(current.length, 0, 0);
        tempend += translate;
        tempend = rotation_matrix(current.rotation) * tempend;
    }
    systemend = Vector(tempend(0), tempend(1), tempend(2));
    goal = goalFunction(t);

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
    gluLookAt(0, 0, -5,
            0, 0, 0,
            0, 1, 0);

    int numJoints = jointlist.size();

    int iter = 0;
    float wholething = 0.0;

    for(int i = 0; i < numJoints; i++){
        wholething += jointlist.at(i).length;
    }
    if(goal.len() > wholething){
        fakeGoal = goal.normalize() * wholething;
    }
    else{
        fakeGoal = goal;
    }

    while ((systemend - fakeGoal).len() > 1e-2) {
        iter++;
        float olddiff = (systemend - fakeGoal).len();
        update_system(jointlist);
        if ((newend - fakeGoal).len() >= olddiff) {
            stepsize /= 2;
        } else {
            for (int i = 0; i < numJoints; i++) {
                jointlist[i] = newjoints.at(i);
                systemend = newend;
            }
        }
        if (stepsize < 1e-2) {
            for (int i = 0; i < numJoints; i++) {
                jointlist[i] = newjoints.at(i);
                systemend = newend;
            }
            stepsize = 1;
            break;
        }
    }
    stepsize = 1;
    //std::cout << iter << std::endl;

    for (int i = 0; i < numJoints; i++) {
        Joint current = jointlist.at(i);
        Vector rot = current.rotation;
        float angle = rot.len() * 180 / M_PI;
        glPushMatrix();
            for (int j = 0; j < i; j++) {
                Joint prev = jointlist.at(j);
                Vector prevrot = prev.rotation;
                float prevangle = prevrot.len() * 180 / M_PI;
                glRotatef(prevangle, prevrot.x, prevrot.y, prevrot.z);
                glTranslatef(prev.length, 0, 0);
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

    // for (int i = 0; i < numJoints; i++) {
    //     Joint current = jointlist2.at(i);
    //     Vector rot = current.rotation;
    //     float angle = rot.len() * 180 / M_PI;
    //     glPushMatrix();
    //         for (int j = 0; j < i; j++) {
    //             Joint prev = jointlist2.at(j);
    //             Vector prevrot = prev.rotation;
    //             float prevangle = prevrot.len() * 180 / M_PI;
    //             glRotatef(prevangle, prevrot.x, prevrot.y, prevrot.z);
    //             glTranslatef(prev.length, 0, 0);
    //         }
    //         glRotatef(angle, rot.x, rot.y, rot.z);
    //         glPushMatrix();
    //             glTranslatef(-0.5, 0, 0);
    //             glRotatef(-90, 0, 1, 0);
    //             glColor3f(1, 1, 1);
    //             gluCylinder(gluNewQuadric(), 0.1, 0.1, current.length - 0.2, 50, 50);
    //         glPopMatrix();
    //         glColor3f(1, 0, 0);
    //         glutSolidSphere(0.1, 50, 50);
    //     glPopMatrix();
    // }

    // glPushMatrix();
    // glTranslatef(goal.x, goal.y, goal.z);
    // glColor3f(1, 1, 0);
    // glutSolidSphere(0.1, 50, 50);
    // glPopMatrix();

    Vector frozenGoal;
    for(float i = 0.0; i <= 4 * M_PI; i += M_PI / 18){
        frozenGoal = goalFunction(i);
        glPushMatrix();
        glTranslatef(frozenGoal.x, frozenGoal.y, frozenGoal.z);
        glColor3f(1, 1, 0);
        glutSolidSphere(0.03, 50, 50);
        glPopMatrix();
    }


    t += 0.01;
    goal = goalFunction(t);

    glFlush();
    glutSwapBuffers();
}

void reshape(int w, int h) {
    float height = (h == 0) ? 1.0 : (float) h;

    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(65, w / height, 1, 1000);

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
    Joint root = Joint(1, Vector());
    Joint arm1 = Joint(0.75, Vector());
    Joint arm2 = Joint(0.5, Vector());
    Joint arm3 = Joint(0.25, Vector());
    jointlist.push_back(root);
    jointlist.push_back(arm1);
    jointlist.push_back(arm2);
    jointlist.push_back(arm3);

    // Joint root2 = Joint(1.5, Vector());
    // Joint arm12 = Joint(1, Vector());
    // Joint arm22 = Joint(0.75, Vector());
    // Joint arm32 = Joint(0.5, Vector());
    // jointlist2.push_back(root2);
    // jointlist2.push_back(arm12);
    // jointlist2.push_back(arm22);
    // jointlist2.push_back(arm32);


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
