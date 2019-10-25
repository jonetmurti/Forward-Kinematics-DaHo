#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace Eigen;
using namespace std;

#define PI 3.14159265

int main () {

    Vector4f P1;
    Vector4f P2;
    Matrix4f Yaw;
    Matrix4f Pitch;
    Matrix4f Roll;

    P1 << 5, 0, 0, 1;
    P2 << 0, 0, 0, 1;

    float a1 = 0;
    float alpha1 = PI/2;
    float d1 = 2;
    float theta1 = (0*PI/180) + (PI/2);

    float a2 = 2;
    float alpha2 = 0;
    float d2 = 0;
    float theta2 = (0*PI/180);

    float a3 = 0;
    float alpha3 = PI/2;
    float d3 = 0;
    float theta3 = (0*PI/180) + (PI/2);

    Yaw(0, 0) = cos(theta1);
    Yaw(0, 1) = (-1)*sin(theta1)*cos(alpha1);
    Yaw(0, 2) = sin(theta1)*sin(alpha1);
    Yaw(0, 3) = a1*cos(theta1);
    Yaw(1, 0) = sin(theta1);
    Yaw(1, 1) = cos(theta1)*cos(alpha1);
    Yaw(1, 2) = (-1)*cos(theta1)*sin(alpha1);
    Yaw(1, 3) = a1*sin(theta1);
    Yaw(2, 0) = 0;
    Yaw(2, 1) = sin(alpha1);
    Yaw(2, 2) = cos(alpha1);
    Yaw(2, 3) = d1;
    Yaw(3, 0) = 0;
    Yaw(3, 1) = 0;
    Yaw(3, 2) = 0;
    Yaw(3, 3) = 1;

    Pitch(0, 0) = cos(theta2);
    Pitch(0, 1) = (-1)*sin(theta2)*cos(alpha2);
    Pitch(0, 2) = sin(theta2)*sin(alpha2);
    Pitch(0, 3) = a2*cos(theta2);
    Pitch(1, 0) = sin(theta2);
    Pitch(1, 1) = cos(theta2)*cos(alpha2);
    Pitch(1, 2) = (-1)*cos(theta2)*sin(alpha2);
    Pitch(1, 3) = a2*sin(theta2);
    Pitch(2, 0) = 0;
    Pitch(2, 1) = sin(alpha2);
    Pitch(2, 2) = cos(alpha2);
    Pitch(2, 3) = d2;
    Pitch(3, 0) = 0;
    Pitch(3, 1) = 0;
    Pitch(3, 2) = 0;
    Pitch(3, 3) = 1;

    Roll(0, 0) = cos(theta3);
    Roll(0, 1) = (-1)*sin(theta3)*cos(alpha3);
    Roll(0, 2) = sin(theta3)*sin(alpha3);
    Roll(0, 3) = a3*cos(theta3);
    Roll(1, 0) = sin(theta3);
    Roll(1, 1) = cos(theta3)*cos(alpha3);
    Roll(1, 2) = (-1)*cos(theta3)*sin(alpha3);
    Roll(1, 3) = a3*sin(theta3);
    Roll(2, 0) = 0;
    Roll(2, 1) = sin(alpha3);
    Roll(2, 2) = cos(alpha3);
    Roll(2, 3) = d3;
    Roll(3, 0) = 0;
    Roll(3, 1) = 0;
    Roll(3, 2) = 0;
    Roll(3, 3) = 1;

    P1 = Yaw*Roll*Pitch*P1;

    cout << P1 << endl << endl;

/*
    a1 = 0;
    alpha1 = (-1)*PI/2;
    d1 = 2;
    theta1 = 0*PI/180;

    a2 = 0;
    alpha2 = (-1)*PI/2;
    d2 = 0;
    theta2 = (90*PI/180) - (PI/2);

    a3 = 0;
    alpha3 = 0;
    d3 = 0;
    theta3 = 30*PI/180;

    Yaw(0, 0) = cos(theta1);
    Yaw(0, 1) = (-1)*sin(theta1)*cos(alpha1);
    Yaw(0, 2) = sin(theta1)*sin(alpha1);
    Yaw(0, 3) = a1*cos(theta1);
    Yaw(1, 0) = sin(theta1);
    Yaw(1, 1) = cos(theta1)*cos(alpha1);
    Yaw(1, 2) = (-1)*cos(theta1)*sin(alpha1);
    Yaw(1, 3) = a1*sin(theta1);
    Yaw(2, 0) = 0;
    Yaw(2, 1) = sin(alpha1);
    Yaw(2, 2) = cos(alpha1);
    Yaw(2, 3) = d1;
    Yaw(3, 0) = 0;
    Yaw(3, 1) = 0;
    Yaw(3, 2) = 0;
    Yaw(3, 3) = 1;

    Pitch(0, 0) = cos(theta2);
    Pitch(0, 1) = (-1)*sin(theta2)*cos(alpha2);
    Pitch(0, 2) = sin(theta2)*sin(alpha2);
    Pitch(0, 3) = a2*cos(theta2);
    Pitch(1, 0) = sin(theta2);
    Pitch(1, 1) = cos(theta2)*cos(alpha2);
    Pitch(1, 2) = (-1)*cos(theta2)*sin(alpha2);
    Pitch(1, 3) = a2*sin(theta2);
    Pitch(2, 0) = 0;
    Pitch(2, 1) = sin(alpha2);
    Pitch(2, 2) = cos(alpha2);
    Pitch(2, 3) = d2;
    Pitch(3, 0) = 0;
    Pitch(3, 1) = 0;
    Pitch(3, 2) = 0;
    Pitch(3, 3) = 1;

    Roll(0, 0) = cos(theta3);
    Roll(0, 1) = (-1)*sin(theta3)*cos(alpha3);
    Roll(0, 2) = sin(theta3)*sin(alpha3);
    Roll(0, 3) = a3*cos(theta3);
    Roll(1, 0) = sin(theta3);
    Roll(1, 1) = cos(theta3)*cos(alpha3);
    Roll(1, 2) = (-1)*cos(theta3)*sin(alpha3);
    Roll(1, 3) = a3*sin(theta3);
    Roll(2, 0) = 0;
    Roll(2, 1) = sin(alpha3);
    Roll(2, 2) = cos(alpha3);
    Roll(2, 3) = d3;
    Roll(3, 0) = 0;
    Roll(3, 1) = 0;
    Roll(3, 2) = 0;
    Roll(3, 3) = 1;

    P2 = Yaw*Pitch*Roll*P2;

    cout << P2 << endl << endl;
*/
    return 0;
}