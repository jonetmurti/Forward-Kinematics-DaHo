/* Created by Jonet Wira Murti */
/* 13518083 */

#include <iostream>
#include "FK.h"
#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;
using namespace std;

#define PI 3.14159265358979323846

FK::FK() {
    dhMat(ID_R_HIP_YAW, A) = ;dhMat(ID_R_HIP_YAW, ALPHA) =  ;dhMat(ID_R_HIP_YAW, THETA) = ;dhMat(ID_R_HIP_YAW, D) = 
    dhMat(ID_L_HIP_YAW, A) = ;dhMat(ID_L_HIP_YAW, ALPHA) =  ;dhMat(ID_L_HIP_YAW, THETA) = ;dhMat(ID_L_HIP_YAW, D) = 
    dhMat(ID_R_HIP_ROLL, A) = ;dhMat(ID_R_HIP_ROLL, ALPHA) =  ;dhMat(ID_R_HIP_ROLL, THETA) = ;dhMat(ID_R_HIP_ROLL, D) = 
    dhMat(ID_L_HIP_ROLL, A) = ;dhMat(ID_L_HIP_ROLL, ALPHA) =  ;dhMat(ID_L_HIP_ROLL, THETA) = ;dhMat(ID_L_HIP_ROLL, D) = 
    dhMat(ID_R_HIP_PITCH, A) = ;dhMat(ID_R_HIP_PITCH, ALPHA) =  ;dhMat(ID_R_HIP_PITCH, THETA) = ;dhMat(ID_R_HIP_PITCH, D) = 
    dhMat(ID_L_HIP_PITCH, A) = ;dhMat(ID_L_HIP_PITCH, ALPHA) =  ;dhMat(ID_L_HIP_PITCH, THETA) = ;dhMat(ID_L_HIP_PITCH, D) = 
    dhMat(ID_R_KNEE, A) = ;dhMat(ID_R_KNEE, ALPHA) =  ;dhMat(ID_R_KNEE, THETA) = ;dhMat(ID_R_KNEE, D) = 
    dhMat(ID_L_KNEE, A) = ;dhMat(ID_L_KNEE, ALPHA) =  ;dhMat(ID_L_KNEE, THETA) = ;dhMat(ID_L_KNEE, D) = 
    dhMat(ID_R_ANKLE_PITCH, A) = ;dhMat(ID_R_ANKLE_PITCH, ALPHA) =  ;dhMat(ID_R_ANKLE_PITCH, THETA) = ;dhMat(ID_R_ANKLE_PITCH, D) = 
    dhMat(ID_L_ANKLE_PITCH, A) = ;dhMat(ID_L_ANKLE_PITCH, ALPHA) =  ;dhMat(ID_L_ANKLE_PITCH, THETA) = ;dhMat(ID_L_ANKLE_PITCH, D) = 
    dhMat(ID_R_ANKLE_ROLL, A) = ;dhMat(ID_R_ANKLE_ROLL, ALPHA) =  ;dhMat(ID_R_ANKLE_ROLL, THETA) = ;dhMat(ID_R_ANKLE_ROLL, D) = 
    dhMat(ID_L_ANKLE_ROLL, A) = ;dhMat(ID_L_ANKLE_ROLL, ALPHA) =  ;dhMat(ID_L_ANKLE_ROLL, THETA) = ;dhMat(ID_L_ANKLE_ROLL, D) = 
    dhMat(ID_HEAD_PAN, A) = ;dhMat(ID_HEAD_PAN, ALPHA) =  ;dhMat(ID_HEAD_PAN, THETA) = ;dhMat(ID_HEAD_PAN, D) = 
    dhMat(ID_HEAD_TILT, A) = ;dhMat(ID_HEAD_TILT, ALPHA) = ;dhMat(ID_HEAD_TILT, THETA) = ;dhMat(ID_HEAD_TILT, D) = ;
}

FK::~FK() {
    //
}

//Vector Getter
Vector4f FK::GetP1(){
    return p1;
}
Vector4f FK::GetP2(){
    return p2;
}
Vector4f FK::GetLeftP1(){
    return leftP1;
}
Vector4f FK::GetLeftP2(){
    return leftP2;
}
Vector4f FK::GetRightP1(){
    return rightP1;
}
Vector4f FK::GetRightP2(){
    return rightP2;
}

//Param Getter
double FK::GetA(int ID) {
    return dhMat(ID, A);
}
double FK::GetAlpha(int ID) {
    return dhMat(ID, ALPHA);
}
double FK::GetTheta(int ID) {
    return dhMat(ID, THETA);
}
double FK::GetD(int ID) {
    return dhMat(ID, D);
    
}

//Param Setter

//Print
void FK::PrintLeftNormal() {
    cout << leftP2 - leftP1;
}
void FK::PrintRightNormal() {
    cout << rightP2 - rightP1;
}

//Process
Matrix4f FK::Transform(double a, double alpha, double d, double theta){
    Matrix4f P;
    P << cos(theta), (-1)*sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
         sin(theta), cos(theta)*cos(alpha), (-1)*cos(theta)*sin(alpha), a*sin(theta),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;

    return P;
}

double FK::DegToRad(double a){
    return a*PI/180;
}

void ThetaOffset();

void FK::Process(){
    //Kaki kiri

    leftP1 = Transform(GetA(ID_L_BASE), GetAlpha(0, 0), GetD(0, 0), GetTheta(0, 0))*Transform(GetA(1, 0), GetAlpha(1, 0), GetD(1, 0), GetTheta(1, 0))*
             Transform(GetA(2, 0), GetAlpha(2, 0), GetD(2, 0), GetTheta(2, 0))*Transform(GetA(3, 0), GetAlpha(3, 0), GetD(3, 0), GetTheta(3, 0))*
             Transform(GetA(4, 0), GetAlpha(4, 0), GetD(4, 0), GetTheta(4, 0))*Transform(GetA(5, 0), GetAlpha(5, 0), GetD(5, 0), GetTheta(5, 0))*
             Transform(GetA(6, 0), GetAlpha(6, 0), GetD(6, 0), GetTheta(6, 0))*Transform(GetA(8, 0), GetAlpha(8, 0), GetD(8, 0), GetTheta(14, 0))*
             Transform(GetA(9, 0), GetAlpha(9, 0), GetD(9, 0), GetTheta(15, 0))*p1;
    leftP2 = Transform(GetA(0, 0), GetAlpha(0, 0), GetD(0, 0), GetTheta(0, 0))*Transform(GetA(1, 0), GetAlpha(1, 0), GetD(1, 0), GetTheta(1, 0))*
             Transform(GetA(2, 0), GetAlpha(2, 0), GetD(2, 0), GetTheta(2, 0))*Transform(GetA(3, 0), GetAlpha(3, 0), GetD(3, 0), GetTheta(3, 0))*
             Transform(GetA(4, 0), GetAlpha(4, 0), GetD(4, 0), GetTheta(4, 0))*Transform(GetA(5, 0), GetAlpha(5, 0), GetD(5, 0), GetTheta(5, 0))*
             Transform(GetA(6, 0), GetAlpha(6, 0), GetD(6, 0), GetTheta(6, 0))*Transform(GetA(8, 0), GetAlpha(8, 0), GetD(8, 0), GetTheta(14, 0))*
             Transform(GetA(9, 0), GetAlpha(9, 0), GetD(9, 0), GetTheta(15, 0))*p2;

    //Kaki kanan
    rightP1= Transform(GetA(0, 0), GetAlpha(0, 0), GetD(0, 0), GetTheta(7, 0))*Transform(GetA(1, 0), GetAlpha(1, 0), GetD(1, 0), GetTheta(8, 0))*
             Transform(GetA(2, 0), GetAlpha(2, 0), GetD(2, 0), GetTheta(9, 0))*Transform(GetA(3, 0), GetAlpha(3, 0), GetD(3, 0), GetTheta(10, 0))*
             Transform(GetA(4, 0), GetAlpha(4, 0), GetD(4, 0), GetTheta(11, 0))*Transform(GetA(5, 0), GetAlpha(5, 0), GetD(5, 0), GetTheta(12, 0))*
             Transform(GetA(7, 0), GetAlpha(7, 0), GetD(7, 0), GetTheta(13, 0))*Transform(GetA(8, 0), GetAlpha(8, 0), GetD(8, 0), GetTheta(14, 0))*
             Transform(GetA(9, 0), GetAlpha(9, 0), GetD(9, 0), GetTheta(15, 0))*P1;
    rightP2= Transform(GetA(0, 0), GetAlpha(0, 0), GetD(0, 0), GetTheta(7, 0))*Transform(GetA(1, 0), GetAlpha(1, 0), GetD(1, 0), GetTheta(8, 0))*
             Transform(GetA(2, 0), GetAlpha(2, 0), GetD(2, 0), GetTheta(9, 0))*Transform(GetA(3, 0), GetAlpha(3, 0), GetD(3, 0), GetTheta(10, 0))*
             Transform(GetA(4, 0), GetAlpha(4, 0), GetD(4, 0), GetTheta(11, 0))*Transform(GetA(5, 0), GetAlpha(5, 0), GetD(5, 0), GetTheta(12, 0))*
             Transform(GetA(7, 0), GetAlpha(7, 0), GetD(7, 0), GetTheta(13, 0))*Transform(GetA(8, 0), GetAlpha(8, 0), GetD(8, 0), GetTheta(14, 0))*
             Transform(GetA(9, 0), GetAlpha(9, 0), GetD(9, 0), GetTheta(15, 0))*P2;
}