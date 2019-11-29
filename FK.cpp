/* Created by Jonet Wira Murti */
/* 13518083 */

#include <iostream>
#include "FK.h"
#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace Eigen;
using namespace std;

#define PI 3.14159265358979323846

FK::FK() {
    dhMat(ID_R_HIP_YAW, A) = -5.05;dhMat(ID_R_HIP_YAW, ALPHA) =  0;dhMat(ID_R_HIP_YAW, THETA) = 0;dhMat(ID_R_HIP_YAW, D) = 16.6192;
    dhMat(ID_L_HIP_YAW, A) = 5.05;dhMat(ID_L_HIP_YAW, ALPHA) = 0;dhMat(ID_L_HIP_YAW, THETA) = 0;dhMat(ID_L_HIP_YAW, D) = 16.6192;
    dhMat(ID_R_HIP_ROLL, A) = 0;dhMat(ID_R_HIP_ROLL, ALPHA) =  PI/2;dhMat(ID_R_HIP_ROLL, THETA) = PI/2;dhMat(ID_R_HIP_ROLL, D) = 0;
    dhMat(ID_L_HIP_ROLL, A) = 0;dhMat(ID_L_HIP_ROLL, ALPHA) =  PI/2;dhMat(ID_L_HIP_ROLL, THETA) = PI/2;dhMat(ID_L_HIP_ROLL, D) = 0;
    dhMat(ID_R_HIP_PITCH, A) = 0;dhMat(ID_R_HIP_PITCH, ALPHA) =  (-1)*PI/2;dhMat(ID_R_HIP_PITCH, THETA) = 0;dhMat(ID_R_HIP_PITCH, D) = 0;
    dhMat(ID_L_HIP_PITCH, A) = 0;dhMat(ID_L_HIP_PITCH, ALPHA) =  (-1)*PI/2;dhMat(ID_L_HIP_PITCH, THETA) = 0;dhMat(ID_L_HIP_PITCH, D) = 0;
    dhMat(ID_R_KNEE, A) = 13.8291;dhMat(ID_R_KNEE, ALPHA) =  0;dhMat(ID_R_KNEE, THETA) = 0;dhMat(ID_R_KNEE, D) = 0;
    dhMat(ID_L_KNEE, A) = 13.8291;dhMat(ID_L_KNEE, ALPHA) =  0;dhMat(ID_L_KNEE, THETA) = 0;dhMat(ID_L_KNEE, D) = 0;
    dhMat(ID_R_ANKLE_PITCH, A) = 13.8291;dhMat(ID_R_ANKLE_PITCH, ALPHA) =  0;dhMat(ID_R_ANKLE_PITCH, THETA) = 0;dhMat(ID_R_ANKLE_PITCH, D) = 0;
    dhMat(ID_L_ANKLE_PITCH, A) = 13.8291;dhMat(ID_L_ANKLE_PITCH, ALPHA) =  0;dhMat(ID_L_ANKLE_PITCH, THETA) = 0;dhMat(ID_L_ANKLE_PITCH, D) = 0;
    dhMat(ID_R_ANKLE_ROLL, A) = 0;dhMat(ID_R_ANKLE_ROLL, ALPHA) =  PI/2;dhMat(ID_R_ANKLE_ROLL, THETA) = PI/2;dhMat(ID_R_ANKLE_ROLL, D) = 0;
    dhMat(ID_L_ANKLE_ROLL, A) = 0;dhMat(ID_L_ANKLE_ROLL, ALPHA) =  PI/2;dhMat(ID_L_ANKLE_ROLL, THETA) = PI/2;dhMat(ID_L_ANKLE_ROLL, D) = 0;
    dhMat(ID_HEAD_PAN, A) = 0;dhMat(ID_HEAD_PAN, ALPHA) =  (-1)*PI/2;dhMat(ID_HEAD_PAN, THETA) = PI/2;dhMat(ID_HEAD_PAN, D) = 2.855;
    dhMat(ID_HEAD_TILT, A) = 0;dhMat(ID_HEAD_TILT, ALPHA) = PI/2;dhMat(ID_HEAD_TILT, THETA) = 0;dhMat(ID_HEAD_TILT, D) = 0;
    dhMat(ID_R_BASE, A) = 0;dhMat(ID_R_BASE, ALPHA) = PI/2;dhMat(ID_R_BASE, THETA) = PI/2;dhMat(ID_R_BASE, D) = 4.98145;
    dhMat(ID_L_BASE, A) = 0;dhMat(ID_L_BASE, ALPHA) = PI/2;dhMat(ID_L_BASE, THETA) = PI/2;dhMat(ID_L_BASE, D) = 4.98145;
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
void FK::ThetaSet(int ID, double val)
{
    dhMat(ID, THETA) += DegToRad(val);
}
void FK::P1Set(double a, double b, double c)
{
    p1 << a, 
          b, 
          c, 
          1;
}
void FK::P2Set(double a, double b, double c)
{
    p2 << a, 
          b, 
          c, 
          1;
}


//Print
void FK::PrintLeftNormal() {
    cout << "Left Normal : " << endl << leftP2 - leftP1 << endl;
}
void FK::PrintRightNormal() {
    cout << "Right Normal : " << endl << rightP2 - rightP1 << endl;
}
void FK::PrintParam()
{
    printf("LEFT PARAM :\n");
    printf("BASE      : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_L_BASE, A), dhMat(ID_L_BASE, ALPHA), dhMat(ID_L_BASE, THETA), dhMat(ID_L_BASE, D));
    printf("ANK_ROLL  : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_L_ANKLE_ROLL, A), dhMat(ID_L_ANKLE_ROLL, ALPHA), dhMat(ID_L_ANKLE_ROLL, THETA), dhMat(ID_L_ANKLE_ROLL, D));
    printf("ANK_PITCH : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_L_ANKLE_PITCH, A), dhMat(ID_L_ANKLE_PITCH, ALPHA), dhMat(ID_L_ANKLE_PITCH, THETA), dhMat(ID_L_ANKLE_PITCH, D));
    printf("KNEE      : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_L_KNEE, A), dhMat(ID_L_KNEE, ALPHA), dhMat(ID_L_KNEE, THETA), dhMat(ID_L_KNEE, D));
    printf("HIP_PITCH : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_L_HIP_PITCH, A), dhMat(ID_L_HIP_PITCH, ALPHA), dhMat(ID_L_HIP_PITCH, THETA), dhMat(ID_L_HIP_PITCH, D));
    printf("HIP_ROLL  : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_L_HIP_ROLL, A), dhMat(ID_L_HIP_ROLL, ALPHA), dhMat(ID_L_HIP_ROLL, THETA), dhMat(ID_L_HIP_ROLL, D));
    printf("HIP_YAW   : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_L_HIP_YAW, A), dhMat(ID_L_HIP_YAW, ALPHA), dhMat(ID_L_HIP_YAW, THETA), dhMat(ID_L_HIP_YAW, D));
    
    printf("\nRIGHT PARAM :\n");
    printf("BASE      : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_R_BASE, A), dhMat(ID_R_BASE, ALPHA), dhMat(ID_R_BASE, THETA), dhMat(ID_R_BASE, D));
    printf("ANK_ROLL  : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_R_ANKLE_ROLL, A), dhMat(ID_R_ANKLE_ROLL, ALPHA), dhMat(ID_R_ANKLE_ROLL, THETA), dhMat(ID_R_ANKLE_ROLL, D));
    printf("ANK_PITCH : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_R_ANKLE_PITCH, A), dhMat(ID_R_ANKLE_PITCH, ALPHA), dhMat(ID_R_ANKLE_PITCH, THETA), dhMat(ID_R_ANKLE_PITCH, D));
    printf("KNEE      : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_R_KNEE, A), dhMat(ID_R_KNEE, ALPHA), dhMat(ID_R_KNEE, THETA), dhMat(ID_R_KNEE, D));
    printf("HIP_PITCH : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_R_HIP_PITCH, A), dhMat(ID_R_HIP_PITCH, ALPHA), dhMat(ID_R_HIP_PITCH, THETA), dhMat(ID_R_HIP_PITCH, D));
    printf("HIP_ROLL  : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_R_HIP_ROLL, A), dhMat(ID_R_HIP_ROLL, ALPHA), dhMat(ID_R_HIP_ROLL, THETA), dhMat(ID_R_HIP_ROLL, D));
    printf("HIP_YAW   : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_R_HIP_YAW, A), dhMat(ID_R_HIP_YAW, ALPHA), dhMat(ID_R_HIP_YAW, THETA), dhMat(ID_R_HIP_YAW, D));

    printf("\nHEAD PARAM :\n");
    printf("HEAD_PAN  : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_HEAD_PAN, A), dhMat(ID_HEAD_PAN, ALPHA), dhMat(ID_HEAD_PAN, THETA), dhMat(ID_HEAD_PAN, D));
    printf("HEAD_TILT : A(%.2f), ALPHA(%.2f), THETA(%.2f), D(%.2f)\n", dhMat(ID_HEAD_TILT, A), dhMat(ID_HEAD_TILT, ALPHA), dhMat(ID_HEAD_TILT, THETA), dhMat(ID_HEAD_TILT, D));
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

void FK::Process(){
    //Kaki kiri

    leftP1 = /* Transform(GetA(ID_L_BASE), GetAlpha(ID_L_BASE), GetD(ID_L_BASE), GetTheta(ID_L_BASE))*
             Transform(GetA(ID_L_ANKLE_ROLL), GetAlpha(ID_L_ANKLE_ROLL), GetD(ID_L_ANKLE_ROLL), GetTheta(ID_L_ANKLE_ROLL))*
             Transform(GetA(ID_L_ANKLE_PITCH), GetAlpha(ID_L_ANKLE_PITCH), GetD(ID_L_ANKLE_PITCH), GetTheta(ID_L_ANKLE_PITCH))*
             Transform(GetA(ID_L_KNEE), GetAlpha(ID_L_KNEE), GetD(ID_L_KNEE), GetTheta(ID_L_KNEE))*
             Transform(GetA(ID_L_HIP_PITCH), GetAlpha(ID_L_HIP_PITCH), GetD(ID_L_HIP_PITCH), GetTheta(ID_L_HIP_PITCH))*
             Transform(GetA(ID_L_HIP_ROLL), GetAlpha(ID_L_HIP_ROLL), GetD(ID_L_HIP_ROLL), GetTheta(ID_L_HIP_ROLL))* */
             Transform(GetA(ID_L_HIP_YAW), GetAlpha(ID_L_HIP_YAW), GetD(ID_L_HIP_YAW), GetTheta(ID_L_HIP_YAW))*
             Transform(GetA(ID_HEAD_PAN), GetAlpha(ID_HEAD_PAN), GetD(ID_HEAD_PAN), GetTheta(ID_HEAD_PAN))*
             Transform(GetA(ID_HEAD_TILT), GetAlpha(ID_HEAD_TILT), GetD(ID_HEAD_TILT), GetTheta(ID_HEAD_TILT))*p1;
    leftP2 = /* Transform(GetA(ID_L_BASE), GetAlpha(ID_L_BASE), GetD(ID_L_BASE), GetTheta(ID_L_BASE))*
             Transform(GetA(ID_L_ANKLE_ROLL), GetAlpha(ID_L_ANKLE_ROLL), GetD(ID_L_ANKLE_ROLL), GetTheta(ID_L_ANKLE_ROLL))*
             Transform(GetA(ID_L_ANKLE_PITCH), GetAlpha(ID_L_ANKLE_PITCH), GetD(ID_L_ANKLE_PITCH), GetTheta(ID_L_ANKLE_PITCH))*
             Transform(GetA(ID_L_KNEE), GetAlpha(ID_L_KNEE), GetD(ID_L_KNEE), GetTheta(ID_L_KNEE))*
             Transform(GetA(ID_L_HIP_PITCH), GetAlpha(ID_L_HIP_PITCH), GetD(ID_L_HIP_PITCH), GetTheta(ID_L_HIP_PITCH))*
             Transform(GetA(ID_L_HIP_ROLL), GetAlpha(ID_L_HIP_ROLL), GetD(ID_L_HIP_ROLL), GetTheta(ID_L_HIP_ROLL))* */
             Transform(GetA(ID_L_HIP_YAW), GetAlpha(ID_L_HIP_YAW), GetD(ID_L_HIP_YAW), GetTheta(ID_L_HIP_YAW))*
             Transform(GetA(ID_HEAD_PAN), GetAlpha(ID_HEAD_PAN), GetD(ID_HEAD_PAN), GetTheta(ID_HEAD_PAN))*
             Transform(GetA(ID_HEAD_TILT), GetAlpha(ID_HEAD_TILT), GetD(ID_HEAD_TILT), GetTheta(ID_HEAD_TILT))*p2;

    //Kaki kanan
    rightP1= /* Transform(GetA(ID_R_BASE), GetAlpha(ID_R_BASE), GetD(ID_R_BASE), GetTheta(ID_R_BASE))*
             Transform(GetA(ID_R_ANKLE_ROLL), GetAlpha(ID_R_ANKLE_ROLL), GetD(ID_R_ANKLE_ROLL), GetTheta(ID_R_ANKLE_ROLL))*
             Transform(GetA(ID_R_ANKLE_PITCH), GetAlpha(ID_R_ANKLE_PITCH), GetD(ID_R_ANKLE_PITCH), GetTheta(ID_R_ANKLE_PITCH))*
             Transform(GetA(ID_R_KNEE), GetAlpha(ID_R_KNEE), GetD(ID_R_KNEE), GetTheta(ID_R_KNEE))*
             Transform(GetA(ID_R_HIP_PITCH), GetAlpha(ID_R_HIP_PITCH), GetD(ID_R_HIP_PITCH), GetTheta(ID_R_HIP_PITCH))*
             Transform(GetA(ID_R_HIP_ROLL), GetAlpha(ID_R_HIP_ROLL), GetD(ID_R_HIP_ROLL), GetTheta(ID_R_HIP_ROLL))* */
             Transform(GetA(ID_R_HIP_YAW), GetAlpha(ID_R_HIP_YAW), GetD(ID_R_HIP_YAW), GetTheta(ID_R_HIP_YAW))* 
             Transform(GetA(ID_HEAD_PAN), GetAlpha(ID_HEAD_PAN), GetD(ID_HEAD_PAN), GetTheta(ID_HEAD_PAN))*
             Transform(GetA(ID_HEAD_TILT), GetAlpha(ID_HEAD_TILT), GetD(ID_HEAD_TILT), GetTheta(ID_HEAD_TILT))*p1;
    rightP2= /* Transform(GetA(ID_R_BASE), GetAlpha(ID_R_BASE), GetD(ID_R_BASE), GetTheta(ID_R_BASE))*
             Transform(GetA(ID_R_ANKLE_ROLL), GetAlpha(ID_R_ANKLE_ROLL), GetD(ID_R_ANKLE_ROLL), GetTheta(ID_R_ANKLE_ROLL))*
             Transform(GetA(ID_R_ANKLE_PITCH), GetAlpha(ID_R_ANKLE_PITCH), GetD(ID_R_ANKLE_PITCH), GetTheta(ID_R_ANKLE_PITCH))*
             Transform(GetA(ID_R_KNEE), GetAlpha(ID_R_KNEE), GetD(ID_R_KNEE), GetTheta(ID_R_KNEE))*
             Transform(GetA(ID_R_HIP_PITCH), GetAlpha(ID_R_HIP_PITCH), GetD(ID_R_HIP_PITCH), GetTheta(ID_R_HIP_PITCH))*
             Transform(GetA(ID_R_HIP_ROLL), GetAlpha(ID_R_HIP_ROLL), GetD(ID_R_HIP_ROLL), GetTheta(ID_R_HIP_ROLL))* */
             Transform(GetA(ID_R_HIP_YAW), GetAlpha(ID_R_HIP_YAW), GetD(ID_R_HIP_YAW), GetTheta(ID_R_HIP_YAW))*
             Transform(GetA(ID_HEAD_PAN), GetAlpha(ID_HEAD_PAN), GetD(ID_HEAD_PAN), GetTheta(ID_HEAD_PAN))*
             Transform(GetA(ID_HEAD_TILT), GetAlpha(ID_HEAD_TILT), GetD(ID_HEAD_TILT), GetTheta(ID_HEAD_TILT))*p2;
}