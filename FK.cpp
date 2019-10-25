#include <iostream>
#include "FK.h"
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace Eigen;
using namespace std;

FK::FK() {
    //
    P1 << 0, 0, 3.44, 1;
    P2 << 3.32, 0, 3.44, 1;
}

FK::~FK() {
    //
}

//Vector Getter
Vector4f FK::GetP1(){
    return P1;
}
Vector4f FK::GetP2(){
    return P2;
}
Vector4f FK::GetLeftP1(){
    return LeftP1;
}
Vector4f FK::GetLeftP2(){
    return LeftP2;
}
Vector4f FK::GetRightP1(){
    return RightP1;
}
Vector4f FK::GetRightP2(){
    return RightP2;
}

//Param Getter
float GetA(int i, int j){
    return A(i, j);
}
float GetAlpha(int i, int j){
    return Alpha(i, j);
}
float GetD(int i, int j){
    return D(i, j);
}
float GetTheta(int i, int j){
    return Theta(i, j);
}

Matrix4f FK::Transform(float a, float alpha, float d, float theta){
    Matrix4f P;
    P << cos(theta), (-1)*sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
         sin(theta), cos(theta)*cos(alpha), (-1)*cos(theta)*sin(alpha), a*sin(theta),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;

    return P;
}

void FK::Process(){
    //Kaki kiri
    LeftP1 = Transform(GetA(0, 1), GetAlpha(0, 1), GetD(0, 1), GetTheta(0, 1))*Transform(GetA(1, 1), GetAlpha(1, 1), GetD(1, 1), GetTheta(1, 1))*
             Transform(GetA(2, 1), GetAlpha(2, 1), GetD(2, 1), GetTheta(2, 1))*Transform(GetA(3, 1), GetAlpha(3, 1), GetD(3, 1), GetTheta(3, 1))*
             Transform(GetA(4, 1), GetAlpha(4, 1), GetD(4, 1), GetTheta(4, 1))*Transform(GetA(5, 1), GetAlpha(5, 1), GetD(5, 1), GetTheta(5, 1))*
             Transform(GetA(6, 1), GetAlpha(6, 1), GetD(6, 1), GetTheta(6, 1))*Transform(GetA(7, 1), GetAlpha(7, 1), GetD(7, 1), GetTheta(7, 1))*
             Transform(GetA(8, 1), GetAlpha(8, 1), GetD(8, 1), GetTheta(8, 1))*P1;
    LeftP2 = Transform(GetA(0, 1), GetAlpha(0, 1), GetD(0, 1), GetTheta(0, 1))*Transform(GetA(1, 1), GetAlpha(1, 1), GetD(1, 1), GetTheta(1, 1))*
             Transform(GetA(2, 1), GetAlpha(2, 1), GetD(2, 1), GetTheta(2, 1))*Transform(GetA(3, 1), GetAlpha(3, 1), GetD(3, 1), GetTheta(3, 1))*
             Transform(GetA(4, 1), GetAlpha(4, 1), GetD(4, 1), GetTheta(4, 1))*Transform(GetA(5, 1), GetAlpha(5, 1), GetD(5, 1), GetTheta(5, 1))*
             Transform(GetA(6, 1), GetAlpha(6, 1), GetD(6, 1), GetTheta(6, 1))*Transform(GetA(7, 1), GetAlpha(7, 1), GetD(7, 1), GetTheta(7, 1))*
             Transform(GetA(8, 1), GetAlpha(8, 1), GetD(8, 1), GetTheta(8, 1))*P2;

    //Kaki kanan
    RightP1= Transform(GetA(9, 1), GetAlpha(9, 1), GetD(9, 1), GetTheta(9, 1))*Transform(GetA(10, 1), GetAlpha(10, 1), GetD(10, 1), GetTheta(10, 1))*
             Transform(GetA(11, 1), GetAlpha(11, 1), GetD(11, 1), GetTheta(11, 1))*Transform(GetA(12, 1), GetAlpha(12, 1), GetD(12, 1), GetTheta(12, 1))*
             Transform(GetA(13, 1), GetAlpha(13, 1), GetD(13, 1), GetTheta(13, 1))*Transform(GetA(14, 1), GetAlpha(14, 1), GetD(14, 1), GetTheta(14, 1))*
             Transform(GetA(15, 1), GetAlpha(15, 1), GetD(15, 1), GetTheta(15, 1))*Transform(GetA(16, 1), GetAlpha(16, 1), GetD(16, 1), GetTheta(16, 1))*
             Transform(GetA(17, 1), GetAlpha(17, 1), GetD(17, 1), GetTheta(17, 1))*P1;
    RightP2= Transform(GetA(9, 1), GetAlpha(9, 1), GetD(9, 1), GetTheta(9, 1))*Transform(GetA(10, 1), GetAlpha(10, 1), GetD(10, 1), GetTheta(10, 1))*
             Transform(GetA(11, 1), GetAlpha(11, 1), GetD(11, 1), GetTheta(11, 1))*Transform(GetA(12, 1), GetAlpha(12, 1), GetD(12, 1), GetTheta(12, 1))*
             Transform(GetA(13, 1), GetAlpha(13, 1), GetD(13, 1), GetTheta(13, 1))*Transform(GetA(14, 1), GetAlpha(14, 1), GetD(14, 1), GetTheta(14, 1))*
             Transform(GetA(15, 1), GetAlpha(15, 1), GetD(15, 1), GetTheta(15, 1))*Transform(GetA(16, 1), GetAlpha(16, 1), GetD(16, 1), GetTheta(16, 1))*
             Transform(GetA(17, 1), GetAlpha(17, 1), GetD(17, 1), GetTheta(17, 1))*P2;
}