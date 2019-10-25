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
    LeftP1 = Transform(GetA(0, 1), GetAlpha(0, 1), GetD(0,1), GetTheta(0, 1))*Transform(GetA(1, 1), GetAlpha(0, 1), GetD(1,1), GetTheta(1, 1))*
             Transform(GetA(2, 1), GetAlpha(2, 1), GetD(2,1), GetTheta(2, 1))*P1;
}