#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "FK.h"

using namespace Eigen;
using namespace std;

#define PI 3.14159265

int main () {

		FK Eva;
		//HEAD OT NECK TESTING

		
		Eva.P1Set(0, 0, 0);
		Eva.P2Set(4, 0, 0);

		Eva.ThetaSet(FK::ID_HEAD_PAN, 45);
		//Eva.ThetaSet(FK::ID_HEAD_TILT, -30);
		
		/*
		cout << "Theta Pan = 30" << endl;
		Eva.PrintParam();

		Eva.Process();

		Eva.PrintLeftNormal();
		Eva.PrintRightNormal();
		*/

		//cout << Eva.Transform(0, Eva.DegToRad(90), 0, Eva.DegToRad(-30)) << endl;
		

		// Vector4f v = Eva.GetLeftP2();

		// cout << endl << v(0, 0) << endl;
		//NECK TO HIP TESTING

		
		Eva.ThetaSet(FK::ID_L_HIP_YAW, 45);
		Eva.ThetaSet(FK::ID_R_HIP_YAW, 45);

		Eva.PrintParam();

		Eva.Process();

		Eva.PrintLeftNormal();
		Eva.PrintRightNormal();

		//
    
    return 0;
}