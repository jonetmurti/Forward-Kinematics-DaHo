/* Created by Jonet Wira */
/* 13518083 */

#ifndef FK_H
#define FK_H

#include <Eigen/Dense>
#include <ros/ros.h>


using namespace Eigen;

class FK {

    private :
        double AngleData[21];

        Vector4f p1;
        Vector4f p2; 
        Vector4f leftP1;
        Vector4f leftP2;
        Vector4f rightP1;
        Vector4f rightP2;        

        // Joint Parameter
        Matrix<double, 23, 5> dhMat;



    public :

        enum
		{
			ID_R_SHOULDER_PITCH     = 1,
			ID_L_SHOULDER_PITCH     = 2,
			ID_R_SHOULDER_ROLL      = 3,
			ID_L_SHOULDER_ROLL      = 4,
			ID_R_ELBOW              = 5,
			ID_L_ELBOW              = 6,
			ID_R_HIP_YAW            = 7,
			ID_L_HIP_YAW            = 8,
			ID_R_HIP_ROLL           = 9,
			ID_L_HIP_ROLL           = 10,
			ID_R_HIP_PITCH          = 11,
			ID_L_HIP_PITCH          = 12,
			ID_R_KNEE               = 13,
			ID_L_KNEE               = 14,
			ID_R_ANKLE_PITCH        = 15,
			ID_L_ANKLE_PITCH        = 16,
			ID_R_ANKLE_ROLL         = 17,
			ID_L_ANKLE_ROLL         = 18,
			ID_HEAD_PAN             = 19,
			ID_HEAD_TILT            = 20,
            ID_R_BASE               = 21,
            ID_L_BASE               = 22,
			NUMBER_OF_JOINTS
		};

        enum DH_PARAM {
            A = 1,
            ALPHA = 2,
            THETA = 3,
            D = 4
        };

        FK();
        ~FK();

        //Vector Getter
        Vector4f GetP1();
        Vector4f GetP2();
        Vector4f GetLeftP1();
        Vector4f GetLeftP2();
        Vector4f GetRightP1();
        Vector4f GetRightP2();

        //Param Getter
        double GetA(int ID);
        double GetAlpha(int ID);
        double GetTheta(int ID);
        double GetD(int ID);


        //Param Setter

        //PRINT
        void PrintLeftNormal();
        void PrintRightNormal();

        //Process
        Matrix4f Transform(double a, double alpha, double d, double theta);
        double DegToRad(double a);
        void ThetaOffset();
        void Process();
};

#endif