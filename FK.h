#ifndef _FK_H
#define _FK_H
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class FK {

    private :
        Vector4f P1;
        Vector4f P2;
        Vector4f LeftP1;
        Vector4f LeftP2;
        Vector4f RightP1;
        Vector4f RightP2;

        // Joint Parameter
        Matrix<float, 18, 1> A; //Jarak antara Origin(i) ke perpotongannya dengan z(i-1)
        Matrix<float, 18, 1> Alpha; //Sudut antara z(i-1) dengan z(i) diukur dari x(i)
        Matrix<float, 18, 1> D; //Jarak Origin(i-1) ke perpotongannya dengan x(i)
        Matrix<float, 18, 1> Theta; // Sudut antara x(i-1) dengan x(i) diukur dari z(i-1)

    public :
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
        float GetA(int i, int j);
        float GetAlpha(int i, int j);
        float GetD(int i, int j);
        float GetTheta(int i, int j);


        Matrix4f Transform(float a, float alpha, float d, float theta);
        void Process();
};

#endif