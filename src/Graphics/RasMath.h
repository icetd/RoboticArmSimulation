#ifndef QDYNMATH_H
#define QDYNMATH_H

#include <math.h>
#include <QDebug>
#include "RasMatrix.h"

#if (WIN32)
#define M_PI       3.14159265358979323846   // pi
#endif

class RasMath
{
public:
    RasMath();

    // Angle functions
    static double normalizeAngle(double angle);
    static double deg2Rad(double angle);
    static double rad2Deg(double angle);

    // Generic Matrix Functions
    static RasMatrix* multMatrix(RasMatrix *A, RasMatrix *B);
    static RasMatrix* addMatrix(RasMatrix *A, RasMatrix *B);
    static void multMatrix(RasMatrix *A, RasMatrix *B, RasMatrix *R);
    static void addMatrix(RasMatrix *A, RasMatrix *B, RasMatrix *R);
    static RasMatrix* multMatScalar(RasMatrix *M, double scalar);
    static void multMatScalar(RasMatrix *M, double scalar, RasMatrix *R);

    // Vector Functions
    static RasMatrix* crossProd(RasMatrix *V1, RasMatrix *V2);
    static void crossProd(RasMatrix *V1, RasMatrix *V2, RasMatrix *Vr);

    // Coordinate Transform Functions
    static void singleCoorTrans(double T[4][4], double P[4], double PNew[4]);

    // Matrix Copying Functions
    static void copyMatTrans(const double S[4][4], double D[4][4]);
    static void copyMatTrans(RasMatrix *S, double D[4][4]);
    static void copyMatTrans(const double S[4][4], RasMatrix *D);
    static void copyMatRot(const double S[3][3], double D[3][3]);
    static void copyMatRot(RasMatrix *S, double D[3][3]);
    static void copyMatRot(const double S[3][3], RasMatrix *D);
    static void copyVec(const double S[3], double D[3]);
    static void copyRotFromTrans(RasMatrix *T, double R[3][3]);
    static void copyRotFromTrans(const double T[4][4], RasMatrix *R);

    // Matrix Operations Functions
    static void multTransMat(double M1[4][4],double M2[4][4],double T[4][4]) ;
    static void multRotMat(double M1[3][3],double M2[3][3],double R[3][3]);

    // Matrix Creation Functions
    static void setTransIdentity(double T[4][4]);
    static void setIdentity(RasMatrix *M);
    static void convRxRyRz2RotM(double rxDeg, double ryDeg, double rzDeg, double Rot[3][3], int rotOrder = 1);
    static void convDH2Trans(double a, double alphaDeg, double d, double thetaDeg, double T[4][4]);
    static void convDH2Trans(double a, double alphaDeg, double d, double thetaDeg, RasMatrix *T);

    // Matrix Conversion Functions
    static void convRotTra2glMat(double R[3][3], double T[3], double glMat[16]);
    static void convTrans2glMat(double T[4][4], double glMat[16]);
    static void convTrans2glMat(RasMatrix *T, double glMat[16]);
    static void convRotTra2Trans(double R[3][3], double T[3], double TT[4][4]);
    static void convRotTra2Trans(double R[3][3], double T[3], RasMatrix *TT);
    static void convTrans2RotTra(double TT[4][4], double R[3][3], double T[3]);
    static void convTrans2RotTra(RasMatrix *TT, double R[3][3], double T[3]);
    static void convRot2RPY(const double R[3][3],double &roll, double &pitch, double &yaw);

    // Joint pair function
    static double derFunErrRot(double TPre[4][4],double TNex[4][4],double xf, double yf, double zf, double a, double alphaDeg, double d, double thetaDeg);
    static double derFunErrPri(double TPre[4][4],double TNex[4][4],double xf, double yf, double zf, double a, double alphaDeg, double d, double thetaDeg);
    static void newtonRR(double &derErr, double &x, double &y, double TPrev[4][4], double TNext[4][4], double xf, double yf, double zf, double a1, double alpha1Deg, double d1, double theta1Deg, double a2, double alpha2Deg, double d2, double theta2Deg);
    static void newtonRP(double &derErr, double &x, double &y, double TPrev[4][4], double TNext[4][4], double xf, double yf, double zf, double a1, double alpha1Deg, double d1, double theta1Deg, double a2, double alpha2Deg, double d2, double theta2Deg);
    static void newtonPR(double &derErr, double &x, double &y, double TPrev[4][4], double TNext[4][4], double xf, double yf, double zf, double a1, double alpha1Deg, double d1, double theta1Deg, double a2, double alpha2Deg, double d2, double theta2Deg);
    static void newtonPP(double &derErr, double &x, double &y, double TPrev[4][4], double TNext[4][4], double xf, double yf, double zf, double a1, double alpha1Deg, double d1, double theta1Deg, double a2, double alpha2Deg, double d2, double theta2Deg);

    // Debugging Functions
    static void printTransMat(double T[4][4]);
    static void printMatrix(RasMatrix *M);
};

#endif // QDYNMATH_H
