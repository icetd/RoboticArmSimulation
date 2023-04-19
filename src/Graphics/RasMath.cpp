#include "RasMath.h"

RasMath::RasMath()
{
}
// Angle functions

double RasMath::normalizeAngle(double angle)
{
    double retAng = angle;
    while(retAng < 0.0)
    {
        retAng += 360.0;
    }
    while(retAng >= 360.0)
    {
        retAng -= 360.0;
    }
    return retAng;
}

double RasMath::deg2Rad(double angle)
{
    return angle * M_PI / 180.0;
}

double RasMath::rad2Deg(double angle)
{
    return angle * 180.0 / M_PI;
}

// End of Angle functions

// Generic Matrix Functions

RasMatrix* RasMath::multMatrix(RasMatrix *A, RasMatrix *B)
{
    RasMatrix *R;

    if(A->isScalar)
    {
        R = new RasMatrix(B->rows, B->cols);

        for(int i=0; i<B->rows; i++)
        {
            for(int j=0; j<B->cols; j++)
            {
                // Every element of the i-th row of B Matrix has to be multiplied with the scalar A
                R->data[i][j] = A->data[0][0] * B->data[i][j];
            }
        }
    }
    else if(B->isScalar)
    {
        R = new RasMatrix(A->rows, A->cols);

        for(int i=0; i<A->rows; i++)
        {
            for(int j=0; j<A->cols; j++)
            {
                // Every element of the i-th row of A Matrix has to be multiplied with the scalar B
                R->data[i][j] = B->data[0][0] * A->data[i][j];
            }
        }
    }
    else
    {
        if(A->cols != B->rows)
        return 0;

        R = new RasMatrix(A->rows, B->cols);

        for(int i=0; i<A->rows; i++)
        {
            for(int j=0; j<B->cols; j++)
            {
                // Every element of the i-th row of A Matrix has to be multiplied with every element of j-th column of matrix B
                R->data[i][j] = 0;
                for(int k=0; k<A->cols; k++)
                {
                    R->data[i][j] += A->data[i][k] * B->data[k][j];
                }
            }
        }
    }

    return R;
}

RasMatrix* RasMath::addMatrix(RasMatrix *A, RasMatrix *B)
{
    RasMatrix *R;
    if(A->isScalar)
    {
        R = new RasMatrix(B->rows, B->cols);

        for(int i=0; i<B->rows; i++)
        {
            for(int j=0; j<B->cols; j++)
            {
                // Every element of the i-th row of B Matrix has to be multiplied with the scalar A
                R->data[i][j] = A->data[0][0] + B->data[i][j];
            }
        }
    }
    else if(B->isScalar)
    {
        R = new RasMatrix(A->rows, A->cols);

        for(int i=0; i<A->rows; i++)
        {
            for(int j=0; j<A->cols; j++)
            {
                // Every element of the i-th row of A Matrix has to be multiplied with the scalar B
                R->data[i][j] = B->data[0][0] + A->data[i][j];
            }
        }
    }
    else
    {

        if(A->cols != B->cols || A->rows != B->rows)
        return 0;

        R = new RasMatrix(A->rows, A->cols);

        for(int i=0; i<A->rows; i++)
        {
            for(int j=0; j<A->cols; j++)
            {
                // Every element of the i-th row of A Matrix has to be multiplied with every element of j-th column of matrix B
                R->data[i][j] = A->data[i][j] + B->data[i][j];
            }
        }
    }

    return R;
}

void RasMath::multMatrix(RasMatrix *A, RasMatrix *B, RasMatrix *R)
{
    if(A->isScalar)
    {
        for(int i=0; i<B->rows; i++)
        {
            for(int j=0; j<B->cols; j++)
            {
                // Every element of the i-th row of B Matrix has to be multiplied with the scalar A
                R->data[i][j] = A->data[0][0] * B->data[i][j];
            }
        }
    }
    else if(B->isScalar)
    {
        for(int i=0; i<A->rows; i++)
        {
            for(int j=0; j<A->cols; j++)
            {
                // Every element of the i-th row of A Matrix has to be multiplied with the scalar B
                R->data[i][j] = B->data[0][0] * A->data[i][j];
            }
        }
    }
    else
    {
        if(A->cols != B->rows)
        return;

        for(int i=0; i<A->rows; i++)
        {
            for(int j=0; j<B->cols; j++)
            {
                // Every element of the i-th row of A Matrix has to be multiplied with every element of j-th column of matrix B
                R->data[i][j] = 0;
                for(int k=0; k<A->cols; k++)
                {
                    R->data[i][j] += A->data[i][k] * B->data[k][j];
                }
            }
        }
    }
}

void RasMath::addMatrix(RasMatrix *A, RasMatrix *B, RasMatrix *R)
{
    if(A->isScalar)
    {
        for(int i=0; i<B->rows; i++)
        {
            for(int j=0; j<B->cols; j++)
            {
                // Every element of the i-th row of B Matrix has to be multiplied with the scalar A
                R->data[i][j] = A->data[0][0] + B->data[i][j];
            }
        }
    }
    else if(B->isScalar)
    {
        for(int i=0; i<A->rows; i++)
        {
            for(int j=0; j<A->cols; j++)
            {
                // Every element of the i-th row of A Matrix has to be multiplied with the scalar B
                R->data[i][j] = B->data[0][0] + A->data[i][j];
            }
        }
    }
    else
    {

        if(A->cols != B->cols || A->rows != B->rows)
        return;

        for(int i=0; i<A->rows; i++)
        {
            for(int j=0; j<A->cols; j++)
            {
                // Every element of the i-th row of A Matrix has to be multiplied with every element of j-th column of matrix B
                R->data[i][j] = A->data[i][j] + B->data[i][j];
            }
        }
    }
}

RasMatrix* RasMath::multMatScalar(RasMatrix *M, double scalar)
{
    RasMatrix *R = new RasMatrix(M->rows,M->cols);

    for(int i=0; i<M->rows; i++)
    {
        for(int j=0; j<M->cols; j++)
        {
            // Every element of the i-th row of A Matrix has to be multiplied with every element of j-th column of matrix B
            R->data[i][j] = M->data[i][j] * scalar;
        }
    }

    return R;
}

void RasMath::multMatScalar(RasMatrix *M, double scalar, RasMatrix *R)
{
    for(int i=0; i<M->rows; i++)
    {
        for(int j=0; j<M->cols; j++)
        {
            // Every element of the i-th row of A Matrix has to be multiplied with every element of j-th column of matrix B
            R->data[i][j] = M->data[i][j] * scalar;
        }
    }
}

// End of Generic Matrix Functions

// Vector Functions

RasMatrix* RasMath::crossProd(RasMatrix *V1, RasMatrix *V2)
{
    if(V1->cols != 1 || V1->rows!=3 || V2->cols != 1 || V2->rows!=3)
    return 0;

    RasMatrix *R = new RasMatrix(3,1);

    R->data[0][0] =  V1->data[1][0]*V2->data[2][0] - V1->data[2][0]*V2->data[1][0];
    R->data[1][0] =  V1->data[2][0]*V2->data[0][0] - V1->data[0][0]*V2->data[2][0];
    R->data[2][0] =  V1->data[0][0]*V2->data[1][0] - V1->data[1][0]*V2->data[0][0];

    return R;
}

void RasMath::crossProd(RasMatrix *V1, RasMatrix *V2, RasMatrix *Vr)
{
    if(V1->cols != 1 || V1->rows!=3 || V2->cols != 1 || V2->rows!=3 || Vr->cols != 1 || Vr->rows!=3)
    return;

    Vr->data[0][0] =  V1->data[1][0]*V2->data[2][0] - V1->data[2][0]*V2->data[1][0];
    Vr->data[1][0] =  V1->data[2][0]*V2->data[0][0] - V1->data[0][0]*V2->data[2][0];
    Vr->data[2][0] =  V1->data[0][0]*V2->data[1][0] - V1->data[1][0]*V2->data[0][0];
}

// End of Vector Functions

// Coordinate Transform Functions

void RasMath::singleCoorTrans(double T[4][4], double P[4], double PNew[4])
{
    double actRow;
    for(int i=0; i<4; i++)
    {
        actRow = 0;
        for(int j=0; j<4; j++)
        {
            actRow += T[i][j] * P[j];
        }
        PNew[i] = actRow;
    }
}

// End of Coordinate Transform Functions

// Matrix Copying Functions

void RasMath::copyMatTrans(const double S[4][4], double D[4][4])
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            D[i][j] = S[i][j];
        }

    }
}

void RasMath::copyMatTrans(const double S[4][4], RasMatrix *D)
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            D->data[i][j] = S[i][j];
        }

    }
}

void RasMath::copyMatTrans(RasMatrix *S, double D[4][4])
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            D[i][j] = S->data[i][j];
        }

    }
}

void RasMath::copyMatRot(const double S[3][3], double D[3][3])
{
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            D[i][j] = S[i][j];
        }

    }
}

void RasMath::copyMatRot(const double S[3][3], RasMatrix *D)
{
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            D->data[i][j] = S[i][j];
        }

    }
}

void RasMath::copyMatRot(RasMatrix *S, double D[3][3])
{
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            D[i][j] = S->data[i][j];
        }

    }
}

void RasMath::copyVec(const double S[3], double D[3])
{
    for (int i=0; i<3; i++)
    {
        D[i] = S[i];
    }
}

void RasMath::copyRotFromTrans(RasMatrix *T, double R[3][3])
{
    R[0][0] = T->data[0][0]; R[0][1] = T->data[0][1]; R[0][2] = T->data[0][2];
    R[1][0] = T->data[1][0]; R[1][1] = T->data[1][1]; R[1][2] = T->data[1][2];
    R[2][0] = T->data[2][0]; R[2][1] = T->data[2][1]; R[2][2] = T->data[2][2];
}

void RasMath::copyRotFromTrans(const double T[4][4], RasMatrix *R)
{
    R->data[0][0] = T[0][0]; R->data[0][1] = T[0][1]; R->data[0][2] = T[0][2];
    R->data[1][0] = T[1][0]; R->data[1][1] = T[1][1]; R->data[1][2] = T[1][2];
    R->data[2][0] = T[2][0]; R->data[2][1] = T[2][1]; R->data[2][2] = T[2][2];
}

// End of Matrix Copying Functions

// Matrix Operations Functions

void RasMath::multTransMat(double M1[4][4],double M2[4][4],double T[4][4])
{
    for (int i=0; i<4; i++)
    {
        for (int j=0; j<4; j++)
        {
            T[i][j] = M1[i][0] * M2[0][j] + M1[i][1] * M2[1][j] + M1[i][2] * M2[2][j] + M1[i][3] * M2[3][j];
        }
    }
}

void RasMath::multRotMat(double M1[3][3],double M2[3][3],double R[3][3])
{
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            R[i][j] = M1[i][0] * M2[0][j] + M1[i][1] * M2[1][j] + M1[i][2] * M2[2][j];
        }
    }
}

// End of Matrix Operations Functions

// Matrix Creation Functions

void RasMath::setTransIdentity(double T[4][4])
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            T[i][j] = (i==j)? 1.0 : 0.0;
        }
    }
}

void RasMath::setIdentity(RasMatrix *M)
{
    if(M->cols != M->rows)
    return;

    for(int i=0; i<M->rows; i++)
    {
        for(int j=0; j<M->cols; j++)
        {
            M->data[i][j] = (i==j)? 1: 0;
        }
    }
}

void RasMath::convRxRyRz2RotM(double rxDeg, double ryDeg, double rzDeg, double Rot[3][3], int rotOrder)
{
    // X, Y, Z rotation angles are given in degrees
    // Is very important to remember that all rotations have to be right-handed
    // The order of the rotations is:
    // 0. XYZ
    // 1. ZXY -> Default because it is more associated with robotic arms
    // Taken from: http://en.wikipedia.org/wiki/Euler_angles

    double rx = deg2Rad(rxDeg);
    double ry = deg2Rad(ryDeg);
    double rz = deg2Rad(rzDeg);

    double Rx[3][3];
    double Ry[3][3];
    double Rz[3][3];

    Rx[0][0] = 1.0;     Rx[0][1] = 0.0    ; Rx[0][2] = 0.0;
    Rx[1][0] = 0.0;     Rx[1][1] = cos(rx); Rx[1][2] = -sin(rx);
    Rx[2][0] = 0.0;     Rx[2][1] = sin(rx); Rx[2][2] = cos(rx);

    Ry[0][0] = cos(ry) ;Ry[0][1] = 0.0;     Ry[0][2] = sin(ry);
    Ry[1][0] = 0.0     ;Ry[1][1] = 1.0;     Ry[1][2] = 0.0;
    Ry[2][0] = -sin(ry);Ry[2][1] = 0.0;     Ry[2][2] = cos(ry);

    Rz[0][0] = cos(rz); Rz[0][1] = -sin(rz);Rz[0][2] = 0.0;
    Rz[1][0] = sin(rz); Rz[1][1] = cos(rz) ;Rz[1][2] = 0.0;
    Rz[2][0] = 0.0    ; Rz[2][1] = 0.0     ;Rz[2][2] = 1.0;

    double RTemp[3][3];

    if(rotOrder == 0) // X-Y-Z
    {
        multRotMat(Rx,Ry,RTemp);
        multRotMat(RTemp,Rz,Rot);
    }

    if(rotOrder == 1) // Z-X-Y (Default)
    {
        multRotMat(Rz,Rx,RTemp);
        multRotMat(RTemp,Ry,Rot);
    }
}

void RasMath::convDH2Trans(double a, double alphaDeg, double d, double thetaDeg, double T[4][4])
{
    double alpha = deg2Rad(alphaDeg);
    double theta = deg2Rad(thetaDeg);

    T[0][0] = cos(theta); T[0][1] = -sin(theta)*cos(alpha); T[0][2] =  sin(theta)*sin(alpha); T[0][3] = a*cos(theta);
    T[1][0] = sin(theta); T[1][1] =  cos(theta)*cos(alpha); T[1][2] = -cos(theta)*sin(alpha); T[1][3] = a*sin(theta);
    T[2][0] =          0; T[2][1] =             sin(alpha); T[2][2] =             cos(alpha); T[2][3] =            d;
    T[3][0] =          0; T[3][1] =                      0; T[3][2] =                      0; T[3][3] =          1.0;
}

void RasMath::convDH2Trans(double a, double alphaDeg, double d, double thetaDeg, RasMatrix *T)
{
    double alpha = deg2Rad(alphaDeg);
    double theta = deg2Rad(thetaDeg);

    T->data[0][0] = cos(theta); T->data[0][1] = -sin(theta)*cos(alpha); T->data[0][2] =  sin(theta)*sin(alpha); T->data[0][3] = a*cos(theta);
    T->data[1][0] = sin(theta); T->data[1][1] =  cos(theta)*cos(alpha); T->data[1][2] = -cos(theta)*sin(alpha); T->data[1][3] = a*sin(theta);
    T->data[2][0] =          0; T->data[2][1] =             sin(alpha); T->data[2][2] =             cos(alpha); T->data[2][3] =            d;
    T->data[3][0] =          0; T->data[3][1] =                      0; T->data[3][2] =                      0; T->data[3][3] =          1.0;
}

// End of Matrix Creation Functions

// Matrix Conversion Functions

void RasMath::convRotTra2glMat(double R[3][3], double T[3], double glMat[16])
{
    glMat[0] = R[0][0]; glMat[4] = R[0][1]; glMat[ 8] = R[0][2]; glMat[12] = T[0];
    glMat[1] = R[1][0]; glMat[5] = R[1][1]; glMat[ 9] = R[1][2]; glMat[13] = T[1];
    glMat[2] = R[2][0]; glMat[6] = R[2][1]; glMat[10] = R[2][2]; glMat[14] = T[2];
    glMat[3] =       0; glMat[7] =       0; glMat[11] =       0; glMat[15] =  1.0;
}

void RasMath::convTrans2glMat(double T[4][4], double glMat[16])
{
    glMat[0] = T[0][0]; glMat[4] = T[0][1]; glMat[ 8] = T[0][2]; glMat[12] = T[0][3];
    glMat[1] = T[1][0]; glMat[5] = T[1][1]; glMat[ 9] = T[1][2]; glMat[13] = T[1][3];
    glMat[2] = T[2][0]; glMat[6] = T[2][1]; glMat[10] = T[2][2]; glMat[14] = T[2][3];
    glMat[3] =       0; glMat[7] =       0; glMat[11] =       0; glMat[15] =  1.0;
}

void RasMath::convTrans2glMat(RasMatrix *T, double glMat[16])
{
    glMat[0] = T->data[0][0]; glMat[4] = T->data[0][1]; glMat[ 8] = T->data[0][2]; glMat[12] = T->data[0][3];
    glMat[1] = T->data[1][0]; glMat[5] = T->data[1][1]; glMat[ 9] = T->data[1][2]; glMat[13] = T->data[1][3];
    glMat[2] = T->data[2][0]; glMat[6] = T->data[2][1]; glMat[10] = T->data[2][2]; glMat[14] = T->data[2][3];
    glMat[3] =             0; glMat[7] =             0; glMat[11] =             0; glMat[15] =  1.0;
}

void RasMath::convRotTra2Trans(double R[3][3], double T[3], double TT[4][4])
{
    TT[0][0] = R[0][0]; TT[0][1] = R[0][1]; TT[0][2] = R[0][2]; TT[0][3] = T[0];
    TT[1][0] = R[1][0]; TT[1][1] = R[1][1]; TT[1][2] = R[1][2]; TT[1][3] = T[1];
    TT[2][0] = R[2][0]; TT[2][1] = R[2][1]; TT[2][2] = R[2][2]; TT[2][3] = T[2];
    TT[3][0] =       0; TT[3][1] =       0; TT[3][2] =       0; TT[3][3] =  1.0;
}

void RasMath::convRotTra2Trans(double R[3][3], double T[3], RasMatrix *TT)
{
    TT->data[0][0] = R[0][0]; TT->data[0][1] = R[0][1]; TT->data[0][2] = R[0][2]; TT->data[0][3] = T[0];
    TT->data[1][0] = R[1][0]; TT->data[1][1] = R[1][1]; TT->data[1][2] = R[1][2]; TT->data[1][3] = T[1];
    TT->data[2][0] = R[2][0]; TT->data[2][1] = R[2][1]; TT->data[2][2] = R[2][2]; TT->data[2][3] = T[2];
    TT->data[3][0] =       0; TT->data[3][1] =       0; TT->data[3][2] =       0; TT->data[3][3] =  1.0;
}

void RasMath::convTrans2RotTra(double TT[4][4], double R[3][3], double T[3])
{
    R[0][0] = TT[0][0]; R[0][1] = TT[0][1]; R[0][2] = TT[0][2]; T[0] = TT[0][3];
    R[1][0] = TT[1][0]; R[1][1] = TT[1][1]; R[1][2] = TT[1][2]; T[1] = TT[1][3];
    R[2][0] = TT[2][0]; R[2][1] = TT[2][1]; R[2][2] = TT[2][2]; T[2] = TT[2][3];
}

void RasMath::convTrans2RotTra(RasMatrix *TT, double R[3][3], double T[3])
{
    R[0][0] = TT->data[0][0]; R[0][1] = TT->data[0][1]; R[0][2] = TT->data[0][2]; T[0] = TT->data[0][3];
    R[1][0] = TT->data[1][0]; R[1][1] = TT->data[1][1]; R[1][2] = TT->data[1][2]; T[1] = TT->data[1][3];
    R[2][0] = TT->data[2][0]; R[2][1] = TT->data[2][1]; R[2][2] = TT->data[2][2]; T[2] = TT->data[2][3];
}

void RasMath::convRot2RPY(const double R[3][3], double &roll, double &pitch, double &yaw)
{
    yaw = rad2Deg(atan2(R[1][0],R[0][0]));
    pitch = rad2Deg(atan2(-R[2][0],sqrt(R[2][1]*R[2][1]+R[2][2]*R[2][2])));
    roll = rad2Deg(atan2(R[2][1],R[2][2]));

    if(fabs(yaw) < 1E-12)
    yaw = 0;

    if(fabs(pitch) < 1E-12)
    pitch = 0;

    if(fabs(roll) < 1E-12)
    roll = 0;
}

// End of Matrix Conversion Functions

// Joint pair function

double RasMath::derFunErrRot(double TPre[4][4], double TNex[4][4], double xf, double yf, double zf, double a, double alphaDeg, double d, double thetaDeg)
{
    double alpha = deg2Rad(alphaDeg);
    double theta = deg2Rad(thetaDeg);

    return 2*(TNex[0][3]*(TPre[0][1]*cos(theta) - TPre[0][0]*sin(theta)) - TNex[1][3]*(TPre[0][0]*cos(alpha)*cos(theta) + TPre[0][1]*cos(alpha)*sin(theta)) + TNex[2][3]*(TPre[0][0]*sin(alpha)*cos(theta) + TPre[0][1]*sin(alpha)*sin(theta)) + TPre[0][1]*a*cos(theta) - TPre[0][0]*a*sin(theta))*(TPre[0][3] - xf + TPre[0][2]*d + TNex[0][3]*(TPre[0][0]*cos(theta) + TPre[0][1]*sin(theta)) + TNex[1][3]*(TPre[0][2]*sin(alpha) + TPre[0][1]*cos(alpha)*cos(theta) - TPre[0][0]*cos(alpha)*sin(theta)) + TNex[2][3]*(TPre[0][2]*cos(alpha) - TPre[0][1]*sin(alpha)*cos(theta) + TPre[0][0]*sin(alpha)*sin(theta)) + TPre[0][0]*a*cos(theta) + TPre[0][1]*a*sin(theta)) + 2*(TNex[0][3]*(TPre[1][1]*cos(theta) - TPre[1][0]*sin(theta)) - TNex[1][3]*(TPre[1][0]*cos(alpha)*cos(theta) + TPre[1][1]*cos(alpha)*sin(theta)) + TNex[2][3]*(TPre[1][0]*sin(alpha)*cos(theta) + TPre[1][1]*sin(alpha)*sin(theta)) + TPre[1][1]*a*cos(theta) - TPre[1][0]*a*sin(theta))*(TPre[1][3] - yf + TPre[1][2]*d + TNex[0][3]*(TPre[1][0]*cos(theta) + TPre[1][1]*sin(theta)) + TNex[1][3]*(TPre[1][2]*sin(alpha) + TPre[1][1]*cos(alpha)*cos(theta) - TPre[1][0]*cos(alpha)*sin(theta)) + TNex[2][3]*(TPre[1][2]*cos(alpha) - TPre[1][1]*sin(alpha)*cos(theta) + TPre[1][0]*sin(alpha)*sin(theta)) + TPre[1][0]*a*cos(theta) + TPre[1][1]*a*sin(theta)) + 2*(TNex[0][3]*(TPre[2][1]*cos(theta) - TPre[2][0]*sin(theta)) - TNex[1][3]*(TPre[2][0]*cos(alpha)*cos(theta) + TPre[2][1]*cos(alpha)*sin(theta)) + TNex[2][3]*(TPre[2][0]*sin(alpha)*cos(theta) + TPre[2][1]*sin(alpha)*sin(theta)) + TPre[2][1]*a*cos(theta) - TPre[2][0]*a*sin(theta))*(TPre[2][3] - zf + TPre[2][2]*d + TNex[0][3]*(TPre[2][0]*cos(theta) + TPre[2][1]*sin(theta)) + TNex[1][3]*(TPre[2][2]*sin(alpha) + TPre[2][1]*cos(alpha)*cos(theta) - TPre[2][0]*cos(alpha)*sin(theta)) + TNex[2][3]*(TPre[2][2]*cos(alpha) - TPre[2][1]*sin(alpha)*cos(theta) + TPre[2][0]*sin(alpha)*sin(theta)) + TPre[2][0]*a*cos(theta) + TPre[2][1]*a*sin(theta));
}

double RasMath::derFunErrPri(double TPre[4][4], double TNex[4][4], double xf, double yf, double zf, double a, double alphaDeg, double d, double thetaDeg)
{
    double alpha = deg2Rad(alphaDeg);
    double theta = deg2Rad(thetaDeg);

    return 2*TPre[0][2]*(TPre[0][3] - xf + TPre[0][2]*d + TNex[0][3]*(TPre[0][0]*cos(theta) + TPre[0][1]*sin(theta)) + TNex[1][3]*(TPre[0][2]*sin(alpha) + TPre[0][1]*cos(alpha)*cos(theta) - TPre[0][0]*cos(alpha)*sin(theta)) + TNex[2][3]*(TPre[0][2]*cos(alpha) - TPre[0][1]*sin(alpha)*cos(theta) + TPre[0][0]*sin(alpha)*sin(theta)) + TPre[0][0]*a*cos(theta) + TPre[0][1]*a*sin(theta)) + 2*TPre[1][2]*(TPre[1][3] - yf + TPre[1][2]*d + TNex[0][3]*(TPre[1][0]*cos(theta) + TPre[1][1]*sin(theta)) + TNex[1][3]*(TPre[1][2]*sin(alpha) + TPre[1][1]*cos(alpha)*cos(theta) - TPre[1][0]*cos(alpha)*sin(theta)) + TNex[2][3]*(TPre[1][2]*cos(alpha) - TPre[1][1]*sin(alpha)*cos(theta) + TPre[1][0]*sin(alpha)*sin(theta)) + TPre[1][0]*a*cos(theta) + TPre[1][1]*a*sin(theta)) + 2*TPre[2][2]*(TPre[2][3] - zf + TPre[2][2]*d + TNex[0][3]*(TPre[2][0]*cos(theta) + TPre[2][1]*sin(theta)) + TNex[1][3]*(TPre[2][2]*sin(alpha) + TPre[2][1]*cos(alpha)*cos(theta) - TPre[2][0]*cos(alpha)*sin(theta)) + TNex[2][3]*(TPre[2][2]*cos(alpha) - TPre[2][1]*sin(alpha)*cos(theta) + TPre[2][0]*sin(alpha)*sin(theta)) + TPre[2][0]*a*cos(theta) + TPre[2][1]*a*sin(theta));
}

void RasMath::newtonRR(double &derErr, double &x, double &y, double TPrev[4][4], double TNext[4][4], double xf, double yf, double zf, double a1, double alpha1Deg, double d1, double theta1Deg, double a2, double alpha2Deg, double d2, double theta2Deg)
{
    double sO1 = sin(deg2Rad(theta1Deg));
    double cO1 = cos(deg2Rad(theta1Deg));
    double sA1 = sin(deg2Rad(alpha1Deg));
    double cA1 = cos(deg2Rad(alpha1Deg));

    double sO2 = sin(deg2Rad(theta2Deg));
    double cO2 = cos(deg2Rad(theta2Deg));
    double sA2 = sin(deg2Rad(alpha2Deg));
    double cA2 = cos(deg2Rad(alpha2Deg));
    
    double Ta11, Ta12, Ta13, Ta14, Ta21, Ta22, Ta23, Ta24, Ta31, Ta32, Ta33, Ta34;
    double Tb11, Tb12, Tb13, Tb14, Tb21, Tb22, Tb23, Tb24, Tb31, Tb32, Tb33, Tb34;

    Ta11 = TPrev[0][0]; Ta12 = TPrev[0][1]; Ta13 = TPrev[0][2]; Ta14 = TPrev[0][3];
    Ta21 = TPrev[1][0]; Ta22 = TPrev[1][1]; Ta23 = TPrev[1][2]; Ta24 = TPrev[1][3];
    Ta31 = TPrev[2][0]; Ta32 = TPrev[2][1]; Ta33 = TPrev[2][2]; Ta34 = TPrev[2][3];

    Tb11 = TNext[0][0]; Tb12 = TNext[0][1]; Tb13 = TNext[0][2]; Tb14 = TNext[0][3];
    Tb21 = TNext[1][0]; Tb22 = TNext[1][1]; Tb23 = TNext[1][2]; Tb24 = TNext[1][3];
    Tb31 = TNext[2][0]; Tb32 = TNext[2][1]; Tb33 = TNext[2][2]; Tb34 = TNext[2][3];
    
    double fR = - 2*(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1);
    double gR = - 2*(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1);
    
    double JInvRR11 = -1/(pow(2*(Tb14*(cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sO2*(Ta12*cO1 - Ta11*sO1)) - Tb24*(cA2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cA2*cO2*(Ta12*cO1 - Ta11*sO1)) + Tb34*(sA2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*cO2*(Ta12*cO1 - Ta11*sO1)) + a2*sO2*(Ta12*cO1 - Ta11*sO1) + a2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sO2*(Ta22*cO1 - Ta21*sO1)) - Tb24*(cA2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cA2*cO2*(Ta22*cO1 - Ta21*sO1)) + Tb34*(sA2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*cO2*(Ta22*cO1 - Ta21*sO1)) + a2*sO2*(Ta22*cO1 - Ta21*sO1) + a2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sO2*(Ta32*cO1 - Ta31*sO1)) - Tb24*(cA2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cA2*cO2*(Ta32*cO1 - Ta31*sO1)) + Tb34*(sA2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*cO2*(Ta32*cO1 - Ta31*sO1)) + a2*sO2*(Ta32*cO1 - Ta31*sO1) + a2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1),2) + (2*(Tb14*(sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cO2*(Ta11*cO1 + Ta12*sO1)) - Tb34*(cA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + sA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) - sA2*sO2*(Ta11*cO1 + Ta12*sO1)) - Tb24*(sA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) - cA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cA2*sO2*(Ta11*cO1 + Ta12*sO1)) - d2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cO2*(Ta21*cO1 + Ta22*sO1)) - Tb34*(cA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + sA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) - sA2*sO2*(Ta21*cO1 + Ta22*sO1)) - Tb24*(sA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) - cA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cA2*sO2*(Ta21*cO1 + Ta22*sO1)) - d2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cO2*(Ta31*cO1 + Ta32*sO1)) - Tb34*(cA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + sA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) - sA2*sO2*(Ta31*cO1 + Ta32*sO1)) - Tb24*(sA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) - cA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cA2*sO2*(Ta31*cO1 + Ta32*sO1)) - d2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*pow(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1,2))*(2*pow(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1),2) - 2*(Tb34*(sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb24*(cA2*sO2*(Ta11*cO1 + Ta12*sO1) - cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb24*(cA2*sO2*(Ta21*cO1 + Ta22*sO1) - cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb24*(cA2*sO2*(Ta31*cO1 + Ta32*sO1) - cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)))*(2*pow(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1),2) - 2*(Tb34*(sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb24*(cA2*sO2*(Ta11*cO1 + Ta12*sO1) - cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb24*(cA2*sO2*(Ta21*cO1 + Ta22*sO1) - cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb24*(cA2*sO2*(Ta31*cO1 + Ta32*sO1) - cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1));
    double JInvRR12 = -1/(pow(2*(Tb14*(cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sO2*(Ta12*cO1 - Ta11*sO1)) - Tb24*(cA2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cA2*cO2*(Ta12*cO1 - Ta11*sO1)) + Tb34*(sA2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*cO2*(Ta12*cO1 - Ta11*sO1)) + a2*sO2*(Ta12*cO1 - Ta11*sO1) + a2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sO2*(Ta22*cO1 - Ta21*sO1)) - Tb24*(cA2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cA2*cO2*(Ta22*cO1 - Ta21*sO1)) + Tb34*(sA2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*cO2*(Ta22*cO1 - Ta21*sO1)) + a2*sO2*(Ta22*cO1 - Ta21*sO1) + a2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sO2*(Ta32*cO1 - Ta31*sO1)) - Tb24*(cA2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cA2*cO2*(Ta32*cO1 - Ta31*sO1)) + Tb34*(sA2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*cO2*(Ta32*cO1 - Ta31*sO1)) + a2*sO2*(Ta32*cO1 - Ta31*sO1) + a2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1),2) + (2*(Tb14*(sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cO2*(Ta11*cO1 + Ta12*sO1)) - Tb34*(cA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + sA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) - sA2*sO2*(Ta11*cO1 + Ta12*sO1)) - Tb24*(sA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) - cA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cA2*sO2*(Ta11*cO1 + Ta12*sO1)) - d2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cO2*(Ta21*cO1 + Ta22*sO1)) - Tb34*(cA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + sA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) - sA2*sO2*(Ta21*cO1 + Ta22*sO1)) - Tb24*(sA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) - cA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cA2*sO2*(Ta21*cO1 + Ta22*sO1)) - d2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cO2*(Ta31*cO1 + Ta32*sO1)) - Tb34*(cA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + sA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) - sA2*sO2*(Ta31*cO1 + Ta32*sO1)) - Tb24*(sA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) - cA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cA2*sO2*(Ta31*cO1 + Ta32*sO1)) - d2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*pow(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1,2))*(2*pow(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1),2) - 2*(Tb34*(sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb24*(cA2*sO2*(Ta11*cO1 + Ta12*sO1) - cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb24*(cA2*sO2*(Ta21*cO1 + Ta22*sO1) - cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb24*(cA2*sO2*(Ta31*cO1 + Ta32*sO1) - cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)))*(2*(Tb14*(cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sO2*(Ta12*cO1 - Ta11*sO1)) - Tb24*(cA2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cA2*cO2*(Ta12*cO1 - Ta11*sO1)) + Tb34*(sA2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*cO2*(Ta12*cO1 - Ta11*sO1)) + a2*sO2*(Ta12*cO1 - Ta11*sO1) + a2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sO2*(Ta22*cO1 - Ta21*sO1)) - Tb24*(cA2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cA2*cO2*(Ta22*cO1 - Ta21*sO1)) + Tb34*(sA2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*cO2*(Ta22*cO1 - Ta21*sO1)) + a2*sO2*(Ta22*cO1 - Ta21*sO1) + a2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sO2*(Ta32*cO1 - Ta31*sO1)) - Tb24*(cA2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cA2*cO2*(Ta32*cO1 - Ta31*sO1)) + Tb34*(sA2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*cO2*(Ta32*cO1 - Ta31*sO1)) + a2*sO2*(Ta32*cO1 - Ta31*sO1) + a2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1));
    double JInvRR21 = JInvRR12;
    double JInvRR22 = 1/(pow(2*(Tb14*(cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sO2*(Ta12*cO1 - Ta11*sO1)) - Tb24*(cA2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cA2*cO2*(Ta12*cO1 - Ta11*sO1)) + Tb34*(sA2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*cO2*(Ta12*cO1 - Ta11*sO1)) + a2*sO2*(Ta12*cO1 - Ta11*sO1) + a2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sO2*(Ta22*cO1 - Ta21*sO1)) - Tb24*(cA2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cA2*cO2*(Ta22*cO1 - Ta21*sO1)) + Tb34*(sA2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*cO2*(Ta22*cO1 - Ta21*sO1)) + a2*sO2*(Ta22*cO1 - Ta21*sO1) + a2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sO2*(Ta32*cO1 - Ta31*sO1)) - Tb24*(cA2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cA2*cO2*(Ta32*cO1 - Ta31*sO1)) + Tb34*(sA2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*cO2*(Ta32*cO1 - Ta31*sO1)) + a2*sO2*(Ta32*cO1 - Ta31*sO1) + a2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1),2) + (2*(Tb14*(sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cO2*(Ta11*cO1 + Ta12*sO1)) - Tb34*(cA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + sA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) - sA2*sO2*(Ta11*cO1 + Ta12*sO1)) - Tb24*(sA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) - cA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cA2*sO2*(Ta11*cO1 + Ta12*sO1)) - d2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cO2*(Ta21*cO1 + Ta22*sO1)) - Tb34*(cA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + sA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) - sA2*sO2*(Ta21*cO1 + Ta22*sO1)) - Tb24*(sA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) - cA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cA2*sO2*(Ta21*cO1 + Ta22*sO1)) - d2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cO2*(Ta31*cO1 + Ta32*sO1)) - Tb34*(cA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + sA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) - sA2*sO2*(Ta31*cO1 + Ta32*sO1)) - Tb24*(sA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) - cA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cA2*sO2*(Ta31*cO1 + Ta32*sO1)) - d2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*pow(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1,2))*(2*pow(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1),2) - 2*(Tb34*(sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb24*(cA2*sO2*(Ta11*cO1 + Ta12*sO1) - cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb24*(cA2*sO2*(Ta21*cO1 + Ta22*sO1) - cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb24*(cA2*sO2*(Ta31*cO1 + Ta32*sO1) - cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)))*(2*(Tb14*(sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cO2*(Ta11*cO1 + Ta12*sO1)) - Tb34*(cA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + sA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) - sA2*sO2*(Ta11*cO1 + Ta12*sO1)) - Tb24*(sA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) - cA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cA2*sO2*(Ta11*cO1 + Ta12*sO1)) - d2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cO2*(Ta21*cO1 + Ta22*sO1)) - Tb34*(cA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + sA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) - sA2*sO2*(Ta21*cO1 + Ta22*sO1)) - Tb24*(sA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) - cA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cA2*sO2*(Ta21*cO1 + Ta22*sO1)) - d2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cO2*(Ta31*cO1 + Ta32*sO1)) - Tb34*(cA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + sA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) - sA2*sO2*(Ta31*cO1 + Ta32*sO1)) - Tb24*(sA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) - cA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cA2*sO2*(Ta31*cO1 + Ta32*sO1)) - d2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*pow(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1,2));
    
    x = theta1Deg - (JInvRR11*fR + JInvRR12*gR);
    y = theta2Deg - (JInvRR21*fR + JInvRR22*gR);

    derErr = sqrt(pow(fR,2) + pow(gR,2));
}

void RasMath::newtonRP(double &derErr, double &x, double &y, double TPrev[4][4], double TNext[4][4], double xf, double yf, double zf, double a1, double alpha1Deg, double d1, double theta1Deg, double a2, double alpha2Deg, double d2, double theta2Deg)
{
    double sO1 = sin(deg2Rad(theta1Deg));
    double cO1 = cos(deg2Rad(theta1Deg));
    double sA1 = sin(deg2Rad(alpha1Deg));
    double cA1 = cos(deg2Rad(alpha1Deg));

    double sO2 = sin(deg2Rad(theta2Deg));
    double cO2 = cos(deg2Rad(theta2Deg));
    double sA2 = sin(deg2Rad(alpha2Deg));
    double cA2 = cos(deg2Rad(alpha2Deg));

    double Ta11, Ta12, Ta13, Ta14, Ta21, Ta22, Ta23, Ta24, Ta31, Ta32, Ta33, Ta34;
    double Tb11, Tb12, Tb13, Tb14, Tb21, Tb22, Tb23, Tb24, Tb31, Tb32, Tb33, Tb34;

    Ta11 = TPrev[0][0]; Ta12 = TPrev[0][1]; Ta13 = TPrev[0][2]; Ta14 = TPrev[0][3];
    Ta21 = TPrev[1][0]; Ta22 = TPrev[1][1]; Ta23 = TPrev[1][2]; Ta24 = TPrev[1][3];
    Ta31 = TPrev[2][0]; Ta32 = TPrev[2][1]; Ta33 = TPrev[2][2]; Ta34 = TPrev[2][3];

    Tb11 = TNext[0][0]; Tb12 = TNext[0][1]; Tb13 = TNext[0][2]; Tb14 = TNext[0][3];
    Tb21 = TNext[1][0]; Tb22 = TNext[1][1]; Tb23 = TNext[1][2]; Tb24 = TNext[1][3];
    Tb31 = TNext[2][0]; Tb32 = TNext[2][1]; Tb33 = TNext[2][2]; Tb34 = TNext[2][3];
    
    double fR = - 2*(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1);
    double gP = 2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1);
    
    double JInvRP11 = -1/((2*pow(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1,2) + 2*pow(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1,2) + 2*pow(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1,2))*(2*(Tb14*(sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cO2*(Ta11*cO1 + Ta12*sO1)) - Tb34*(cA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + sA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) - sA2*sO2*(Ta11*cO1 + Ta12*sO1)) - Tb24*(sA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) - cA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cA2*sO2*(Ta11*cO1 + Ta12*sO1)) - d2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cO2*(Ta21*cO1 + Ta22*sO1)) - Tb34*(cA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + sA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) - sA2*sO2*(Ta21*cO1 + Ta22*sO1)) - Tb24*(sA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) - cA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cA2*sO2*(Ta21*cO1 + Ta22*sO1)) - d2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cO2*(Ta31*cO1 + Ta32*sO1)) - Tb34*(cA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + sA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) - sA2*sO2*(Ta31*cO1 + Ta32*sO1)) - Tb24*(sA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) - cA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cA2*sO2*(Ta31*cO1 + Ta32*sO1)) - d2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*pow(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1,2)) + pow(2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1)*(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1) + 2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1)*(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1) + 2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1)*(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1) - 2*(Ta11*sA1*cO1 + Ta12*sA1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Ta21*sA1*cO1 + Ta22*sA1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Ta31*sA1*cO1 + Ta32*sA1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1),2))*(2*pow(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1,2) + 2*pow(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1,2) + 2*pow(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1,2));
    double JInvRP12 = 1/((2*pow(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1,2) + 2*pow(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1,2) + 2*pow(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1,2))*(2*(Tb14*(sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cO2*(Ta11*cO1 + Ta12*sO1)) - Tb34*(cA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + sA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) - sA2*sO2*(Ta11*cO1 + Ta12*sO1)) - Tb24*(sA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) - cA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cA2*sO2*(Ta11*cO1 + Ta12*sO1)) - d2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cO2*(Ta21*cO1 + Ta22*sO1)) - Tb34*(cA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + sA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) - sA2*sO2*(Ta21*cO1 + Ta22*sO1)) - Tb24*(sA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) - cA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cA2*sO2*(Ta21*cO1 + Ta22*sO1)) - d2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cO2*(Ta31*cO1 + Ta32*sO1)) - Tb34*(cA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + sA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) - sA2*sO2*(Ta31*cO1 + Ta32*sO1)) - Tb24*(sA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) - cA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cA2*sO2*(Ta31*cO1 + Ta32*sO1)) - d2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*pow(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1,2)) + pow(2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1)*(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1) + 2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1)*(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1) + 2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1)*(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1) - 2*(Ta11*sA1*cO1 + Ta12*sA1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Ta21*sA1*cO1 + Ta22*sA1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Ta31*sA1*cO1 + Ta32*sA1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1),2))*(2*(Ta11*sA1*cO1 + Ta12*sA1*sO1)*(Ta14 - xf + Ta13*d1 - Tb24*(cA2*sO2*(Ta11*cO1 + Ta12*sO1) - cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + Tb34*cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Ta21*sA1*cO1 + Ta22*sA1*sO1)*(Ta24 - yf + Ta23*d1 - Tb24*(cA2*sO2*(Ta21*cO1 + Ta22*sO1) - cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + Tb34*cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Ta31*sA1*cO1 + Ta32*sA1*sO1)*(Ta34 - zf + Ta33*d1 - Tb24*(cA2*sO2*(Ta31*cO1 + Ta32*sO1) - cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + Tb34*cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1)*(Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) + Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - a2*cO2*(Ta12*cO1 - Ta11*sO1) - Tb34*cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1) - 2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1)*(Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) + Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - a2*cO2*(Ta22*cO1 - Ta21*sO1) - Tb34*cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1) - 2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1)*(Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) + Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - a2*cO2*(Ta32*cO1 - Ta31*sO1) - Tb34*cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1) + d2*(4*(Ta11*sA1*cO1 + Ta12*sA1*sO1)*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + 4*(Ta21*sA1*cO1 + Ta22*sA1*sO1)*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + 4*(Ta31*sA1*cO1 + Ta32*sA1*sO1)*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1)) + sA2*(2*(Tb34*(cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sO2*(Ta12*cO1 - Ta11*sO1)) + Tb24*(Ta11*sA1*cO1 + Ta12*sA1*sO1))*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + 2*(Tb34*(cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sO2*(Ta22*cO1 - Ta21*sO1)) + Tb24*(Ta21*sA1*cO1 + Ta22*sA1*sO1))*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + 2*(Tb34*(cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sO2*(Ta32*cO1 - Ta31*sO1)) + Tb24*(Ta31*sA1*cO1 + Ta32*sA1*sO1))*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + 2*(Ta11*sA1*cO1 + Ta12*sA1*sO1)*(Tb24*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb34*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))) + 2*(Ta21*sA1*cO1 + Ta22*sA1*sO1)*(Tb24*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb34*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))) + 2*(Ta31*sA1*cO1 + Ta32*sA1*sO1)*(Tb24*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb34*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)))));
    double JInvRP21 = JInvRP12;
    double JInvRP22 = 1/((2*pow(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1,2) + 2*pow(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1,2) + 2*pow(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1,2))*(2*(Tb14*(sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cO2*(Ta11*cO1 + Ta12*sO1)) - Tb34*(cA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + sA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) - sA2*sO2*(Ta11*cO1 + Ta12*sO1)) - Tb24*(sA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) - cA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cA2*sO2*(Ta11*cO1 + Ta12*sO1)) - d2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cO2*(Ta21*cO1 + Ta22*sO1)) - Tb34*(cA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + sA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) - sA2*sO2*(Ta21*cO1 + Ta22*sO1)) - Tb24*(sA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) - cA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cA2*sO2*(Ta21*cO1 + Ta22*sO1)) - d2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cO2*(Ta31*cO1 + Ta32*sO1)) - Tb34*(cA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + sA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) - sA2*sO2*(Ta31*cO1 + Ta32*sO1)) - Tb24*(sA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) - cA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cA2*sO2*(Ta31*cO1 + Ta32*sO1)) - d2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*pow(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1,2)) + pow(2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1)*(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1) + 2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1)*(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1) + 2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1)*(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1) - 2*(Ta11*sA1*cO1 + Ta12*sA1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Ta21*sA1*cO1 + Ta22*sA1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Ta31*sA1*cO1 + Ta32*sA1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1),2))*(2*(Tb14*(sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cO2*(Ta11*cO1 + Ta12*sO1)) - Tb34*(cA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + sA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) - sA2*sO2*(Ta11*cO1 + Ta12*sO1)) - Tb24*(sA2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) - cA2*cO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + cA2*sO2*(Ta11*cO1 + Ta12*sO1)) - d2*(Ta12*sA1*cO1 - Ta11*sA1*sO1) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Tb14*(sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cO2*(Ta21*cO1 + Ta22*sO1)) - Tb34*(cA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + sA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) - sA2*sO2*(Ta21*cO1 + Ta22*sO1)) - Tb24*(sA2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) - cA2*cO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + cA2*sO2*(Ta21*cO1 + Ta22*sO1)) - d2*(Ta22*sA1*cO1 - Ta21*sA1*sO1) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Tb14*(sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cO2*(Ta31*cO1 + Ta32*sO1)) - Tb34*(cA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + sA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) - sA2*sO2*(Ta31*cO1 + Ta32*sO1)) - Tb24*(sA2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) - cA2*cO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + cA2*sO2*(Ta31*cO1 + Ta32*sO1)) - d2*(Ta32*sA1*cO1 - Ta31*sA1*sO1) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1) - 2*pow(Tb24*(cA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - sA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + cA2*sO2*(Ta12*cO1 - Ta11*sO1)) - Tb34*(cA2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) + sA2*cO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) + sA2*sO2*(Ta12*cO1 - Ta11*sO1)) + Tb14*(sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - cO2*(Ta12*cO1 - Ta11*sO1)) - d2*(Ta11*sA1*cO1 + Ta12*sA1*sO1) - a2*cO2*(Ta12*cO1 - Ta11*sO1) + a2*sO2*(Ta11*cA1*cO1 + Ta12*cA1*sO1) - Ta12*a1*cO1 + Ta11*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - sA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + cA2*sO2*(Ta22*cO1 - Ta21*sO1)) - Tb34*(cA2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) + sA2*cO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) + sA2*sO2*(Ta22*cO1 - Ta21*sO1)) + Tb14*(sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - cO2*(Ta22*cO1 - Ta21*sO1)) - d2*(Ta21*sA1*cO1 + Ta22*sA1*sO1) - a2*cO2*(Ta22*cO1 - Ta21*sO1) + a2*sO2*(Ta21*cA1*cO1 + Ta22*cA1*sO1) - Ta22*a1*cO1 + Ta21*a1*sO1,2) - 2*pow(Tb24*(cA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - sA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + cA2*sO2*(Ta32*cO1 - Ta31*sO1)) - Tb34*(cA2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) + sA2*cO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) + sA2*sO2*(Ta32*cO1 - Ta31*sO1)) + Tb14*(sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - cO2*(Ta32*cO1 - Ta31*sO1)) - d2*(Ta31*sA1*cO1 + Ta32*sA1*sO1) - a2*cO2*(Ta32*cO1 - Ta31*sO1) + a2*sO2*(Ta31*cA1*cO1 + Ta32*cA1*sO1) - Ta32*a1*cO1 + Ta31*a1*sO1,2));
    
    x = theta1Deg - (JInvRP11*fR + JInvRP12*gP);
    y = d2        - (JInvRP21*fR + JInvRP22*gP);

    derErr = sqrt(pow(fR,2) + pow(gP,2));
}

void RasMath::newtonPR(double &derErr, double &x, double &y, double TPrev[4][4], double TNext[4][4], double xf, double yf, double zf, double a1, double alpha1Deg, double d1, double theta1Deg, double a2, double alpha2Deg, double d2, double theta2Deg)
{
    double sO1 = sin(deg2Rad(theta1Deg));
    double cO1 = cos(deg2Rad(theta1Deg));
    double sA1 = sin(deg2Rad(alpha1Deg));
    double cA1 = cos(deg2Rad(alpha1Deg));

    double sO2 = sin(deg2Rad(theta2Deg));
    double cO2 = cos(deg2Rad(theta2Deg));
    double sA2 = sin(deg2Rad(alpha2Deg));
    double cA2 = cos(deg2Rad(alpha2Deg));

    double Ta11, Ta12, Ta13, Ta14, Ta21, Ta22, Ta23, Ta24, Ta31, Ta32, Ta33, Ta34;
    double Tb11, Tb12, Tb13, Tb14, Tb21, Tb22, Tb23, Tb24, Tb31, Tb32, Tb33, Tb34;

    Ta11 = TPrev[0][0]; Ta12 = TPrev[0][1]; Ta13 = TPrev[0][2]; Ta14 = TPrev[0][3];
    Ta21 = TPrev[1][0]; Ta22 = TPrev[1][1]; Ta23 = TPrev[1][2]; Ta24 = TPrev[1][3];
    Ta31 = TPrev[2][0]; Ta32 = TPrev[2][1]; Ta33 = TPrev[2][2]; Ta34 = TPrev[2][3];

    Tb11 = TNext[0][0]; Tb12 = TNext[0][1]; Tb13 = TNext[0][2]; Tb14 = TNext[0][3];
    Tb21 = TNext[1][0]; Tb22 = TNext[1][1]; Tb23 = TNext[1][2]; Tb24 = TNext[1][3];
    Tb31 = TNext[2][0]; Tb32 = TNext[2][1]; Tb33 = TNext[2][2]; Tb34 = TNext[2][3];
    
    double fP = 2*Ta13*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*Ta23*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*Ta33*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1);
    double gR = - 2*(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1);
    
    double JInvPR11 = 1/((2*pow(Ta13,2) + 2*pow(Ta23,2) + 2*pow(Ta33,2))*(2*pow(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1),2) - 2*(Tb34*(sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb24*(cA2*sO2*(Ta11*cO1 + Ta12*sO1) - cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb24*(cA2*sO2*(Ta21*cO1 + Ta22*sO1) - cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb24*(cA2*sO2*(Ta31*cO1 + Ta32*sO1) - cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)) - pow(2*Ta13*(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + 2*Ta23*(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + 2*Ta33*(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)),2))*(2*pow(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1),2) - 2*(Tb34*(sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb24*(cA2*sO2*(Ta11*cO1 + Ta12*sO1) - cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb24*(cA2*sO2*(Ta21*cO1 + Ta22*sO1) - cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb24*(cA2*sO2*(Ta31*cO1 + Ta32*sO1) - cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1));
    double JInvPR12 = 1/((2*pow(Ta13,2) + 2*pow(Ta23,2) + 2*pow(Ta33,2))*(2*pow(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1),2) - 2*(Tb34*(sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb24*(cA2*sO2*(Ta11*cO1 + Ta12*sO1) - cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb24*(cA2*sO2*(Ta21*cO1 + Ta22*sO1) - cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb24*(cA2*sO2*(Ta31*cO1 + Ta32*sO1) - cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)) - pow(2*Ta13*(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + 2*Ta23*(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + 2*Ta33*(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)),2))*(a2*(2*Ta13*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + 2*Ta23*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + 2*Ta33*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))) + 2*Ta13*(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))) + 2*Ta23*(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))) + 2*Ta33*(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))));
    double JInvPR21 = JInvPR12;
    double JInvPR22 = 1/((2*pow(Ta13,2) + 2*pow(Ta23,2) + 2*pow(Ta33,2))*(2*pow(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1),2) + 2*pow(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1),2) - 2*(Tb34*(sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb24*(cA2*sO2*(Ta11*cO1 + Ta12*sO1) - cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1))*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb24*(cA2*sO2*(Ta21*cO1 + Ta22*sO1) - cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1))*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) - 2*(Tb34*(sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb24*(cA2*sO2*(Ta31*cO1 + Ta32*sO1) - cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1))*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1)) - pow(2*Ta13*(Tb24*(cA2*cO2*(Ta11*cO1 + Ta12*sO1) + cA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) - Tb34*(sA2*cO2*(Ta11*cO1 + Ta12*sO1) + sA2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb14*(sO2*(Ta11*cO1 + Ta12*sO1) - cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*sO2*(Ta11*cO1 + Ta12*sO1) - a2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + 2*Ta23*(Tb24*(cA2*cO2*(Ta21*cO1 + Ta22*sO1) + cA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) - Tb34*(sA2*cO2*(Ta21*cO1 + Ta22*sO1) + sA2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb14*(sO2*(Ta21*cO1 + Ta22*sO1) - cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*sO2*(Ta21*cO1 + Ta22*sO1) - a2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + 2*Ta33*(Tb24*(cA2*cO2*(Ta31*cO1 + Ta32*sO1) + cA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) - Tb34*(sA2*cO2*(Ta31*cO1 + Ta32*sO1) + sA2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb14*(sO2*(Ta31*cO1 + Ta32*sO1) - cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*sO2*(Ta31*cO1 + Ta32*sO1) - a2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)),2))*(2*pow(Ta13,2) + 2*pow(Ta23,2) + 2*pow(Ta33,2));
    
    x = d1        - (JInvPR11*fP + JInvPR12*gR);
    y = theta2Deg - (JInvPR21*fP + JInvPR22*gR);

    derErr = sqrt(pow(fP,2) + pow(gR,2));
}

void RasMath::newtonPP(double &derErr, double &x, double &y, double TPrev[4][4], double TNext[4][4], double xf, double yf, double zf, double a1, double alpha1Deg, double d1, double theta1Deg, double a2, double alpha2Deg, double d2, double theta2Deg)
{
    double sO1 = sin(deg2Rad(theta1Deg));
    double cO1 = cos(deg2Rad(theta1Deg));
    double sA1 = sin(deg2Rad(alpha1Deg));
    double cA1 = cos(deg2Rad(alpha1Deg));

    double sO2 = sin(deg2Rad(theta2Deg));
    double cO2 = cos(deg2Rad(theta2Deg));
    double sA2 = sin(deg2Rad(alpha2Deg));
    double cA2 = cos(deg2Rad(alpha2Deg));

    double Ta11, Ta12, Ta13, Ta14, Ta21, Ta22, Ta23, Ta24, Ta31, Ta32, Ta33, Ta34;
    double Tb11, Tb12, Tb13, Tb14, Tb21, Tb22, Tb23, Tb24, Tb31, Tb32, Tb33, Tb34;

    Ta11 = TPrev[0][0]; Ta12 = TPrev[0][1]; Ta13 = TPrev[0][2]; Ta14 = TPrev[0][3];
    Ta21 = TPrev[1][0]; Ta22 = TPrev[1][1]; Ta23 = TPrev[1][2]; Ta24 = TPrev[1][3];
    Ta31 = TPrev[2][0]; Ta32 = TPrev[2][1]; Ta33 = TPrev[2][2]; Ta34 = TPrev[2][3];

    Tb11 = TNext[0][0]; Tb12 = TNext[0][1]; Tb13 = TNext[0][2]; Tb14 = TNext[0][3];
    Tb21 = TNext[1][0]; Tb22 = TNext[1][1]; Tb23 = TNext[1][2]; Tb24 = TNext[1][3];
    Tb31 = TNext[2][0]; Tb32 = TNext[2][1]; Tb33 = TNext[2][2]; Tb34 = TNext[2][3];
    
    double fP = 2*Ta13*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*Ta23*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*Ta33*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1);
    double gP = 2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1)*(Ta14 - xf + Ta13*d1 + Tb24*(sA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) - cA2*sO2*(Ta11*cO1 + Ta12*sO1) + cA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + Tb34*(cA2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + sA2*sO2*(Ta11*cO1 + Ta12*sO1) - sA2*cO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + d2*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + Tb14*(cO2*(Ta11*cO1 + Ta12*sO1) + sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1)) + a2*cO2*(Ta11*cO1 + Ta12*sO1) + a2*sO2*(Ta13*sA1 + Ta12*cA1*cO1 - Ta11*cA1*sO1) + Ta11*a1*cO1 + Ta12*a1*sO1) + 2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1)*(Ta24 - yf + Ta23*d1 + Tb24*(sA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) - cA2*sO2*(Ta21*cO1 + Ta22*sO1) + cA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + Tb34*(cA2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + sA2*sO2*(Ta21*cO1 + Ta22*sO1) - sA2*cO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + d2*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + Tb14*(cO2*(Ta21*cO1 + Ta22*sO1) + sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1)) + a2*cO2*(Ta21*cO1 + Ta22*sO1) + a2*sO2*(Ta23*sA1 + Ta22*cA1*cO1 - Ta21*cA1*sO1) + Ta21*a1*cO1 + Ta22*a1*sO1) + 2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1)*(Ta34 - zf + Ta33*d1 + Tb24*(sA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) - cA2*sO2*(Ta31*cO1 + Ta32*sO1) + cA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + Tb34*(cA2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + sA2*sO2*(Ta31*cO1 + Ta32*sO1) - sA2*cO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + d2*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1) + Tb14*(cO2*(Ta31*cO1 + Ta32*sO1) + sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1)) + a2*cO2*(Ta31*cO1 + Ta32*sO1) + a2*sO2*(Ta33*sA1 + Ta32*cA1*cO1 - Ta31*cA1*sO1) + Ta31*a1*cO1 + Ta32*a1*sO1);
    
    double JInvPP11 = (2*pow(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1,2) + 2*pow(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1,2) + 2*pow(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1,2))/((2*pow(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1,2) + 2*pow(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1,2) + 2*pow(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1,2))*(2*pow(Ta13,2) + 2*pow(Ta23,2) + 2*pow(Ta33,2)) - pow(2*Ta13*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + 2*Ta23*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + 2*Ta33*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1),2));
    double JInvPP12 = -(2*pow(Ta13,2)*cA1 - sA1*(2*Ta12*Ta13*cO1 + 2*Ta22*Ta23*cO1 + 2*Ta32*Ta33*cO1) + 2*pow(Ta23,2)*cA1 + 2*pow(Ta33,2)*cA1 + sA1*sO1*(2*Ta11*Ta13 + 2*Ta21*Ta23 + 2*Ta31*Ta33))/((2*pow(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1,2) + 2*pow(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1,2) + 2*pow(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1,2))*(2*pow(Ta13,2) + 2*pow(Ta23,2) + 2*pow(Ta33,2)) - pow(2*Ta13*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + 2*Ta23*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + 2*Ta33*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1),2));
    double JInvPP21 = JInvPP12;
    double JInvPP22 = (2*pow(Ta13,2) + 2*pow(Ta23,2) + 2*pow(Ta33,2))/((2*pow(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1,2) + 2*pow(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1,2) + 2*pow(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1,2))*(2*pow(Ta13,2) + 2*pow(Ta23,2) + 2*pow(Ta33,2)) - pow(2*Ta13*(Ta13*cA1 - Ta12*sA1*cO1 + Ta11*sA1*sO1) + 2*Ta23*(Ta23*cA1 - Ta22*sA1*cO1 + Ta21*sA1*sO1) + 2*Ta33*(Ta33*cA1 - Ta32*sA1*cO1 + Ta31*sA1*sO1),2));
    
    x = d1 - (JInvPP11*fP + JInvPP12*gP);
    y = d2 - (JInvPP21*fP + JInvPP22*gP);

    derErr = sqrt(pow(fP,2) + pow(gP,2));
}

// End of Joint pair function

// Debugging Functions

void RasMath::printTransMat(double T[4][4])
{
    for(int i=0; i<4; i++)
    {
        qDebug() << T[i][0] << "\t" << T[i][1] << "\t" << T[i][2] << "\t" << T[i][3] << "\t";
    }
}

void RasMath::printMatrix(RasMatrix *M)
{
    for(int i=0; i<M->rows; i++)
    {
        QString actRow = "";
        for(int j=0; j<M->cols; j++)
        {
            actRow += QString::number(M->data[i][j]) + ((j<(M->cols-1))? "\t" : "" );
        }
        qDebug() << actRow;
    }
}
