#ifndef QDYNLINK_H
#define QDYNLINK_H

#include <QString>
#include "RasMath.h"
#include "RasMesh.h"
#include "RasGeneric.h"

class RasLink
{
public:

    enum RasJointType{Revolute, Prismatic};

    RasLink(QString meshfile, QString linkName);
    ~RasLink();

    // Link Properties Functions
    void computeInertiaAndCenterOfMass();

    // Matrices Functions
    void updateDH();
    void updateP();
    void setBaseFrame(double T[4][4], bool secondaryMatrices = false);
    void updateglMatrices();

    // Debugging Functions
    void printLinkFeatures();


    RasMesh *mesh;

    int parentId;
    int id;

    QString name;

    RasJointType jointType;
    double qMin;
    double qMax;

    double a;
    double alpha;
    double d;
    double theta;
    double plusTheta;

    double defaultD;
    double defaultTheta;

    double roll, pitch, yaw;

    double T0a[4][4];
    double T0b[4][4];
    double Tab[4][4];
    double Tbn[4][4];
    RasMatrix *R0a, *R0b, *Rab, *Ra0, *Rb0, *Rba;

    RasMatrix *I, *s, *p;
    double mass, density;

    double RLink[3][3];
    double PLink[3];

    double glMat[16];

    RasMatrix *Asys;
    RasMatrix *Bsys;
    RasMatrix *Csys;
    RasMatrix *Dsys;

    bool isColliding;

};

#endif // QDYNLINK_H
