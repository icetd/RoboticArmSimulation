#ifndef RASOBSTACLE_H
#define RASOBSTACLE_H

#include <QString>
#include <RasMesh.h>
#include <RasMath.h>
#include <QVariant>

class RasObstacle
{
public:

    enum RasLocRotProps{PropName,PropX,PropY,PropZ,PropRotX,PropRotY,PropRotZ};

    RasObstacle(QString filename);
    ~RasObstacle();

    void setProperties(QString name, double tx, double ty, double tz, double rx, double ry, double rz);
    void setProperties(RasLocRotProps propertie, QVariant value);

    QString name;
    RasMesh *mesh;

    float rotX;
    float rotY;
    float rotZ;

    double R[3][3];
    double T[3];
    double glMat[16];

    bool isColliding;
};

#endif // RASOBSTACLE_H
