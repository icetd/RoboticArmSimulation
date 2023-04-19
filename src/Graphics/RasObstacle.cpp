#include "RasObstacle.h"

RasObstacle::RasObstacle(QString filename)
{
    mesh = new RasMesh(filename);

    QFileInfo fileInfo(filename);
    name = fileInfo.completeBaseName();

    setProperties(name,0,0,0,0,0,0);
}

void RasObstacle::setProperties(QString newname, double tx, double ty, double tz, double rx, double ry, double rz)
{
    if(newname != QString(""))
    name = newname;

    T[0] = tx;
    T[1] = ty;
    T[2] = tz;

    rotX = RasMath::normalizeAngle(rx);
    rotY = RasMath::normalizeAngle(ry);
    rotZ = RasMath::normalizeAngle(rz);

    isColliding = false;

    RasMath::convRxRyRz2RotM(rotX,rotY,rotZ,R);
    RasMath::convRotTra2glMat(R,T,glMat);
}

void RasObstacle::setProperties(RasLocRotProps propertie, QVariant value)
{
    switch(propertie)
    {
        case PropName:
        if(value.toString()!= QString(""))
        name = value.toString();
        break;

        case PropX:
        T[0] = value.toDouble();
        break;

        case PropY:
        T[1] = value.toDouble();
        break;

        case PropZ:
        T[2] = value.toDouble();
        break;

        case PropRotX:
        rotX = RasMath::normalizeAngle(value.toDouble());
        RasMath::convRxRyRz2RotM(rotX,rotY,rotZ,R);
        break;

        case PropRotY:
        rotY = RasMath::normalizeAngle(value.toDouble());
        RasMath::convRxRyRz2RotM(rotX,rotY,rotZ,R);
        break;

        case PropRotZ:
        rotZ = RasMath::normalizeAngle(value.toDouble());
        RasMath::convRxRyRz2RotM(rotX,rotY,rotZ,R);
        break;

        default:
        break;
    }

    if(propertie == PropRotX || propertie == PropRotY || propertie == PropRotZ)
    {
        RasMath::convRxRyRz2RotM(rotX,rotY,rotZ,R);
    }
    if(propertie != PropName)
    {
        RasMath::convRotTra2glMat(R,T,glMat);
    }

    isColliding = false;
}

RasObstacle::~RasObstacle()
{
    delete mesh;
}
