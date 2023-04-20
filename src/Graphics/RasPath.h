#ifndef RASPATH_H
#define RASPATH_H

#include <QString>
#include <QFile>
#include <QFileInfo>
#include <QTreeWidget>
#include <QVariant>
#include "math.h"

typedef struct RasPathPoint
{
    double pos[3];
}
RasPathPoint;

class RasPath
{
public:
    RasPath(QString filename);
    RasPath(double xIni,double yIni, double zIni, double xFin, double yFin, double zFin, int n);
    ~RasPath();

    void setProp(int prop, QVariant value);

    int nPoints;
    bool visible;
    double totalDistance;
    RasPathPoint *points;
    QString name;

};

#endif // RASPATH_H
