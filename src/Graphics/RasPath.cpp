#include "RasPath.h"

RasPath::RasPath(QString filename)
{
    QFile       *pointsFile = new QFile(filename);
    QFileInfo   *pointsFileInfo = new QFileInfo(*pointsFile);

    pointsFile->open(QIODevice::ReadOnly);
    nPoints = 0;
    while(!pointsFile->atEnd())
    {
        QByteArray pointsFileLine = pointsFile->readLine();
        if(pointsFileLine != QString("\n"))
        nPoints++;
    }

    points = new RasPathPoint[nPoints];
    pointsFile->reset();
    int iPoint = 0;
    totalDistance = 0;

    while(!pointsFile->atEnd())
    {
        QByteArray pointsFileLine = pointsFile->readLine();

        char split = '\t';
        if(pointsFileLine.indexOf(" ") != -1)
        split = ' ';
        if(pointsFileLine.indexOf("\t") != -1)
        split = '\t';

        QList<QByteArray> pointsLineValues = pointsFileLine.split(split);
        if(pointsLineValues.length() == 3)
        {
            points[iPoint].pos[0] = pointsLineValues[0].simplified().toDouble();
            points[iPoint].pos[1] = pointsLineValues[1].simplified().toDouble();
            points[iPoint].pos[2] = pointsLineValues[2].simplified().toDouble();

            if(iPoint!=0)
            totalDistance += sqrt(pow(points[iPoint].pos[0]-points[iPoint-1].pos[0],2) + pow(points[iPoint].pos[1]-points[iPoint-1].pos[1],2) + pow(points[iPoint].pos[2]-points[iPoint-1].pos[2],2));

            iPoint++;
        }
    }
    name = pointsFileInfo->completeBaseName();
    visible = true;
    pointsFile->close();
    delete pointsFile;
    delete pointsFileInfo;
}

RasPath::RasPath(double xIni,double yIni, double zIni, double xFin, double yFin, double zFin, int n)
{
    points = new RasPathPoint[n];
    nPoints = n;

    double stepX = (xFin - xIni)/(double)(n-1);
    double stepY = (yFin - yIni)/(double)(n-1);
    double stepZ = (zFin - zIni)/(double)(n-1);

    for(int i=0; i<n; i++)
    {
        points[i].pos[0] = xIni + ((double)i * stepX);
        points[i].pos[1] = yIni + ((double)i * stepY);
        points[i].pos[2] = zIni + ((double)i * stepZ);
    }

    totalDistance = sqrt(pow(xFin-xIni,2) + pow(yFin-yIni,2) + pow(zFin-zIni,2));

    visible = true;
    name = "Line";
}

RasPath::~RasPath()
{
    delete [] points;
}

void RasPath::setProp(int prop, QVariant value)
{
    switch(prop)
    {
        case 0:
        name = value.toString();
        break;
        case 1:
        visible = value.toBool();
        break;
    }
}
