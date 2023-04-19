#include "RasGeneric.h"

RasGeneric::RasGeneric()
{
}

QString RasGeneric::double2meters(double value)
{
    return QString::number(value) + " m";
}

QString RasGeneric::double2degree(double value)
{
    return QString::number(value) + " °";
}

QString RasGeneric::double2linspeed(double value)
{
    return QString::number(value) + " m/s";
}

QString RasGeneric::double2angspeed(double value)
{
    return QString::number(value) + " M^2/s";
}

QString RasGeneric::double2kilogram(double value)
{
    return QString::number(value) + " kg";
}

double RasGeneric::meters2double(QString value)
{
    value = value.simplified();
    value.chop(1);
    return value.toDouble();
}

double RasGeneric::degree2double(QString value)
{
    value = value.simplified();
    value.chop(1);
    return value.toDouble();
}

double RasGeneric::kilogram2double(QString value)
{
    value = value.simplified();
    value.chop(2);
    return value.toDouble();
}

RasMatrix* RasGeneric::matrixFromString(QString matStr)
{
    matStr.remove(0,1);
    matStr.chop(1);
    QList<QString> strRows;
    QList<QString> strColsActRow;
    strRows = matStr.split(";");
    strColsActRow = strRows[0].split(" ");

    RasMatrix *R = new RasMatrix(strRows.length(),strColsActRow.length());
    strColsActRow.clear();

    for(int i=0; i<strRows.length(); i++)
    {
        strColsActRow = strRows[i].split(" ");

        for(int j=0; j<R->cols; j++)
        {
            R->data[i][j] = strColsActRow[j].toDouble();
        }

        strColsActRow.clear();
    }

    return R;
}
