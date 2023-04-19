#ifndef QDYNMESH_H
#define QDYNMESH_H

#include <QString>
#include <QFile>
#include <QFileInfo>
#include <math.h>
#include <QDebug>
#include "RAPID.H"

typedef struct RasMeshVertex
{
    float x;
    float y;
    float z;
}
RasMeshVertex;


typedef struct RasMeshFace
{
    int material;
    double normal[3];
    double offset;
}
RasMeshFace;

typedef struct RasMeshMaterial
{
    float r;
    float g;
    float b;
    float a;
    QString name;
}
RasMeshMaterial;


class RasMesh
{
public:

    int nVert;
    int nFace;
    int nMate;

    double xMin,xMax,yMin,yMax,zMin,zMax;

    RasMeshFace        *faces;
    RasMeshMaterial    *materials;

    RAPID_model         *triangles;

    RasMesh(QString filename);
    ~RasMesh();

private:
    RasMeshVertex      *vertices;
    void computeFaceNormals();
};

#endif // QDYNMESH_H
