#include "RasMesh.h"

void RasMesh::computeFaceNormals()
{
    float x0,y0,z0,x1,y1,z1,x2,y2,z2,Px,Py,Pz,Qx,Qy,Qz,nx,ny,nz,len;
    for (int i=0; i<nFace; i++)
    {
        x0 = triangles->tris[i].p1[0];
        y0 = triangles->tris[i].p1[1];
        z0 = triangles->tris[i].p1[2];

        x1 = triangles->tris[i].p2[0];
        y1 = triangles->tris[i].p2[1];
        z1 = triangles->tris[i].p2[2];

        x2 = triangles->tris[i].p3[0];
        y2 = triangles->tris[i].p3[1];
        z2 = triangles->tris[i].p3[2];

        Px= x1-x0;
        Py= y1-y0;
        Pz= z1-z0;
        Qx= x2-x1;
        Qy= y2-y1;
        Qz= z2-z1;

        nx = Py*Qz - Pz*Qy;
        ny = Pz*Qx - Px*Qz;
        nz = Px*Qy - Py*Qx;

        len = (float)(sqrt((nx * nx) + (ny * ny) + (nz * nz)));

        nx=nx/len;
        ny=ny/len;
        nz=nz/len;

        faces[i].normal[0] = nx;
        faces[i].normal[1] = ny;
        faces[i].normal[2] = nz;

        faces[i].offset = - (nx * x0) - (ny * y0) - (nz * z0);
    }
}

RasMesh::~RasMesh()
{
    delete [] faces;
    delete [] materials;
    delete triangles;
}

RasMesh::RasMesh(QString meshFileName)
{
    QFile       *meshFile = new QFile(meshFileName);
    QFileInfo   *meshFileInfo = new QFileInfo(*meshFile);
    QString      mateFileName = "";

    nVert = 0;
    nFace = 0;
    nMate = 1;

    xMax = yMax = zMax = -5000;
    xMin = yMin = zMin =  5000;

    meshFile->open(QIODevice::ReadOnly);

    while(!meshFile->atEnd())
    {
        QByteArray meshFileLine = meshFile->readLine();
        QList<QByteArray> meshLineValues = meshFileLine.simplified().split(' ');

        if(meshLineValues.at(0) == QString("mtllib"))
        {
            mateFileName = meshFileInfo->absolutePath() + "/" + meshLineValues.at(1);
            QFile *mateFile = new QFile(mateFileName);
            mateFile->open(QIODevice::ReadOnly);

            while(!mateFile->atEnd())
            {
                QByteArray mateFileLine = mateFile->readLine();
                QList<QByteArray> mateLineValues = mateFileLine.simplified().split(' ');
                if(mateLineValues.at(0) == QString("newmtl"))
                {
                    nMate++;
                }
            }
            mateFile->close();
            delete mateFile;
        }
        if (meshLineValues.at(0) == QString("v"))
        {
            nVert++;
        }
        if (meshLineValues.at(0) == QString("f"))
        {
            nFace++;
            if(meshLineValues.length() == 5)
            {
                nFace++;
            }
        }
    }

    meshFile->reset();

    vertices = new RasMeshVertex[nVert];
    faces = new RasMeshFace[nFace];
    materials = new RasMeshMaterial[nMate];

    triangles = new RAPID_model;
    triangles->BeginModel();

    int iVert = 0;
    int iFace = 0;
    int iMate = 0;

    QString     actMatName = "";
    int         actNumMat = 0;
    double      actV1[3];
    double      actV2[3];
    double      actV3[3];

    materials[0].r = 0.7f;
    materials[0].g = 0.7f;
    materials[0].b = 0.7f;
    materials[0].a = 1.0f;
    materials[0].name = "0";

    while(!meshFile->atEnd())
    {
        QByteArray meshFileLine = meshFile->readLine();
        QList<QByteArray> meshLineValues = meshFileLine.simplified().split(' ');

        if(meshLineValues.at(0) == QString("mtllib"))
        {
            mateFileName = meshFileInfo->absolutePath() + "/" + meshLineValues.at(1);
            QFile *mateFile = new QFile(mateFileName);
            mateFile->open(QIODevice::ReadOnly);

            while(!mateFile->atEnd())
            {
                QByteArray mateFileLine = mateFile->readLine();
                QList<QByteArray> mateLineValues = mateFileLine.simplified().split(' ');

                if(mateLineValues.at(0) == QString("newmtl"))
                {
                    iMate++;
                    materials[iMate].name = mateLineValues.at(1);
                    materials[iMate].r = 0.0f;
                    materials[iMate].g = 0.0f;
                    materials[iMate].b = 0.0f;
                    materials[iMate].a = 1.0f;
                }
                if(mateLineValues.at(0).toUpper() == QString("KD"))
                {
                    materials[iMate].r = mateLineValues.at(1).toFloat();
                    materials[iMate].g = mateLineValues.at(2).toFloat();
                    materials[iMate].b = mateLineValues.at(3).toFloat();
                }
                if(mateLineValues.at(0).toUpper() == QString("TR"))
                {
                    materials[iMate].a = 1.0f - mateLineValues.at(1).toFloat();
                }
            }
            mateFile->close();
            delete mateFile;
        }

        if (meshLineValues.at(0) == QString("usemtl"))
        {
            actMatName = meshLineValues.at(1);
            if (actMatName != QString("(null)"))
            {
                for (int i=0;i<nMate;i++)
                {
                    if(actMatName == materials[i].name)
                    {
                        actNumMat = i;
                        break;
                    }
                }
            }
        }
        if (meshLineValues.at(0) == QString("v"))
        {
            vertices[iVert].x = meshLineValues.at(1).toFloat();
            vertices[iVert].y = meshLineValues.at(2).toFloat();
            vertices[iVert].z = meshLineValues.at(3).toFloat();

            if(vertices[iVert].x <= xMin)
            xMin = vertices[iVert].x;

            if(vertices[iVert].x >= xMax)
            xMax = vertices[iVert].x;

            if(vertices[iVert].y <= yMin)
            yMin = vertices[iVert].y;

            if(vertices[iVert].y >= yMax)
            yMax = vertices[iVert].y;

            if(vertices[iVert].z <= zMin)
            zMin = vertices[iVert].z;

            if(vertices[iVert].z >= zMax)
            zMax = vertices[iVert].z;

            iVert++;
        }
        if (meshLineValues.at(0) == QString("f"))
        {
            actV1[0] = vertices[meshLineValues.at(1).toInt()-1].x;
            actV1[1] = vertices[meshLineValues.at(1).toInt()-1].y;
            actV1[2] = vertices[meshLineValues.at(1).toInt()-1].z;

            actV2[0] = vertices[meshLineValues.at(2).toInt()-1].x;
            actV2[1] = vertices[meshLineValues.at(2).toInt()-1].y;
            actV2[2] = vertices[meshLineValues.at(2).toInt()-1].z;

            actV3[0] = vertices[meshLineValues.at(3).toInt()-1].x;
            actV3[1] = vertices[meshLineValues.at(3).toInt()-1].y;
            actV3[2] = vertices[meshLineValues.at(3).toInt()-1].z;

            triangles->AddTri(actV1,actV2,actV3,iFace);

            faces[iFace].material = actNumMat;
            iFace++;

            if(meshLineValues.length() == 5)
            {
                actV1[0] = vertices[meshLineValues.at(3).toInt()-1].x;
                actV1[1] = vertices[meshLineValues.at(3).toInt()-1].y;
                actV1[2] = vertices[meshLineValues.at(3).toInt()-1].z;

                actV2[0] = vertices[meshLineValues.at(4).toInt()-1].x;
                actV2[1] = vertices[meshLineValues.at(4).toInt()-1].y;
                actV2[2] = vertices[meshLineValues.at(4).toInt()-1].z;

                actV3[0] = vertices[meshLineValues.at(1).toInt()-1].x;
                actV3[1] = vertices[meshLineValues.at(1).toInt()-1].y;
                actV3[2] = vertices[meshLineValues.at(1).toInt()-1].z;

                triangles->AddTri(actV1,actV2,actV3,iFace);

                faces[iFace].material = actNumMat;
                iFace++;
            }
        }

    }
    delete[] vertices;
    triangles->EndModel();
    meshFile->close();
    delete meshFile;
    delete meshFileInfo;
    computeFaceNormals();
}
