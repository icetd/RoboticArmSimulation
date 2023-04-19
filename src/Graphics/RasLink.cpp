#include "RasLink.h"

RasLink::RasLink(QString meshfile, QString linkName)
{
    mesh = new RasMesh(meshfile);

    id = 0;
    parentId = -1;

    name = linkName;

    jointType = Revolute;
    qMin = 0;
    qMax = 0;

    a = 0;
    alpha = 0;
    d = 0;
    theta = 0;
    plusTheta = 0;
    defaultD = 0;
    defaultTheta = 0;

    isColliding = false;

    // Rotation Matrices
    R0a = new RasMatrix(true,3);
    R0b = new RasMatrix(true,3);
    Rab = new RasMatrix(true,3);
    Ra0 = new RasMatrix(true,3);
    Rb0 = new RasMatrix(true,3);
    Rba = new RasMatrix(true,3);

    // Physical properties of the Link
    I   = new RasMatrix(true,3);
    s   = new RasMatrix(3,1);
    p   = new RasMatrix(3,1);
    mass = 1;
    density = 1000;

    // Dynamical System Matrices
    Asys = RasGeneric::matrixFromString("[-1 -1;1 0]");
    Bsys = RasGeneric::matrixFromString("[1;0]");
    Csys = RasGeneric::matrixFromString("[0 1]");
    Dsys = RasGeneric::matrixFromString("[0]");

    updateP();
//    setBaseFrame(T0a);
    RasMath::setTransIdentity(T0a);
    RasMath::setTransIdentity(Tab);
    RasMath::setTransIdentity(T0b);
}

RasLink::~RasLink()
{
    delete mesh;
}

// Link Properties Functions

void RasLink::computeInertiaAndCenterOfMass()
{
    //    Steps of Inertia and Center of Mass calculation:

    //    1 - Read Polyhedron (It was already done when loading the mesh)
    //    2 - Compute Volume Integrals
    //    3 - Compute Center of Mass
    //    4 - Compute Inertia Tensor
    //    5 - Translate Inertia Tensor to Center of Mass

    //    For computing Volume Integrals we have this algorithm:
    //    For Each Face we compute the Face Integrals
    //    Each time we are computing a Face Integral we have to compute the Projection Integrals

    int A;   /* alpha */
    int B;   /* beta */
    int C;   /* gamma */

    int X = 0;
    int Y = 1;
    int Z = 2;

    /* projection integrals */
    double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

    /* face integrals */
    double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

    /* volume integrals */
    double T0, T1[3], T2[3], TP[3];


    /*Compute Volume Integrals*/
    RasMeshFace *f;
    double nx, ny, nz;

    T0 = T1[X] = T1[Y] = T1[Z]
       = T2[X] = T2[Y] = T2[Z]
       = TP[X] = TP[Y] = TP[Z] = 0;

    for(int i=0; i<mesh->nFace; i++)
    {
        f = &mesh->faces[i];
        nx = fabs(f->normal[X]);
        ny = fabs(f->normal[Y]);
        nz = fabs(f->normal[Z]);

        if(!(nx!=nx || ny!=ny || nz!=nz))
        {

        if (nx > ny && nx > nz) C = X;
        else C = (ny > nz) ? Y : Z;
        A = (C + 1) % 3;
        B = (A + 1) % 3;

        // Compute Face Integrals for the Actual Face

        double k1, k2, k3, k4;

        // Compute Projection Integrals

        double a0=0, a1=0, da;
        double b0=0, b1=0, db;
        double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
        double a1_2, a1_3, b1_2, b1_3;
        double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
        double Cab, Kab, Caab, Kaab, Cabb, Kabb;

        P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

        for (int j=0; j<3; j++)
        {
            if(j==0)
            {
                a0 = mesh->triangles->tris[i].p1[A];
                b0 = mesh->triangles->tris[i].p1[B];
                a1 = mesh->triangles->tris[i].p2[A];
                b1 = mesh->triangles->tris[i].p2[B];
            }
            if(j==1)
            {
                a0 = mesh->triangles->tris[i].p2[A];
                b0 = mesh->triangles->tris[i].p2[B];
                a1 = mesh->triangles->tris[i].p3[A];
                b1 = mesh->triangles->tris[i].p3[B];
            }
            if(j==2)
            {
                a0 = mesh->triangles->tris[i].p3[A];
                b0 = mesh->triangles->tris[i].p3[B];
                a1 = mesh->triangles->tris[i].p1[A];
                b1 = mesh->triangles->tris[i].p1[B];
            }

            da = a1 - a0;
            db = b1 - b0;
            a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
            b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
            a1_2 = a1 * a1; a1_3 = a1_2 * a1;
            b1_2 = b1 * b1; b1_3 = b1_2 * b1;

            C1 = a1 + a0;
            Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
            Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
            Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
            Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
            Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
            Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

            P1 += db*C1;
            Pa += db*Ca;
            Paa += db*Caa;
            Paaa += db*Caaa;
            Pb += da*Cb;
            Pbb += da*Cbb;
            Pbbb += da*Cbbb;
            Pab += db*(b1*Cab + b0*Kab);
            Paab += db*(b1*Caab + b0*Kaab);
            Pabb += da*(a1*Cabb + a0*Kabb);
        }

        P1 /= 2.0;
        Pa /= 6.0;
        Paa /= 12.0;
        Paaa /= 20.0;
        Pb /= -6.0;
        Pbb /= -12.0;
        Pbbb /= -20.0;
        Pab /= 24.0;
        Paab /= 60.0;
        Pabb /= -60.0;

        // End of Compute Projection Integrals

        double w = f->offset;
        double n[] = {f->normal[0],f->normal[1],f->normal[2]};

        k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

        Fa = k1 * Pa;
        Fb = k1 * Pb;
        Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

        Faa = k1 * Paa;
        Fbb = k1 * Pbb;
        Fcc = k3 * (n[A]*n[A]*Paa + 2*n[A]*n[B]*Pab + n[B]*n[B]*Pbb + w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

        Faaa = k1 * Paaa;
        Fbbb = k1 * Pbbb;
        Fccc = -k4 * (n[A]*n[A]*n[A]*Paaa + 3*n[A]*n[A]*n[B]*Paab + 3*n[A]*n[B]*n[B]*Pabb + n[B]*n[B]*n[B]*Pbbb + 3*w*(n[A]*n[A]*Paa + 2*n[A]*n[B]*Pab + n[B]*n[B]*Pbb) + w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

        Faab = k1 * Paab;
        Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
        Fcca = k3 * (n[A]*n[A]*Paaa + 2*n[A]*n[B]*Paab + n[B]*n[B]*Pabb + w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));

        // End of compute Face Integrals

        T0 += f->normal[X] * ((A == X) ? Fa : ((B == X) ? Fb : Fc));
//        qDebug() << "Normal para cara" << i << "Normal:" << f->normal[X] << "T0" << T0;

        T1[A] += f->normal[A] * Faa;
        T1[B] += f->normal[B] * Fbb;
        T1[C] += f->normal[C] * Fcc;
        T2[A] += f->normal[A] * Faaa;
        T2[B] += f->normal[B] * Fbbb;
        T2[C] += f->normal[C] * Fccc;
        TP[A] += f->normal[A] * Faab;
        TP[B] += f->normal[B] * Fbbc;
        TP[C] += f->normal[C] * Fcca;

        }
    }

    T1[X] /= 2; T1[Y] /= 2; T1[Z] /= 2;
    T2[X] /= 3; T2[Y] /= 3; T2[Z] /= 3;
    TP[X] /= 2; TP[Y] /= 2; TP[Z] /= 2;
    // End of Computing Volume Integrals

    double r[3];
    density = mass/T0;
    r[X] = T1[X] / T0;
    r[Y] = T1[Y] / T0;
    r[Z] = T1[Z] / T0;

    /*Compute Center of Mass*/
    s->data[X][0] = r[X];
    s->data[Y][0] = r[Y];
    s->data[Z][0] = r[Z];

    /*Compute Inertia Tensor*/
    I->data[X][X] = density * (T2[Y] + T2[Z]);
    I->data[Y][Y] = density * (T2[Z] + T2[X]);
    I->data[Z][Z] = density * (T2[X] + T2[Y]);
    I->data[X][Y] = I->data[Y][X] = - density * TP[X];
    I->data[Y][Z] = I->data[Z][Y] = - density * TP[Y];
    I->data[Z][X] = I->data[X][Z] = - density * TP[Z];

    /*Translate Inertia Tensor to Center of Mass*/
    I->data[X][X] -= mass * (r[Y]*r[Y] + r[Z]*r[Z]);
    I->data[Y][Y] -= mass * (r[Z]*r[Z] + r[X]*r[X]);
    I->data[Z][Z] -= mass * (r[X]*r[X] + r[Y]*r[Y]);
    I->data[X][Y] = I->data[Y][X] += mass * r[X] * r[Y];
    I->data[Y][Z] = I->data[Z][Y] += mass * r[Y] * r[Z];
    I->data[Z][X] = I->data[X][Z] += mass * r[Z] * r[X];

//    RasMath::printMatrix(I);

}

// End of Link Properties Functions

// Matrices Functions

void RasLink::updateDH()
{
    RasMath::convDH2Trans(a,alpha,d,theta + plusTheta,Tab);
}

void RasLink::updateP()
{
    p->data[0][0] = a;
    p->data[1][0] = d * sin(RasMath::deg2Rad(alpha));
    p->data[2][0] = d * cos(RasMath::deg2Rad(alpha));
}

void RasLink::setBaseFrame(double T[4][4], bool secondaryMatrices)
{
    RasMath::copyMatTrans(T,T0a);
    updateDH();
    RasMath::multTransMat(T0a,Tab,T0b);
    RasMath::convTrans2RotTra(T0b,RLink,PLink);
    RasMath::convRot2RPY(RLink,roll,pitch,yaw);

    if(secondaryMatrices == true)
    {
        RasMath::copyRotFromTrans(T0a,R0a);
        RasMath::copyRotFromTrans(T0b,R0b);
        RasMath::copyRotFromTrans(Tab,Rab);

        R0a->getTranspose(Ra0);
        R0b->getTranspose(Rb0);
        Rab->getTranspose(Rba);

        if(jointType == Prismatic)
        updateP();
    }
}

void::RasLink::updateglMatrices()
{
    RasMath::convTrans2glMat(T0b,glMat);
}

// Enf of Matrices Functions

// Debugging Functions

void RasLink::printLinkFeatures()
{
//    qDebug() << "a =" << a << ", alpha =" << alpha << ", d =" << d << ", theta =" << theta;
//    qDebug() << "Ta =";
//    RasMath::printTransMat(T0a);
//    qDebug() << "Tb =";
//    RasMath::printTransMat(T0b);
//    qDebug() << "DH =";
//    RasMath::printTransMat(Tab);
}

// Enf of Debugging Functions
