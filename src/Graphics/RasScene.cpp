#include "RasScene.h"
#include "RasMath.h"
#include <GL/glu.h>
#include <QMouseEvent>
#include <QMessageBox>

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

// Create light components
float ambientLight[] = { 0.1f, 0.1f, 0.1f, 1.0f };
float diffuseLight[] = { 0.8f, 0.8f, 0.8f, 1.0f };
float specularLight[] = { 0.8f, 0.8f, 0.8f, 1.0f };
float light_position[] = {0.0, 0.0, 1.0, 1.0};

RasScene::RasScene(QWidget *parent) : QGLWidget(parent)
{
    xRot = -45.0;
    yRot = 0;
    zRot = -45.0;

    xPos = 0;
    yPos = 0;
    zPos = -10.0;

    angle = 45.0;
    fovy = 45.0f;
    zNear = 0.1f;
    zFar = 500.0f;

    painting = false;

    enabledBlend = false;
    enabledGlobalAxis = true;
    enabledLocalAxes = false;
    enabledObjectCoordinates = false;
    enabledGrid = true;

    collisionType = CollideEndEffector;
}

RasScene::~RasScene()
{
}

void RasScene::initializeGL() {
    glClearColor(0.2f, 0.2f, 0.2f, 0.0f);
    glShadeModel(GL_SMOOTH);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    glEnable(GL_COLOR_MATERIAL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);

    glDisable(GL_BLEND);

    _fpsTime.start();
    _fpsCounter = 0;
    _fps = 0.0;
    _fpsString = "?Hz";
}

void RasScene::resizeGL(int width, int height)
{
    glViewport(0, 0,(double)width,(double)height);
    aspect = ((double)width)/((double)height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fovy, aspect, zNear, zFar);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void RasScene::paintGL()
{
    if(painting)
    return;

    painting = true;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
    glTranslatef(xPos,yPos,zPos);
    glRotatef(xRot, 1.0, 0.0, 0.0);
    glRotatef(zRot, 0.0, 0.0, 1.0);


    // Draw the Grid if needed
    if(enabledGrid)
    {
        drawGrid(5);
    }

    // Draw the global Axis if needed
    if(enabledGlobalAxis)
    {
        drawAxis(1);
    }

    // Draw the Paths
    if(Paths.length()>0)
    {
        glDisable(GL_LIGHTING);
//        glEnable(GL_LINE_STIPPLE);
//        glLineStipple(4,0xAAAA);
        glBegin(GL_LINES);
        glColor4f(0.0f,0.8f,0.0f,1.0f);
        for(int i=0; i<Paths.length(); i++)
        {
            if(Paths[i]->visible)
            {
                for(int j=0; j<(Paths[i]->nPoints-1); j++)
                {
                    glVertex3f(Paths[i]->points[j  ].pos[0],Paths[i]->points[ j ].pos[1],Paths[i]->points[ j ].pos[2]);
                    glVertex3f(Paths[i]->points[j+1].pos[0],Paths[i]->points[j+1].pos[1],Paths[i]->points[j+1].pos[2]);
                }
            }
        }
        glEnd();
//        glDisable(GL_LINE_STIPPLE);
        glEnable(GL_LIGHTING);
    }

    // Draw the Robots
    for(int k=0; k<Robots.length(); k++)
    {
        if(enabledLocalAxes)
        {
            if(Robots[k]->isSelected)
            drawBox(Robots[k]->xMin,Robots[k]->xMax,Robots[k]->yMin,Robots[k]->yMax,Robots[k]->zMin,Robots[k]->zMax);

            glPushMatrix();
            glMultMatrixd(Robots[k]->glMat);
            drawAxis(0.25);
            glPopMatrix();
        }

        for(int i=0; i<Robots[k]->nLinks; i++)
        {
            if(enabledObjectCoordinates)
            drawCoordinates(Robots[k]->Links[i]->PLink[0],Robots[k]->Links[i]->PLink[1],Robots[k]->Links[i]->PLink[2]);

            glPushMatrix();
            glMultMatrixd(Robots[k]->Links[i]->glMat);

            if(enabledLocalAxes)
            {
                drawAxis(0.25);
                if(i==Robots[k]->selectedLink)
                drawBox(Robots[k]->Links[i]->mesh->xMin, Robots[k]->Links[i]->mesh->xMax, Robots[k]->Links[i]->mesh->yMin, Robots[k]->Links[i]->mesh->yMax, Robots[k]->Links[i]->mesh->zMin, Robots[k]->Links[i]->mesh->zMax);
            }

            glBegin(GL_TRIANGLES);
            drawMesh(Robots[k]->Links[i]->mesh,Robots[k]->Links[i]->isColliding,0);
            glEnd();
            glPopMatrix();
        }

        if(Robots[k]->nPoints > 0 && simRunning)
        {
            glDisable(GL_LIGHTING);
    //        glEnable(GL_LINE_STIPPLE);
    //        glLineStipple(4,0xAAAA);
            glBegin(GL_LINES);
            glColor4f(0.0f,0.0f,0.8f,1.0f);
            for(int i=0; i<Robots[k]->nPoints-1; i++)
            {
                glVertex3f(Robots[k]->xRef[i  ],Robots[k]->yRef[i  ],Robots[k]->zRef[i  ]);
                glVertex3f(Robots[k]->xRef[i+1],Robots[k]->yRef[i+1],Robots[k]->zRef[i+1]);
            }
            glEnd();
    //        glDisable(GL_LINE_STIPPLE);
            glEnable(GL_LIGHTING);
        }
    }

    // Draw the Obstacles
    if(!enabledBlend)
    {
        // If blending is not enabled then all faces are drawn
        for(int i=0; i<Obstacles.length(); i++)
        {
            if(enabledObjectCoordinates)
            drawCoordinates(Obstacles[i]->T[0],Obstacles[i]->T[1],Obstacles[i]->T[2]);

            glPushMatrix();
            glMultMatrixd(Obstacles[i]->glMat);

            if(enabledLocalAxes)
            drawAxis(0.50);

            glBegin(GL_TRIANGLES);
            drawMesh(Obstacles[i]->mesh,Obstacles[i]->isColliding,0);
            glEnd();
            glPopMatrix();
        }
    }
    else
    {
        // If alpha blending is enabled, all opaque faces are drawn first
        for(int i=0; i<Obstacles.length(); i++)
        {
            if(enabledObjectCoordinates)
            drawCoordinates(Obstacles[i]->T[0],Obstacles[i]->T[1],Obstacles[i]->T[2]);

            glPushMatrix();
            glMultMatrixd(Obstacles[i]->glMat);

            if(enabledLocalAxes)
            drawAxis(0.5);

            glBegin(GL_TRIANGLES);
            drawMesh(Obstacles[i]->mesh,Obstacles[i]->isColliding,1);
            glEnd();
            glPopMatrix();
        }
        // When all opaque faces are drawn, then all the transparent faces are drawn
        for(int i=0; i<Obstacles.length(); i++)
        {
            if(!Obstacles[i]->isColliding)
            {
                glPushMatrix();
                glMultMatrixd(Obstacles[i]->glMat);
                glBegin(GL_TRIANGLES);
                drawMesh(Obstacles[i]->mesh,Obstacles[i]->isColliding,2);
                glEnd();
                glPopMatrix();
            }

        }
    }

    glPopMatrix();

    // FPS computation
    const unsigned int maxCounter = 20;
    if (++_fpsCounter == maxCounter)
    {
        _fps = 1000.0 * (float)maxCounter / (float)_fpsTime.restart();
//        fpsString_ = QString("%1Hz").arg(f_p_s_, 0, 'f', ((f_p_s_ < 10.0)?1:0));
        _fpsString = QString::number(_fps) + "Hz";
        _fpsCounter = 0;
    }

    painting = false;
//    qDebug() << fpsString_;
}

void RasScene::drawCoordinates(double x, double y, double z)
{
    glDisable(GL_LIGHTING);
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(3,0xAAAA);
    glColor4f(0.0f,1.0f,0.0f,1.0f);
    glBegin(GL_LINES);

    glVertex3f(x,0,0);
    glVertex3f(x,y,0);

    glVertex3f(0,y,0);
    glVertex3f(x,y,0);

    glVertex3f(x,y,0);
    glVertex3f(x,y,z);

    glEnd();
    glDisable(GL_LINE_STIPPLE);
    glEnable(GL_LIGHTING);
}

void RasScene::drawBox(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax)
{
    glDisable(GL_LIGHTING);
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(3,0xAAAA);
    glColor4f(0.8f,0.8f,0.8f,1.0f);
    glBegin(GL_LINES);

    glVertex3f(xMin,yMax,zMax);
    glVertex3f(xMin,yMin,zMax);

    glVertex3f(xMin,yMin,zMax);
    glVertex3f(xMin,yMin,zMin);

    glVertex3f(xMin,yMin,zMin);
    glVertex3f(xMin,yMax,zMin);

    glVertex3f(xMin,yMax,zMin);
    glVertex3f(xMin,yMax,zMax);

    glVertex3f(xMin,yMax,zMax);
    glVertex3f(xMax,yMax,zMax);

    glVertex3f(xMin,yMin,zMax);
    glVertex3f(xMax,yMin,zMax);

    glVertex3f(xMin,yMin,zMin);
    glVertex3f(xMax,yMin,zMin);

    glVertex3f(xMin,yMax,zMin);
    glVertex3f(xMax,yMax,zMin);

    glVertex3f(xMax,yMax,zMax);
    glVertex3f(xMax,yMin,zMax);

    glVertex3f(xMax,yMin,zMax);
    glVertex3f(xMax,yMin,zMin);

    glVertex3f(xMax,yMin,zMin);
    glVertex3f(xMax,yMax,zMin);

    glVertex3f(xMax,yMax,zMin);
    glVertex3f(xMax,yMax,zMax);

    glEnd();
    glDisable(GL_LINE_STIPPLE);
    glEnable(GL_LIGHTING);
}

void RasScene::drawArrow(float length, int nbSubdivisions)
{
    static GLUquadric* quadric = gluNewQuadric();

    double cylRad = 0.02*length;
    double conRad = 2*cylRad;
    double cylLen = 0.85*length;
    gluCylinder(quadric,cylRad,cylRad,cylLen,nbSubdivisions,1);
    glTranslatef(0.0,0.0,cylLen);
    gluCylinder(quadric,conRad,0.0,length - cylLen,nbSubdivisions,1);
    gluDisk(quadric,cylRad,conRad,nbSubdivisions,1);
}

void RasScene::drawAxis(double length, bool opaque)
{
    const float charWidth = length / 30.0;
    const float charHeight = length / 20.0;
    const float charShift = 1.04 * length;

    glDisable(GL_LIGHTING);
    glColor4f(0.8f,0.8f,0.8f,1.0f);
    glBegin(GL_LINES);
    // The X
    glVertex3f(charShift,  charWidth, -charHeight);
    glVertex3f(charShift, -charWidth,  charHeight);
    glVertex3f(charShift, -charWidth, -charHeight);
    glVertex3f(charShift,  charWidth,  charHeight);
    // The Y
    glVertex3f( charWidth, charShift, charHeight);
    glVertex3f(0.0,        charShift, 0.0);
    glVertex3f(-charWidth, charShift, charHeight);
    glVertex3f(0.0,        charShift, 0.0);
    glVertex3f(0.0,        charShift, 0.0);
    glVertex3f(0.0,        charShift, -charHeight);
    // The Z

    glVertex3f(-charWidth,  charHeight, charShift);
    glVertex3f( charWidth,  charHeight, charShift);
    glVertex3f( charWidth,  charHeight, charShift);
    glVertex3f(-charWidth, -charHeight, charShift);
    glVertex3f(-charWidth, -charHeight, charShift);
    glVertex3f( charWidth, -charHeight, charShift);
    glEnd();
    glEnable(GL_LIGHTING);

    // Eje X
    glPushMatrix();
    if(opaque)
    glColor4f(0.8f,0.4f,0.4f,1.0f);
    else
    glColor4f(1.0f,0.2f,0.2f,1.0f);
    glRotatef(90.0,0.0,1.0,0.0);
    drawArrow(length);
    glPopMatrix();
    // Fin EjeX

    // Eje Y
    glPushMatrix();
    if(opaque)
    glColor4f(0.4f,0.8f,0.4f,1.0f);
    else
    glColor4f(0.2f,1.0f,0.2f,1.0f);
    glRotatef(-90.0,1.0,0.0,0.0);
    drawArrow(length);
    glPopMatrix();
    // Fin EjeY

    // Eje Z
    glPushMatrix();
    if(opaque)
    glColor4f(0.4f,0.4f,0.8f,1.0f);
    else
    glColor4f(0.2f,0.2f,1.0f,1.0f);
    drawArrow(length);
    glPopMatrix();
    // Fin EjeZ
}


void RasScene::drawGrid(float size, int nbSubdivisions)
{
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);

    for (int i=0; i<=nbSubdivisions; ++i)
    {
        const float pos = size*(2.0*i/nbSubdivisions-1.0);
        if(pos != 0)
        {
            glColor4f(0.46f,0.46f,0.46f,1.0f);
        }
        else
        {
            glColor4f(0.0f,0.0f,0.0f,1.0f);
        }
        glVertex2f(pos, -size);
        glVertex2f(pos, +size);
        glVertex2f(-size, pos);
        glVertex2f( size, pos);
    }
    glEnd();
    glEnable(GL_LIGHTING);
}


void RasScene::drawMesh(RasMesh *mesh, bool isColliding, int type)
{

    // if type = 0 it draws all kind of faces
    // if type = 1 it draws only opaque faces
    // if type = 2 it draws only transparent faces

    RasMeshFace *face;
    if(!isColliding)
    {
        RasMeshMaterial *mat;
        int actMat = 0;
        mat = &mesh->materials[actMat];
        glColor4f(mat->r, mat->g, mat->b, mat->a);

        // All faces
        if (type==0)
        {
            for(int i=0; i<mesh->nFace; i++)
            {
                face = &mesh->faces[i];

                if (face->material != actMat)
                {
                    actMat = face->material;
                    mat = &mesh->materials[actMat];
                    glColor4f(mat->r, mat->g, mat->b, mat->a);
                }
                glNormal3f(face->normal[0],face->normal[1],face->normal[2]);
                glVertex3f(mesh->triangles->tris[i].p1[0], mesh->triangles->tris[i].p1[1] , mesh->triangles->tris[i].p1[2]);
                glVertex3f(mesh->triangles->tris[i].p2[0], mesh->triangles->tris[i].p2[1] , mesh->triangles->tris[i].p2[2]);
                glVertex3f(mesh->triangles->tris[i].p3[0], mesh->triangles->tris[i].p3[1] , mesh->triangles->tris[i].p3[2]);
            }
        }

        // Opaque Faces
        if (type==1)
        {
            for(int i=0; i<mesh->nFace; i++)
            {
                face = &mesh->faces[i];
                if (face->material != actMat)
                {
                    actMat = face->material;
                    mat = &mesh->materials[actMat];
                    glColor4f(mat->r, mat->g, mat->b, mat->a);
                }
                if(mat->a == 1.0f)
                {
                    glNormal3f(face->normal[0],face->normal[1],face->normal[2]);
                    glVertex3f(mesh->triangles->tris[i].p1[0], mesh->triangles->tris[i].p1[1] , mesh->triangles->tris[i].p1[2]);
                    glVertex3f(mesh->triangles->tris[i].p2[0], mesh->triangles->tris[i].p2[1] , mesh->triangles->tris[i].p2[2]);
                    glVertex3f(mesh->triangles->tris[i].p3[0], mesh->triangles->tris[i].p3[1] , mesh->triangles->tris[i].p3[2]);
                }
            }
        }
        // Transparent Faces
        if (type==2)
        {
            for(int i=0; i<mesh->nFace; i++)
            {
                face = &mesh->faces[i];
                if (face->material != actMat)
                {
                    actMat = face->material;
                    mat = &mesh->materials[actMat];
                    glColor4f(mat->r, mat->g, mat->b, mat->a);
                }
                if(mat->a != 1.0f)
                {
                    glNormal3f(face->normal[0],face->normal[1],face->normal[2]);
                    glVertex3f(mesh->triangles->tris[i].p1[0], mesh->triangles->tris[i].p1[1] , mesh->triangles->tris[i].p1[2]);
                    glVertex3f(mesh->triangles->tris[i].p2[0], mesh->triangles->tris[i].p2[1] , mesh->triangles->tris[i].p2[2]);
                    glVertex3f(mesh->triangles->tris[i].p3[0], mesh->triangles->tris[i].p3[1] , mesh->triangles->tris[i].p3[2]);
                }
            }
        }

    }
    else // If is colliding draw all faces with 1.0 Alpha regardless the faces are transparent or not
    {
        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
        for(int i=0; i<mesh->nFace; i++)
        {
            face = &mesh->faces[i];
            glNormal3f(face->normal[0],face->normal[1],face->normal[2]);
            glVertex3f(mesh->triangles->tris[i].p1[0], mesh->triangles->tris[i].p1[1] , mesh->triangles->tris[i].p1[2]);
            glVertex3f(mesh->triangles->tris[i].p2[0], mesh->triangles->tris[i].p2[1] , mesh->triangles->tris[i].p2[2]);
            glVertex3f(mesh->triangles->tris[i].p3[0], mesh->triangles->tris[i].p3[1] , mesh->triangles->tris[i].p3[2]);
        }
    }
}

void RasScene::mousePressEvent(QMouseEvent *event)
{
    lastMousePos = event->pos();
}

void RasScene::mouseMoveEvent(QMouseEvent *event)
{
    float dx = event->x() - lastMousePos.x();
    float dy = event->y() - lastMousePos.y();

    if (event->buttons() & Qt::LeftButton)
    {
        xRot += dy/3.5;
        zRot += dx/2.5;
        xRot = RasMath::normalizeAngle(xRot);
        zRot = RasMath::normalizeAngle(zRot);
        this->update();

    }
    else if (event->buttons() & Qt::RightButton)
    {
        double movZ = fabs(zPos);
        if(movZ < 0.1)
        movZ = 0.1;
        double alto = tan ((angle/2)*M_PI/180) * 2.0 * movZ;
        double ancho = alto * aspect;
        dx *= ancho / this->width();
        dy *= alto / this->height();
        xPos += dx;
        yPos -= dy;
        this->update();
    }
    lastMousePos = event->pos();
}

void RasScene::wheelEvent(QWheelEvent *event)
{

    zPos += event->delta() * 0.0005 * (fabs(zPos) < 0.5 ? 0.5 : fabs(zPos));

    if(zPos < -550.0)
    zPos = -550.0;


    this->update();
    event->accept();
}

// Camera Functions

void RasScene::resetView()
{
    xRot = -45.0;
    yRot = 0;
    zRot = -45.0;

    xPos = 0;
    yPos = 0;
    zPos = -10.0;

    angle = 45.0;
    fovy = 45.0f;
    zNear = 0.1f;
    zFar = 500.0f;
    update();
}
// End of Camera functions

// Robot functions

void RasScene::addRobot(QString filename)
{
    Robots << new RasRobot(filename,this->parent());
    RobotsProcesses << new RasRobotThread(this->parent());

    int i = Robots.length()-1;
    RobotsProcesses[i]->setRobot(Robots[i]);
    connect(RobotsProcesses[i],SIGNAL(finished()),this,SLOT(robotProcessFinished()));
    connect(RobotsProcesses[i],SIGNAL(processUpdated(int,int)),this,SLOT(robotProcessUpdated(int,int)));

    checkCollisions();
}

void RasScene::setRobotLinkQ(double q, int robot, int link, bool refreshBounds)
{
    Robots[robot]->setQ(q,link,false,refreshBounds);
    checkCollisions();
}

void RasScene::setRobotProp(int prop, QVariant value, int robot)
{
    Robots[robot]->setProp(prop,value);
    checkCollisions();
}

void RasScene::resetRobot(int index)
{
    Robots[index]->resetRobot();
    checkCollisions();
}

void RasScene::deleteRobot(int index)
{
    delete Robots.takeAt(index);
    delete RobotsProcesses.takeAt(index);
    checkCollisions();
}

void RasScene::addRobotPath(int robotIndex, RasPath *path, int linkIndex, double appSpeed, double speed)
{
    Robots[robotIndex]->addPath(path,linkIndex,appSpeed,speed);
}

void RasScene::deleteRobotPath(int robotIndex, int pathIndex)
{
    Robots[robotIndex]->deletePath(pathIndex);
}

void RasScene::setInitialPathConfig(int robotIndex)
{
    Robots[robotIndex]->setInitialPathConfig(true);
    checkCollisions();
}

// End of Robot functions


// Simulation functions

void RasScene::startSimulation()
{
    // First emit a signal that simulation has started
    emit simulationStarted();
    simStarted = true;
    simRunning = false;

    for(int i=0; i<Robots.length(); i++)
    {
        if(Robots[i]->nPaths>0)
        {
            Robots[i]->setInitialPathConfig();
            RobotsProcesses[i]->setProcess(1);
            RobotsProcesses[i]->start();
        }
    }
//    emit simulationStopped();

    // Steps of the Simulation
    // 1 - Fix each single path (everything but approachs paths) in order to get equally spaced points to guarantee a constant linear speed
    // 2 - Calculate for each path (even the approach paths) the step time between a point and the next based on distance between the points and path speed.
    // 3 - For the entire real path of each robot, calculate the inverse kinematics, i.e. the reference joint values
    // 4 - For each robot create a timer. Setup the robot's timer time
    // 5 - Each time a timer ticks, check the timer who sent the signal and make the proper update of the robot's configuration based on the timer, the actual path being followed and the actual path point index.
    // 6 - When the robot's configuration is updated then the updateGL() method has to be called.

}

void RasScene::stopSimulation()
{
    // Stop all robot threads and emit the signal.
    for(int i=0; i<RobotsProcesses.length(); i++)
    {
        RobotsProcesses[i]->stopSimulation();
        Robots[i]->refreshBounds();
    }
    this->setEnabled(true);
    simStarted = false;
    simRunning = false;
    checkCollisions();
    emit simulationStopped();
}

void RasScene::pauseSimulation()
{
    // Pause all robot threads and emit the signal.
    for(int i=0; i<RobotsProcesses.length(); i++)
    {
        RobotsProcesses[i]->pauseSimulation();
    }
    simRunning = false;
    checkCollisions();
    emit simulationPaused();
}

void RasScene::resumeSimulation()
{
    // Pause all robot threads and emit the signal.
    for(int i=0; i<RobotsProcesses.length(); i++)
    {
        RobotsProcesses[i]->resumeSimulation();
    }
    simRunning = true;
    checkCollisions();
    emit simulationResumed();
}

void RasScene::robotProcessFinished()
{
    RasRobotThread *robotProcess = static_cast<RasRobotThread*>(sender());
    if(robotProcess->process == 1)
    {
        bool ikFinished = true;
        bool stopped = false;
        for(int i=0; i<RobotsProcesses.length(); i++)
        {
            if(!RobotsProcesses[i]->isFinished())
            {
                ikFinished = false;
                break;
            }
            else
            {
                if(RobotsProcesses[i]->stopped)
                stopped = true;
            }
        }
        if(ikFinished && !stopped && !simRunning)
        {
            simRunning = true;
            // When the IK Computation Finishes for all Robots, Start the Simulation
            // Also update the Graphs Window
            this->setEnabled(true);
            checkCollisions();
            QMessageBox::information(0,"DynaBOT::Simulation::Information","The simulation is about to start. Press OK to continue.");

            for(int i=0; i<Robots.length(); i++)
            {
                RobotsProcesses[i]->setProcess(2);
                RobotsProcesses[i]->start();
            }
            emit simulationResumed();
        }
    }
    if(robotProcess->process == 2)
    {
        bool simFinished = true;
        for(int i=0; i<RobotsProcesses.length(); i++)
        {
            if(RobotsProcesses[i]->isRunning())
            {
                simFinished = false;
                break;
            }
        }
        if(simFinished && simRunning)
        {
            // When the simulation finishes for all Robots, the simulation has ended.
//            qDebug() << "Simulations Done";
            simStarted = false;
            simRunning = false;
            checkCollisions();
            emit simulationStopped();
        }
    }
}

void RasScene::robotProcessUpdated(int procType, int percentage)
{
    if(procType == 1)
    {
        int totalPercentage = 0;
        for(int i=0; i<Robots.length(); i++)
        {
            totalPercentage += RobotsProcesses[i]->processPercentage/Robots.length();
        }
        emit processUpdated("Inverse Kinematics Calculated",totalPercentage);
    }
    if(procType == 2)
    {
        checkCollisions();
        int totalPercentage = 0;
        for(int i=0; i<Robots.length(); i++)
        {
            totalPercentage += RobotsProcesses[i]->processPercentage/Robots.length();
        }
        emit processUpdated("Simulation Running",totalPercentage);
    }
}

// End of Simulation functions

// Obstacle functions

void RasScene::addObstacle(QString filename)
{
    Obstacles << new RasObstacle(filename);
    checkCollisions();
}

void RasScene::setObstacleProperties(QString newname, double tx, double ty, double tz, double rx, double ry, double rz, int index)
{
    Obstacles[index]->setProperties(newname, tx, ty, tz, rx, ry, rz);
    checkCollisions();
}

void RasScene::setObstacleProperties(RasObstacle::RasLocRotProps propertie, QVariant value, int index)
{
    if(propertie == RasObstacle::PropName)
    {
    Obstacles[index]->setProperties(propertie,value.toString());
    }
    else
    {
    Obstacles[index]->setProperties(propertie,value.toDouble());
    checkCollisions();
    }
}

void RasScene::deleteObstacle(int index)
{
    delete Obstacles.takeAt(index);
    checkCollisions();
}

// End Obstacle functions


// Path functions

void RasScene::addPath(QString filename)
{
    Paths << new RasPath(filename);
    updateGL();
}

void RasScene::addPath(double xIni,double yIni, double zIni, double xFin, double yFin, double zFin, int n)
{
    Paths << new RasPath(xIni,yIni,zIni,xFin,yFin,zFin,n);
    updateGL();
}

void RasScene::setPathProperties(int prop, QVariant value, int index)
{
    Paths[index]->setProp(prop,value);
    if(prop != 0)
    updateGL();
}

void RasScene::deletePath(int index)
{
    for(int i=0; i<Robots.length(); i++)
    {
        for(int j=0; j<Robots[i]->nPaths; j++)
        {
            if(Robots[i]->Paths[j]->path == Paths[index])
            Robots[i]->deletePath(j);
        }
    }
    delete Paths.takeAt(index);
    updateGL();
}

// Enf of Path functions

// Toggle functions

void RasScene::toggleBlend(bool state)
{
    enabledBlend = state;

    if(enabledBlend)
    {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    }
    else
    {
        glDisable(GL_BLEND);
    }
    updateGL();
}

void RasScene::toggleGlobalAxis(bool state)
{
    enabledGlobalAxis = state;
    updateGL();
}

void RasScene::toggleLocalAxes(bool state)
{
    enabledLocalAxes = state;
    updateGL();
}

void RasScene::toggleGrid(bool state)
{
    enabledGrid = state;
    updateGL();
}

void RasScene::toggleObjectCoordinates(bool state)
{
    enabledObjectCoordinates = state;
    updateGL();
}

// End of Toggle functions


// Collision functions

void RasScene::checkCollisions()
{
    int nObstacles = Obstacles.length();
    int nRobots = Robots.length();

    // First, all collisions in the Robot Links have to be set to false in order to update them later.
    for(int k=0; k<nRobots; k++)
    {
        for(int i=0; i<Robots[k]->nLinks; i++)
        {
        Robots[k]->Links[i]->isColliding = false;
        }
    }

    if(nObstacles>0)
    {
        switch (collisionType)
        {
        case CollideAllLinks:

            // First, all collisions in the Obstacles have to be set to false.
            for(int i=0; i<nObstacles; i++)
            {
                Obstacles[i]->isColliding = false;
            }

            for(int k=0; k<nRobots; k++)
            {
                for(int i=1; i<Robots[k]->nLinks; i++)
                {
                    for(int j=0; j<nObstacles; j++)
                    {
                        RAPID_Collide(Robots[k]->Links[i]->RLink, Robots[k]->Links[i]->PLink , Robots[k]->Links[i]->mesh->triangles, Obstacles[j]->R, Obstacles[j]->T, Obstacles[j]->mesh->triangles, RAPID_FIRST_CONTACT);

                        if(RAPID_num_contacts > 0)
                        {
                        Obstacles[j]->isColliding = true;
                        Robots[k]->Links[i]->isColliding = true;
                        }
                    }
                }
            }
        break;

        case CollideEndEffector:

            // First, all collisions have to be set to false.
            for(int i=0; i<nObstacles; i++)
            {
                Obstacles[i]->isColliding = false;
            }

            for(int k=0; k<nRobots; k++)
            {
                int i = Robots[k]->nLinks - 1;
                for(int j=0; j<nObstacles; j++)
                {
                    RAPID_Collide(Robots[k]->Links[i]->RLink, Robots[k]->Links[i]->PLink , Robots[k]->Links[i]->mesh->triangles, Obstacles[j]->R, Obstacles[j]->T, Obstacles[j]->mesh->triangles, RAPID_FIRST_CONTACT);

                    if(RAPID_num_contacts > 0)
                    {
                    Obstacles[j]->isColliding = true;
                    Robots[k]->Links[i]->isColliding = true;
                    }
                }
            }
        break;

        case CollideNone:

            // First, all collisions have to be set to false.
            for(int i=0; i<nObstacles; i++)
            {
                Obstacles[i]->isColliding = false;
            }
        break;

        }
    }
    updateGL();
}

void RasScene::setCollisionType(RasTypeCollision type)
{
    collisionType = type;
    checkCollisions();
}

// End of Collision functions
