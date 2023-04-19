#ifndef __RASSCENE_H
#define __RASSCENE_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QtOpenGL/QGLWidget>
#include <QOpenGLContext>
#include <QTime>

#include "RasObstacle.h"
#include "RasRobotThread.h"
#include "RasRobot.h"
#include "RasPath.h"


class RasScene : public QGLWidget
{
	Q_OBJECT

public:
	enum RasTypeCollision {
		CollideAllLinks,
		CollideEndEffector,
		CollideNone
	};
	
	QList<RasObstacle*>        Obstacles;
    QList<RasRobotThread*>   	RobotsProcesses;
    QList<RasRobot*>           Robots;
    QList<RasPath*>            Paths;

	RasScene(QWidget *parent = nullptr);
	virtual ~RasScene();

	/* reset Camera */
	void resetView();

	/* Robot functions */

	void addRobot(QString filename);
    void setRobotLinkQ(double q, int robot, int link, bool refreshBounds = false);
    void setRobotProp(int prop, QVariant value, int robot);
    void resetRobot(int index);
    void deleteRobot(int index);
    void addRobotPath(int robotIndex, RasPath *path, int linkIndex, double appSpeed, double speed);
    void deleteRobotPath(int robotIndex, int pathIndex);
    void setInitialPathConfig(int robotIndex);
	/* Simulation functions */
	void startSimulation();
	void stopSimulation();
	void pauseSimulation();
	void resumeSimulation();
	/* Obstacle functions */
	void addObstacle(QString filename);
    void setObstacleProperties(QString newname, double tx, double ty, double tz, double rx, double ry, double rz, int index);
    void setObstacleProperties(RasObstacle::RasLocRotProps propertie, QVariant value, int index);
    void deleteObstacle(int index);
	/* Path functions */
	void addPath(QString filename);
    void addPath(double xIni,double yIni, double zIni, double xFin, double yFin, double zFin, int n);
    void setPathProperties(int prop, QVariant value, int index);
    void deletePath(int index);
	/* Toggle functions */
	void toggleBlend(bool state);
    void toggleGlobalAxis(bool state);
    void toggleLocalAxes(bool state);
    void toggleGrid(bool state);
    void toggleObjectCoordinates(bool state);
	/* Collision functions */
    void checkCollisions();
    void setCollisionType(RasTypeCollision type);
	
	void resetScene();
	bool simStarted;
    bool simRunning;

private:
	QPoint lastMousePos;
	float xRot;
	float yRot;
	float zRot;

	float xPos;
	float yPos;
	float zPos;

	float angle;
	double aspect;
	double fovy;
	double zNear;
	double zFar;

	bool painting;

	bool enabledBlend;
	bool enabledGlobalAxis;
	bool enabledLocalAxes;
	bool enabledObjectCoordinates;
	bool enabledGrid;
	
	RasTypeCollision collisionType;

	QTime _fpsTime;
	unsigned int _fpsCounter;
	QString _fpsString;
	float _fps;

signals:
	void simulationStarted();
	void simulationStopped();
	void simulationPaused();
	void simulationResumed();
	void processUpdated(QString process, int percentage);

public slots:
    void robotProcessFinished();
    void robotProcessUpdated(int procType, int percentage);


protected:
	void initializeGL() override;
	void resizeGL(int width, int height) override;
	void paintGL() override;
	
	QOpenGLContext *m_context;

	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;

	void drawCoordinates(double x, double y, double z);
	void drawBox(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax);
	void drawGrid(float size, int nbSubdivisions = 10);
	void drawMesh(RasMesh *mesh, bool isColliding = false, int type = 0);
	void drawAxis(double length, bool opaque = true);
	void drawArrow(float length, int nbSubdivisions = 10);

private:
	bool isinitialized = false;
};

#endif