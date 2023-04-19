#include "RasRobotPath.h"

RasRobotPath::RasRobotPath(int nLinks, RasPath *trajectory, int link, double appSpeed, double velocity)
{
    qIni = new double[nLinks];
    rIni = new double[6];
    path = trajectory;
    approachSpeed = appSpeed;
    speed = velocity;
    linkID = link+1;
}

RasRobotPath::~RasRobotPath()
{
    delete [] qIni;
}
