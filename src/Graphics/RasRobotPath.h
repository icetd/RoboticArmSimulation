#ifndef RASROBOTPATH_H
#define RASROBOTPATH_H

#include "RasPath.h"

class RasRobotPath
{
public:
    RasRobotPath(int nLinks, RasPath *trajectory, int link, double appSpeed, double velocity);
    ~RasRobotPath();

    double *qIni;
    double *rIni;
    RasPath *path;
    int linkID;
    double approachSpeed;
    double speed;
};

#endif // RASROBOTPATH_H
