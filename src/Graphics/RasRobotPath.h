#ifndef QDYNROBOTPATH_H
#define QDYNROBOTPATH_H

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

#endif // QDYNROBOTPATH_H
