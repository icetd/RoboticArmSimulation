#ifndef QDYNROBOTTHREAD_H
#define QDYNROBOTTHREAD_H

#include <QThread>
#include "RasRobot.h"

class RasRobotThread : public QThread
{
    Q_OBJECT

public:

    RasRobotThread(QObject *parent);
    void setRobot(RasRobot *Bot);
    void setProcess(int proc);
    void run();
    void stopSimulation();
    void pauseSimulation();
    void resumeSimulation();

    RasRobot *Robot;
    int process;
    int processPercentage;
    bool stopped;
    bool paused;

signals:
//    void ikSolved();
    void processUpdated(int procType, int percentage);

public slots:
    void processState(int procType, int percentage);
};

#endif // QDYNROBOTIKTHREAD_H
