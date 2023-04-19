#include "RasRobotThread.h"

RasRobotThread::RasRobotThread(QObject *parent) : QThread(parent)
{
    stopped = false;
}

void RasRobotThread::setRobot(RasRobot *Bot)
{
    Robot = Bot;
    connect(Robot,SIGNAL(processState(int,int)),this,SLOT(processState(int,int)));
}

void RasRobotThread::setProcess(int proc)
{
    process = proc;
}

void RasRobotThread::run()
{
    stopped = false;
    paused = false;

    switch(process)
    {
        case 0:
        break;
        case 1: // Inverse Kinematics

            if(!Robot->ikSolved)
            {
                Robot->calculateFullPath();

                if(stopped)
                return;

                Robot->calculateInverseKinematics();
                Robot->calculateRobotDynamics();
//                Robot->calculateDynamicResponses();

            }
            else
            {
                processPercentage = 100;
                emit processUpdated(1,100);
            }
        break;
        case 2: // Simulation
            Robot->actSimState = 0;
            while(Robot->actSimState < Robot->nPoints)
            {
                if(stopped)
                return;

                if(!paused)
                {
                    Robot->simulationStep();
                    msleep((int) floor((Robot->tRef[Robot->actSimState - 1] * 1000) + 0.5));
                }
                else
                {
                    msleep(15);
                }
            }
        break;
    }
}

void RasRobotThread::stopSimulation()
{
    stopped = true;
    Robot->stopCalculation();
}

void RasRobotThread::pauseSimulation()
{
    paused = true;
}

void RasRobotThread::resumeSimulation()
{
    paused = false;
}

void RasRobotThread::processState(int procType, int percentage)
{
    processPercentage = percentage;
    emit processUpdated(procType,percentage);
}
