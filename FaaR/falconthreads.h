#ifndef FALCONTHREADS_H
#define FALCONTHREADS_H

//******************************************************************
//***********************REAL TIME PART*****************************
//******************************************************************
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <native/pipe.h>

//Include files for Logging data
#include <iostream>
#include <fstream>
#include <string>
#define T_ms 1  // = SysClock / 1000

#include "controlForGui.h"

class FalconThreads
{
public:
    FalconThreads();

    void InitThreads();
    void CloseThreads(void);
    void OpenLog(string openedFileName);
    void CloseLog(); //Not used now

    void startProgram();
    void stopProgram();
    void startFalcon();
    void stopFalcon();
    controlForGui mControl;



    double posX;
    double posY;
    double posZ;
    double Kp;
    double Kd;
    bool falconStop;
    bool firstTimeRun;
    bool lowPassfilter;
    std::vector<double> logCount;

private:
    double logCountV;
    double logTh0V;
    double logTh1V;
    double logTh2V;

};

#endif // FALCONTHREADS_H
