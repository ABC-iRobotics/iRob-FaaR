#include "falconthreads.h"
#include <fstream>

FalconThreads::FalconThreads()
{
}

RT_TASK periodic_real_time_task; //Here we can define tasks
RT_TASK controlTask;
RT_MUTEX mutex_desc;
RT_PIPE pipe_desc;

ofstream myLogFile;

//type for the variables of the real time task
struct t_rt_var
{
    int i; //index parameter
    unsigned int n; //Not used now
    bool Stop;
    bool falconStop;

};
static t_rt_var rt_var;


//Close log file, but now it isn't used
void FalconThreads::CloseLog()
{
   //logging.close();
}

void controlLoop(void *arg)
{
    rt_var.falconStop = false;
    std::cout << "ControlLoop running." << std::endl;
    reinterpret_cast<controlForGui *>(arg)->setLedGreen();


    RTIME now, previos; //Define now and previous time
    //Set task periodic - Arguments: &task (NULL=self),start time,period (here: 1 s)
    rt_task_set_periodic(NULL, TM_NOW, 1 * T_ms);
    previos = rt_timer_read();
    double totalTime = 0;
    double loopTime = 0;
    int loopCountofLog = 1;

    gmtl::Vec3d encoderAng;
    bool prevModeWasLog = false;
    controlForGui* ctrl = reinterpret_cast<controlForGui *>(arg);


    while(1)
    {

        rt_task_wait_period(NULL);

        if (!rt_var.falconStop)
        {
        ctrl->FalconLoop();
        encoderAng=ctrl->returnEncoderAngles();

        if(ctrl->currentState == ctrl->logPathMode)
        {

            //std::cout<< "ifen belüli " << std::endl;
            if(myLogFile.is_open()==true)
            {
                loopCountofLog++;
                //std::cout<< " file megnyitva " << std::endl;
                myLogFile << encoderAng[0] << " " << encoderAng[1] <<" " << encoderAng[2] << endl;
                prevModeWasLog = true;

            }
            else
            {
                myLogFile.open("log.dat" ,ios::trunc);
            }
        }
        if(prevModeWasLog==true && ctrl->currentState!=ctrl->logPathMode)
        {
        // ha az előző mód loggolás volt, fáljt zárjuk be
        std::cout<<"[Logging off] number of entries: " << loopCountofLog<< std::endl;
        loopCountofLog=1;
        myLogFile.close();
        prevModeWasLog=false;
        }



        }
        if (rt_var.falconStop)
        {
        ctrl->runIOLoop();
        ctrl->setLedRed();
        ctrl->resetFirstRun();
        }
            now = rt_timer_read();
        double ido;
        ido = (double)(now - previos) / 1000000;
        previos = now;


          totalTime = totalTime+ido;
          loopTime = ido;
       //ctrl->readTime(ido);
    }
}

//This fuction starts the real time task

void FalconThreads::OpenLog(std::string openedFileName)
{
    ifstream logfile;
    logfile.open(openedFileName.c_str());
    if(logfile.is_open())
    {
        mControl.logTh0.clear();
        mControl.logTh1.clear();
        mControl.logTh2.clear();

        std::cout<< "[Open Log] Pályát tartalmazó file megnyitva!" << std::endl;
        mControl.replayCount = 0;
        while(!logfile.eof())
        {
            logfile >> logTh0V >> logTh1V >> logTh2V;
            logCount.push_back(logCountV);
            mControl.logTh0.push_back(logTh0V);
            mControl.logTh1.push_back(logTh1V);
            mControl.logTh2.push_back(logTh2V);
            mControl.replayCount++;
        }
    }
            std::cout<< "[Open Log] Pálya pontjai betárazva" << mControl.replayCount <<std::endl;
    logfile.close();
}


void FalconThreads::InitThreads()
{
    static int err;
    //Create a mutex
    err = rt_mutex_create(&mutex_desc, "MyMutex");

    //Creatre a rt pipe
    err = rt_pipe_create(&pipe_desc, NULL, 0, 6);
    if (err) printf("RT PIPE CREATE FAIL\n");

    //Avoids memory swapping for this program
    mlockall(MCL_CURRENT|MCL_FUTURE);
    firstTimeRun = true;
    ///Trajectory planning
    //gmtl::Vec3d p0(0, 0, 110);
    //gmtl::Vec3d p1(-20, 30, 130);

    //gmtl::Vec3d v(0.62, 0.21, 0.41);
   // mControl.trajectory.trajectory(p0, p1, nn);
    //mControl.trajectoryPath();

}

void FalconThreads::CloseThreads(void)
{
    //Close RT PIPE - Arguments: &RT_PIPE
    rt_pipe_delete(&pipe_desc);

    //Delete MUTEX - Arguments: &RT_MUTEX
    rt_mutex_delete(&mutex_desc);

    std::cout << "Threads stopped.\n" << std::endl;

}



void FalconThreads::startFalcon()
{
    if (firstTimeRun)
    {
    Kp = 2;
    Kd = 0.025;
    lowPassfilter=false;
    FalconDevice device;

    controlForGui control(device, posX, posY, posZ, Kp, Kd, lowPassfilter);
    mControl = control;
    rt_task_create(&controlTask, "falconLoop", 0, 99, 0);
    rt_task_start(&controlTask, controlLoop, &mControl);
    std::cout << "Init done.\n" << std::endl;
    myLogFile.open("log.dat" ,ios::trunc);
    }
    else
    {
        rt_var.falconStop = false;
    }
}

void FalconThreads::stopFalcon()
{
    //Stop periodic task - Arguments: &task
   // mControl.closeComm();
    //rt_task_suspend(&controlTask);
    rt_var.falconStop=true;
    firstTimeRun = false;
    //std::cout <<"Falcon stopped.\n";
}
