#include "falcon/threads/optothread.h"


OptoThread::OptoThread(QObject *parent):QThread(parent)
{
    ph = 0.000001;
    pC = 1;
    pK = 200;
    pm = 5;
    time.start();
}

RungeKutta Xkord;
RungeKutta Ykord;
RungeKutta Zkord;

OptoDAQ optoDaq;
OptoPorts optoPorts;
double Offsetx = 0;
double Offsety = 0;
double Offsetz = 0;
double Fx = 0;
double Fy = 0;
double Fz = 0;
double Force[3];
double Offset[3];



double returnXpos()
{
    return Xkord.getPos();
}
double returnYpos()
{
    return Ykord.getPos();
}
double returnZpos()
{
    return Zkord.getPos();
}



void OptoThread::run()
{

    emit sensorInitialized();
    Config_default(optoDaq,optoPorts,0);            // Set sensor parameters
    OffsetAll(optoDaq,Offsetx,Offsety,Offsetz);     // Calculate offset values from 5 samples
    Offset[0]=Offsetx;                              // Array initialisation
    Offset[1]=Offsety;
    Offset[2]=Offsetz;
    Xkord.setPos(0);
    Ykord.setPos(0);
    Zkord.setPos(0);
    emit offsetDone();


    while(1)
    {

        double elapsedTime= double(time.nsecsElapsed())/1000000000;          // time measure and convert from ns to s
        time.restart();                                                      // restart timer


    /// Update parameters from gui data

Xkord.setMass(pm);
Xkord.setStep(elapsedTime);
Xkord.setC(pC);
Xkord.setK(pK);

Ykord.setMass(pm);
Ykord.setStep(elapsedTime);
Ykord.setC(pC);
Ykord.setK(pK);

Zkord.setMass(pm);
Zkord.setStep(elapsedTime);
Zkord.setC(pC);
Zkord.setK(pK);

 /// read force from sensor
ReadForce(optoDaq,Offset,&Fx,&Fy,&Fz);


double Force[3] = {Fx,Fy,Fz};

 /// iteration step
QMutex iteracio;
iteracio.lock();
Xkord.Iteracio(Force,0); Ykord.Iteracio(Force,1); Zkord.Iteracio(Force,2);
iteracio.unlock();


double x = 1; //scaling
emit xkord(x*Xkord.getPos());
emit ykord(x*Ykord.getPos());
emit zkord(x*Zkord.getPos());


usleep(500);  /// some sleep
QMutex muci;
muci.lock();
if(this->Stop)
{
    /// if sensor is stopped, reinitialize position
    /// and velocity in iteration modell
Xkord.resetPosAndVel();
Ykord.resetPosAndVel();
Zkord.resetPosAndVel();


    emit sensorStopped();
    optoDaq.close();
break;}
muci.unlock();
if(this->reset)
{
    Xkord.resetPosAndVel();
    Ykord.resetPosAndVel();
    Zkord.resetPosAndVel();

reset=false;

}



    }


}
