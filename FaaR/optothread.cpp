#include "optothread.h"


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
    Config_default(optoDaq,optoPorts,0);            // Optoforce sensor paramétereit beállítjuk (Freq, Filter)
    Offsex(optoDaq,Offsetx,Offsety,Offsetz);        // Mintavétel-> átlagolás -> globalis offset értéket átállítjuk nulláról
    Offset[0]=Offsetx;                              // Sima értékadás tömbnek
    Offset[1]=Offsety;
    Offset[2]=Offsetz;
    Xkord.setPos(0);
    Ykord.setPos(0);
    Zkord.setPos(0);
    emit offsetDone();


    while(1)
    {

        double elapsedTime= double(time.nsecsElapsed())/1000000000;
     //   std::cout<<"Eltelt idő: "<< elapsedTime << " Seconds"<<std::endl;
        time.restart();
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


ReadForce(optoDaq,Offset,&Fx,&Fy,&Fz);


double Force[3] = {Fx,Fy,Fz};
QMutex iteracio;
iteracio.lock();
Xkord.Iteracio(Force,0); Ykord.Iteracio(Force,1); Zkord.Iteracio(Force,2);
iteracio.unlock();

//std::cout << Xkord.getPos() << " // " << Ykord.getPos() << " // " << Zkord.getPos() << std::endl;
double x = 1; //skála
emit xkord(x*Xkord.getPos());
emit ykord(x*Ykord.getPos());
emit zkord(x*Zkord.getPos());
//std::cout<<Xkord.getTime()<<std::endl;
usleep(500);
QMutex muci;
muci.lock();
if(this->Stop)
{
Xkord.setV(1);
Ykord.setV(1);
Zkord.setV(1);

    emit sensorStopped();
    optoDaq.close();
    break;}
muci.unlock();
     }


}
