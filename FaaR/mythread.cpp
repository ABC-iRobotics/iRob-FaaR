#include "mythread.h"
#include <QtCore>




MyThread::MyThread(QObject *parent) :
    QThread(parent)
{

}



/// Lokális változók
ofstream logFile;
boost::chrono::duration<double> loopTime;
boost::chrono::duration<double> totalTime;
gmtl::Vec3d encoderAng;
gmtl::Vec3d desAng;
gmtl::Vec3d Kp;
gmtl::Vec3d Kd;
gmtl::Vec3d hiba;

using namespace libnifalcon;
using namespace StamperKinematicImpl;


void callBack( boost::asio::deadline_timer* t, bool* pStop ,controlForGui* ctrl)
{

   if (1)
    {
        t->expires_at(t->expires_at() +boost::posix_time::milliseconds(1.0));
        t->async_wait(boost::bind(callBack, t, pStop, ctrl));
    }

   if (*pStop == false)
   {

    //std::cout<< "pStop = "<< *pStop << std::endl;
    gmtl::Vec3d desipos(0,0,0.12);
    ctrl->FalconLoop(desipos);

    /// loggolás:
    // idő
    loopTime = ctrl->returnLooptime();
    totalTime +=+loopTime;
    // Encoder, hibák, P , D
    encoderAng=ctrl->returnEncoderAngles();
    desAng=ctrl->returnDesiredAng();
    Kp=ctrl->returnKp();
    Kd=ctrl->returnKd();
    hiba = desAng-encoderAng;

   logFile<<totalTime<<";"<<loopTime<<";"<<encoderAng[0]<<";"<<encoderAng[1]<<";"<<encoderAng[2]<<";"<<hiba[0]<<";"<<hiba[1]<<";"
         <<hiba[2]<<";"<<Kp[0]<<";"<<Kp[1]<<";"<<Kp[2]<<";"<<Kd[0]<<";"<<Kd[1]<<";"<<Kd[2]<<" log"<<endl;



   }
    if (*pStop == true)
    {
        ctrl->runIOLoop();
        ctrl->setLedRed();
        ctrl->resetFirstRun();
        ctrl->sendZeroTorque();
   //std::cout<< "pStop = "<< *pStop << std::endl;
    }
}
void MyThread::run()
{
    logFile.open("log.dat");

    logFile << "Total time;"<<" loopTime;" << "Encoder[0];"<<"Encoder[1];"<<"Encoder[2];"<<"hibaEncoder[0];"
            <<"hibaEncoder[1];"<<"hibaEncoder[2];"<<"P_tag[0];"<<"P_tag[1];"<<"P_tag[2];"<<"D_tag[0];"
            <<"D_tag[1];"<<"D_tag[2];"<<endl;
    Kp=0 ; Kd = 0.00;


    FalconDevice device;
    controlForGui controll(device,posX,posY,posZ,Kp,Kd);
    control = controll;


usleep(100);

    boost::asio::io_service io;

     int count = 0;
     boost::asio::deadline_timer t(io, boost::posix_time::milliseconds(1.0));
     t.async_wait(boost::bind(callBack, &t, &Stop, &controll));


     io.run();



     //std::cout << "Final count is " << count << "\n";




}


