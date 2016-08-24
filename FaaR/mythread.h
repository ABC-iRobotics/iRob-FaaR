#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <QThread>
#include <QTimer>

#include "falcon/core/FalconLogger.h"
#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include "stdint.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include <cmath>
#include <fstream>

#include "controlForGui.h"






class MyThread : public QThread
{
    Q_OBJECT

public:

    explicit MyThread(QObject *parent = 0);
    void run();
    bool Stop;
    double posX;
    double posY;
    double posZ;
    controlForGui control;
    double Kp;
    double Kd;

    double Th1;
    double Th2;
    double Th3;

signals:

    void NumberChanged0(double);
    void NumberChanged1(double);
    void NumberChanged2(double);

public slots:


};



void callBack( boost::asio::deadline_timer* t, bool* pStop ,controlForGui* ctrl);



#endif // MYTHREAD_H



















/*
void MyThread::falconloop()
{
    while(1)
        {
                //std::cout<< this->Stop << " Külső nagy while" << std::endl;
            while(1)
            {

                if(this->Stop)
                {

                    break;
                }
                //std::cout<< this->Stop<< " Control while " << std::endl;
                control.runIOLoop();

            ///Encoder szöget beolvasása, radiánba konvertálása
                control.read_encoder();
                gmtl::Vec3d desipos(0,0,0.12);
                desipos[0] += (posX/5000.0);
                desipos[1] += (posY/5000.0);
                desipos[2] -= (posZ/5000.0);

            /// Inverz
                control.IK(desipos);
                control.setDesAng();
                control.PID();
                control.sendTorque();
                control.setLedGreen();
                //std::cout<< posX << " // " << posY << "  //  " << posZ << std::endl;


                //myfile<< count << "    " << encoderAngles[0] << "    " << encoderAngles[1] << "    " << encoderAngles[2]<< "   " << encoderAngles[0] << "    " << encoderAngles[1] << "    " << encoderAngles[2] <<endl;


    /*            emit NumberChanged0(encoderAngles[0]);
                emit NumberChanged1(encoderAngles[1]);
                emit NumberChanged2(encoderAngles[2]);*/
                //this->msleep(100);
               // count++;

 /*           }
            while(this->Stop)
            {
                control.runIOLoop();
                control.setLedRed();
                control.sendZeroTorque();
                //std::cout<< this->Stop <<" Üres while " << std::endl;
            }

          }
    }*/
