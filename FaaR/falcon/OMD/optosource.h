#ifndef OPTOSOURCE_H
#define OPTOSOURCE_H

#include <iostream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include "omd/opto.h"
#include "omd/sensorconfig.h"
#include "omd/optopackage.h"



typedef unsigned long long mytime_t;

class OptoSource
{
public:
    OptoSource();
};

mytime_t Now();
mytime_t NowMicro();
mytime_t ElapsedTime(mytime_t p_Time);
mytime_t ElapsedTimeMicro(mytime_t p_Time);
void MySleep(unsigned long p_uMillisecs);
bool SetConfig(OptoDAQ & p_optoDAQ, int p_iSpeed, int p_iFilter);
void ShowInformation(OptoDAQ & p_optoDAQ, OPort & p_Port);
bool OpenPort(OptoDAQ & p_optoDAQ, OptoPorts & p_Ports, int p_iIndex);
void Offseteles(OptoDAQ & poptodaq, double &Offx, double &Offy, double &Offz);
void ReadForce(OptoDAQ & p_optodaq, double offsetek[], double* Fx, double* Fy, double* Fz);
void Config_default(OptoDAQ & p_optodaq, OptoPorts & p_optoPorts, int iPortindex);
void Offsex(OptoDAQ & optoDaq,double & Offsetx,double & Offsety,double & Offsetz);
/**/
#endif // OPTOSOURCE_H
