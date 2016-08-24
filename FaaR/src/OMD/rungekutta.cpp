#include "falcon/OMD/rungekutta.h"

RungeKutta::RungeKutta()
{

    pos = 0;
    v = 0;
    h = 0.0001;
    C = 1;
    K = 200;
    m = 5;
    Time = 0;
}


RungeKutta::RungeKutta(double &pm, double &pK, double &pC, double &ph)
{
    mK=&pK;
    mC=&pC;
    mm=&pm;
    mh=&ph;
    pos = 0;
    v = 0;
    h = *mh;
    C = *mC;
    K = *mK;
    m = *mm;
    Time = 0;

}


RungeKutta::~RungeKutta()
{
}
 /// Modifier functions
void RungeKutta::setPos(double newpos)
{
    pos = newpos;
}

void RungeKutta::setV(double newV)
{
    v = 0;
    pos =0;
}

void RungeKutta::setMass(double newMass)
{
 m=newMass;
}


void RungeKutta::setC(double newC)
{
 C=newC;
}


void RungeKutta::setK(double newK)
{
 K=newK;
}


void RungeKutta::setStep(double newStep)
{
 h=newStep;
}

  /// Iteration step
void RungeKutta::Iteracio(double Force[3],int index)
{

    F = Force[index];

    double dv = h * ((F - C*v - K*pos)/m);
    double dpos = h * v;
/*
        /// This section is not working as intended, so it's been replaced by an even simpler method

    double dpos1 = h*v;
    double dv1 = (F/m) - (C / m)*v - (K / m)*pos;

    double dpos2 = h*(v + (dv1 / 2));
    double dv2 = h*((F / m) - ((C / m)*(v + (dv1 / 2))) - (K / m)*(pos + (dpos1 / 2)));

    double dpos3 = h*(v + (dv2 / 2));
    double dv3 = h*((F / m) - ((C / m)*(v + (dv2 / 2))) - (K / m)*(pos + (dpos2 / 2)));

    double dpos4 = h*(v + (dv3));
    double dv4 = h*((F / m) - ((C / m)*(v + (dv3))) - (K / m)*(pos + (dpos3)));

    double dpos = (dpos1 + dpos2 * 2 + 2 * dpos3 + dpos4) / 6;
    double dv = (dv1 + dv2 * 2 + 2 * dv3 + dv4) / 6;
*/
    pos = pos + dpos;
    v = v + dv;
    Time = Time + h;

}


    /// Accessor functions here
double RungeKutta::getPos()
{
    return pos;
}
double RungeKutta::getTime()
{
    return Time;
}

double RungeKutta::getF()
{
    return F;
}
