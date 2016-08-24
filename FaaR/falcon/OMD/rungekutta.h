#ifndef RUNGEKUTTA_H
#define RUNGEKUTTA_H

#include <cmath>
#include <iostream>

class RungeKutta
{ public:
    RungeKutta();
    RungeKutta(double& m,double& K,double& C,double& h);
    ~RungeKutta();
    void resetPosAndVel();
    void setPos(double);
    void setV(double);
    void setTime(double);
    void setMass(double);
    void setC(double);
    void setK(double);
    void setStep(double);

    void Iteracio(double Force[3],int index);

    double getPos();
    double getTime();
    double getF();





private:

    double pos;
    double v;
    double h;
    double C, K, m;
    double Time;
    double F;

    double* mC;
    double* mK;
    double* mm;
    double* mh;
};

#endif
