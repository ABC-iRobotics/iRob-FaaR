#ifndef OPTOTHREAD_H
#define OPTOTHREAD_H

#include <QThread>
#include <QtCore>
#include <falcon/OMD/optosource.h>
#include <falcon/OMD/rungekutta.h>
#include <QElapsedTimer>
#include <QDebug>
class OptoThread : public QThread
{
    Q_OBJECT
public:
    explicit OptoThread(QObject *parent = 0);
    void run();
    bool Stop;
    double returnXpos;
    double returnYpos;
    double returnZpos;
    double ph,pm,pC,pK;

    RungeKutta X;
    RungeKutta Y;
    RungeKutta Z;

    QElapsedTimer time;
signals:
    void xkord(double);
    void ykord(double);
    void zkord(double);
    void sensorInitialized();
    void connectionError();
    void offsetDone();
    void sensorStopped();
};

#endif // OPTOTHREAD_H