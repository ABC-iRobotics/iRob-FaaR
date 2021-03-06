#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <falcon/threads/falconthreads.h>
#include <falcon/threads/optothread.h>
#include "qcustomplot.h"
#include <QTimer>
#include <QListWidget>
#include <QMessageBox>
#include <QDebug>
#include <fstream>
#include <QFileDialog>
#include <falconDataStruct.h>
#include <QDir>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QMessageBox showMessage;
    FalconThreads *falconThreads;
    OptoThread *oThread;
    labelUpdate *labelUpdateThread;
    QTimer dataTimer;
    bool stopPlot;
    bool startPlot;
    double plot1Val;
    double plot2Val;
    double plot3Val;
    double plot1Min;
    double plot2Min;
    double plot3Min;
    double plot1Max;
    double plot2Max;
    double plot3Max;
    int size;
    gmtl::Vec3d p0;
    gmtl::Vec3d p1;
    int nn;

    bool isFalconReady;

    void setupencoderplot2();
    void setupencoderplot3();
    void setupencoderplot1();

    ///Generate trajectory
    QString stringx;
    QString stringy;
    QString stringz;
    double genX;
    double genY;
    double genZ;
    int genCount = 0;

    double stepSize = 0.00001;


    gmtl::Vec3d addPos;

public slots:
    void onXchange(double);
    void onYchange(double);
    void onZchange(double);

    void encoderplot1();
    void encoderplot2();
    void encoderplot3();

    void sensorConfigStatus(bool);
    void offssetStatus(bool);

    void slotToUpdateLabel();

private slots:

    /*void on_startThreads_clicked();
    void on_stopThreads_clicked();*/
    void on_startSensor_clicked();
    void on_stopProgram_clicked();
    void on_startFalcon_clicked();
    void on_stopFalcon_clicked();
    void on_MassChanger_valueChanged(int value);
    void on_SpringChanger_valueChanged(int value);
    void on_newKd_valueChanged(double arg1);
    void on_newKp_valueChanged(double arg1);
    void on_lowPassFilter_toggled(bool checked);
    void on_DamperChanger_valueChanged(int value);


    void on_startPlot_clicked();

    void on_stopPlot_clicked();

    void on_plot1List_itemDoubleClicked(QListWidgetItem *item);

    void on_plot2List_itemDoubleClicked(QListWidgetItem *item);

    void on_plot3List_itemDoubleClicked(QListWidgetItem *item);

    void on_plot1Min_valueChanged(double arg1);

    void on_plot1Max_valueChanged(double arg1);

    void on_plot2Min_valueChanged(double arg1);
    void on_plot2Max_valueChanged(double arg1);
    void on_plot3Min_valueChanged(double arg1);
    void on_plot3Max_valueChanged(double arg1);
    void on_ModeStartButton_clicked();

    void on_valueMass_editingFinished();

    void on_valueSpting_editingFinished();

    void on_valueDamper_editingFinished();

    void on_trajectoryPlot_clicked();

    void on_openLog_clicked();

    void on_startLogPath_clicked();

    void on_stopLogging_clicked();

    void on_addPath_clicked();

    void on_genTraj_clicked();

    void on_genNew_clicked();

    void on_replayMode_clicked();


    void on_resetPosAndVel_clicked();

    void on_FalconFeedback_clicked();

    void on_buttonFalconShowCoords_clicked();


    void on_btnXdec_clicked();

    void on_btnXinc_clicked();

    void on_btnYinc_clicked();

    void on_btnYdec_clicked();

    void on_btnZinc_clicked();

    void on_btnZdec_clicked();

    void on_lineEdit_textEdited(const QString &arg1);

    void on_navMode_clicked();

private:

    double key;
    Ui::MainWindow *ui;

    ///bullshit dolgok elkerülésére
    int threadsStarted = 0;
    int sensorStarted = 0;
    int falconStarted = 0;
    bool posShowPauseIsOn;
};

#endif // MAINWINDOW_H
