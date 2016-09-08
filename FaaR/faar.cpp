#include "faar.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    falconThreads = new FalconThreads;
    oThread= new OptoThread(this);
    labelUpdateThread = new labelUpdate(this);

    connect(oThread,SIGNAL(xkord(double)),this,SLOT(onXchange(double)));
    connect(oThread,SIGNAL(ykord(double)),this,SLOT(onYchange(double)));
    connect(oThread,SIGNAL(zkord(double)),this,SLOT(onZchange(double)));
    connect(oThread,SIGNAL(configSet(bool)),this,SLOT(sensorConfigStatus(bool)));
    connect(oThread,SIGNAL(offsetSet(bool)),this,SLOT(offssetStatus(bool)));

    connect(labelUpdateThread,SIGNAL(updateSignal()),this,SLOT(slotToUpdateLabel()));


posShowPauseIsOn == true;
}

MainWindow::~MainWindow()
{
    delete ui;
}

///Sensor stuff
void::MainWindow::onXchange(double kord)
{
//ui->label_9->setText(QString::number(kord));
QMutex mx;
mx.lock();
falconThreads->posX = kord;
ui->sensorX->setText(QString::number(kord));
mx.unlock();

}
void::MainWindow::onYchange(double kord)
{
//ui->label_10->setText(QString::number(kord));
QMutex my;
my.lock();
falconThreads->posY = kord;
ui->sensorY->setText(QString::number(kord));
my.unlock();

}
void::MainWindow::onZchange(double kord)
{
//ui->label_11->setText(QString::number(kord));
QMutex mz;
mz.lock();
falconThreads->posZ = kord;
ui->sensorZ->setText(QString::number(kord));
mz.unlock();
}
void::MainWindow::sensorConfigStatus(bool status)
{
    if (status == true)  // If connection is up, set feedback label , and feedback button
    {
ui->labelSensorConnection->setText("Sensor connected");
ui->SensoFeedback->setText("ON");
ui->SensoFeedback->setStyleSheet("background-color: green");
    }
    else  // else display error and set feedback
    {
ui->labelSensorConnection->setText("Sensor Could not open port");
ui->SensoFeedback->setText("OFF");
ui->SensoFeedback->setStyleSheet("background-color: red");


/// Show message box about the problem ///
QMessageBox msgBox;
msgBox.setText("Could not open port to sensor, try reconnecting it to the computer and restart the program");
msgBox.exec();

    }

}
void::MainWindow::offssetStatus(bool status)
{
    if (status == true)  // If connection is up, set feedback label , and feedback button
    {
        ui->labelSensorOffset->setText("Sensor Offset Done");
    }
    else
    {
        ui->labelSensorOffset->setText("Sensor Offset Error");
        ui->labelSensorRunning->setText("Sensor is Running");

    }

}
void MainWindow::on_resetPosAndVel_clicked()
{
    oThread->reset=true;
    falconThreads->mControl.currentState=falconThreads->mControl.goHomeMode;

}


void MainWindow::on_startThreads_clicked()
{
    threadsStarted++;

    if(threadsStarted > 1)
    {
        showMessage.warning(this, "", "Threads are already started, stop it first.");
    }
    else
    {

        falconThreads->InitThreads();
        //ui->writeOut->append("Initiation done.");
         ui->ThreadFeedback->setText("ON");
        ui->ThreadFeedback->setStyleSheet("background-color: green");
        ui->labelThreadFeedback->setText("Thread running");
        labelUpdateThread->start();
    }
}
void MainWindow::on_stopThreads_clicked()
{
    threadsStarted = 0;
    falconThreads->CloseThreads();
   // ui->writeOut->append("Threads stopped.");
    ui->ThreadFeedback->setText("OFF");
    ui->ThreadFeedback->setStyleSheet("background-color: red");
}
void MainWindow::on_startSensor_clicked()
{
    sensorStarted++;
    if(sensorStarted > 1)
    {
        showMessage.warning(this, "", "The sensor is already started, stop it first.");
    }
    else
    {
        oThread->Stop = false;
        oThread->start();
    }
}
void MainWindow::on_stopProgram_clicked()
{
    sensorStarted = 0;
    oThread->Stop = true;
    ui->labelSensorConnection->setText("Sensor Stopped");
    ui->labelSensorOffset->setText("Sensor offset -none");
    ui->labelSensorRunning->setText("Sensor not running");
    ui->SensoFeedback->setText("OFF");
    ui->SensoFeedback->setStyleSheet("background-color: red");
}
void MainWindow::on_startFalcon_clicked()
{
    falconStarted++;
    if(falconStarted > 1)
    {
        showMessage.warning(this, "", "Falcon is already started, stop it first.");
    }
    else
    {
        falconThreads->startFalcon();
        //ui->writeOut->append("Falcon running.");
        on_FalconFeedback_clicked();
    }
}
void MainWindow::on_stopFalcon_clicked()
{
    falconStarted = 0;
    falconThreads->stopFalcon();
   // ui->writeOut->append("Falcon stopped.");
    ui->FalconFeedback->setText("OFF");
    ui->FalconFeedback->setStyleSheet("background-color: red");
}
/// Impedance
void MainWindow::on_MassChanger_valueChanged(int value)
{
    oThread->pm=value;
    //ui->LabelMassValue->setText(QString::number(value));
    ui->valueMass->setText(QString::number(value));
}
void MainWindow::on_SpringChanger_valueChanged(int value)
{
    oThread->pK=value;
    //ui->LabelSpringVal->setText(QString::number(value));
    ui->valueSpting->setText(QString::number(value));
}
void MainWindow::on_DamperChanger_valueChanged(int value)
{
    oThread->pC=value;
    //ui->LabelDamperVal->setText(QString::number(value));
    ui->valueDamper->setText(QString::number(value));
}
void MainWindow::on_valueMass_editingFinished()
{
    ui->MassChanger->setValue(ui->valueMass->text().toInt());
}
void MainWindow::on_valueSpting_editingFinished()
{
    ui->SpringChanger->setValue(ui->valueSpting->text().toInt());
}
void MainWindow::on_valueDamper_editingFinished()
{
    ui->DamperChanger->setValue(ui->valueDamper->text().toInt()/10);
}
/// PID
void MainWindow::on_newKd_valueChanged(double arg1)
{
    falconThreads->Kd=arg1;
}
void MainWindow::on_newKp_valueChanged(double arg1)
{
    falconThreads->Kp=arg1;
}
void MainWindow::on_lowPassFilter_toggled(bool checked)
{
    falconThreads->lowPassfilter=checked;
}
/// Operating Mode
void MainWindow::on_ModeStartButton_clicked()
{
    if (ui->centerMode->isChecked())
    {
       /* falconThreads->justStayHere=true;
        falconThreads->goToPoint=false;*/
        falconThreads->mControl.currentState=falconThreads->mControl.stayMode;
        ui->labelOperationMode->setText("Stay Mode");

    }
    else if(ui->goHomeMode->isChecked())
    {
        falconThreads->mControl.currentState=falconThreads->mControl.goHomeMode;
        falconThreads->mControl.resetFirstRun();
        ui->labelOperationMode->setText("Go home mode");
    }

    else if(ui->replayMode->isChecked())
    {
        falconThreads->mControl.currentState=falconThreads->mControl.replayMode;
        ui->labelOperationMode->setText("Replay mode");
    }
}
void MainWindow::on_trajectoryPlot_clicked()
{
    size = falconThreads->mControl.trajectory.count;
    //std::cout << "Sample size: " << size << std::endl;
    QVector<double> n(size), vel1(size), vel21(size), vel22(size),  vel31(size),  vel32(size), vel2(size), vel3(size), vx(size), vy(size), vz(size) , ax(size), ay(size), az(size);
    for (int i=0; i<size; ++i)
    {
      n[i] = falconThreads->mControl.trajectory.n[i];
      //px[i] = falconThreads->mControl.trajectory.x[i];
      //py[i] = falconThreads->mControl.posx[i];
      //py[i] = falconThreads->mControl.trajectory.y[i];
      //pz[i] = falconThreads->mControl.trajectory.z[i];
      vx[i] = falconThreads->mControl.trajectory.vx[i];
      vy[i] = falconThreads->mControl.trajectory.vy[i];
      vz[i] = falconThreads->mControl.trajectory.vz[i];
      ax[i] = falconThreads->mControl.trajectory.ax[i];
      ay[i] = falconThreads->mControl.trajectory.ay[i];
      az[i] = falconThreads->mControl.trajectory.az[i];
      switch(ui->trajPlot1List->currentRow())
      {
          case 0:
            vel1[i] = falconThreads->mControl.trajectory.x[i];
            vel2[i] = falconThreads->mControl.posx[i];
            //ui->trajectoryPos->graph(0)->setName("trajectory x");
            //ui->trajectoryPos->graph(1)->setName("robot x");
            break;
          case 1:
            vel1[i] = falconThreads->mControl.trajectory.y[i];
            vel2[i] = falconThreads->mControl.posy[i];
            //ui->trajectoryPos->graph(0)->setName("trajectory y");
            //ui->trajectoryPos->graph(1)->setName("robot y");
            break;
          case 2:
            vel1[i] = falconThreads->mControl.trajectory.z[i];
            vel2[i] = falconThreads->mControl.posz[i];
            //ui->trajectoryPos->graph(0)->setName("trajectory z");
            //ui->trajectoryPos->graph(1)->setName("robot z");
            break;
      }
      switch(ui->trajPlot2List->currentRow())
      {
          case 0:
            vel21[i] = falconThreads->mControl.trajectory.x[i];
            vel22[i] = falconThreads->mControl.posx[i];
            break;
          case 1:
            vel21[i] = falconThreads->mControl.trajectory.y[i];
            vel22[i] = falconThreads->mControl.posy[i];
            break;
          case 2:
            vel21[i] = falconThreads->mControl.trajectory.z[i];
            vel22[i] = falconThreads->mControl.posz[i];
            break;
      }
      switch(ui->trajPlot3List->currentRow())
      {
          case 0:
            vel31[i] = falconThreads->mControl.trajectory.x[i];
            vel32[i] = falconThreads->mControl.posx[i];
            break;
          case 1:
            vel31[i] = falconThreads->mControl.trajectory.y[i];
            vel32[i] = falconThreads->mControl.posy[i];
            break;
          case 2:
            vel31[i] = falconThreads->mControl.trajectory.z[i];
            vel32[i] = falconThreads->mControl.posz[i];
            break;
      }

    }

    ///Plot trajectory position
    // create graph and assign data to it:
    ui->trajectoryPos->addGraph();
    ui->trajectoryPos->graph(0)->setData(n, vel1);
    ui->trajectoryPos->graph(0)->setName("trajectory x");
    ui->trajectoryPos->addGraph();
    ui->trajectoryPos->graph(1)->setData(n, vel2);
    ui->trajectoryPos->graph(1)->setPen(QPen(Qt::red));
    ui->trajectoryPos->addGraph();
    ui->trajectoryPos->graph(2)->setData(n, vel3);
    ui->trajectoryPos->graph(2)->setPen(QPen(Qt::green));
    // give the axes some labels:
    ui->trajectoryPos->xAxis->setLabel("n");
    ui->trajectoryPos->yAxis->setLabel("p [mm]");
    ui->trajectoryPos->rescaleAxes();
    ui->trajectoryPos->replot();

    ///Plot trajectory Velocity
    // create graph and assign data to it:
    ui->trajectoryVel->addGraph();
    ui->trajectoryVel->graph(0)->setData(n, vel21);
    ui->trajectoryVel->addGraph();
    ui->trajectoryVel->graph(1)->setData(n, vel22);
    ui->trajectoryVel->graph(1)->setPen(QPen(Qt::red));
    ui->trajectoryVel->addGraph();
    ui->trajectoryVel->graph(2)->setData(n, vel3);
    ui->trajectoryVel->graph(2)->setPen(QPen(Qt::green));
    // give the axes some labels:
    ui->trajectoryVel->xAxis->setLabel("n");
    ui->trajectoryVel->yAxis->setLabel("v [mm/s]");
    ui->trajectoryVel->rescaleAxes();
    ui->trajectoryVel->replot();

    ///Plot trajectory Acceleration
    // create graph and assign data to it:
    ui->trajectoryAcc->addGraph();
    ui->trajectoryAcc->graph(0)->setData(n, vel31);
    ui->trajectoryAcc->addGraph();
    ui->trajectoryAcc->graph(1)->setData(n, vel32);
    ui->trajectoryAcc->graph(1)->setPen(QPen(Qt::red));
    ui->trajectoryAcc->addGraph();
    ui->trajectoryAcc->graph(2)->setData(n, vel3);
    ui->trajectoryAcc->graph(2)->setPen(QPen(Qt::green));
    // give the axes some labels:
    ui->trajectoryAcc->xAxis->setLabel("n");
    ui->trajectoryAcc->yAxis->setLabel("v [mm/s^2]");
    ui->trajectoryAcc->rescaleAxes();
    ui->trajectoryAcc->replot();
}
void MainWindow::on_openLog_clicked()
{

    QString QFileName = QFileDialog::getOpenFileName(
                this,
                tr("Open file"),
                "/home/matyas/google_drive/Qt_projects",
                "All files (*.*)"

                );
    std::string fileName =QFileName.toStdString();
 falconThreads->OpenLog(fileName);
    ui->replayMode->setCheckable(true);
}
void MainWindow::on_startLogPath_clicked()
{
    falconThreads->mControl.currentState=falconThreads->mControl.logPathMode;
    std::cout<< "[Logging path started]"<<std::endl;
}
void MainWindow::on_stopLogging_clicked()
{
    falconThreads->mControl.currentState=falconThreads->mControl.goHomeMode;
    falconThreads->mControl.resetFirstRun();
    std::cout<< "[Logging path ended]"<<std::endl;
    ///check that the log directory is existing
    QDir dir(".");
    QDir logPath("./Logs/teachmode");
    if(!logPath.exists())
    {
        dir.mkpath("./Logs/teachmode");
    }
    ///
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                               "./Logs/teachmode",
                               tr("Log Files (*.dat)"));
    fileName.append(".dat");
    qDebug()<< fileName;

   bool succesfulCopy = QFile::copy("./log.dat", fileName);
   if (succesfulCopy)
   {
       qDebug() << "file saved to: " + fileName;
   }
   else
   {
       qDebug() << "the file coudnt be saved ,but the temporary file can be found with the data (log.dat) ";
   }


}
void MainWindow::on_addPath_clicked()
{
    genX = ui->pathX->text().toDouble(); // kiolvas és konvertál doublebe
    genY = ui->pathY->text().toDouble();
    genZ = ui->pathZ->text().toDouble();
    stringx = QString::number(genX);     // kiiráshoz
    stringy = QString::number(genY);
    stringz = QString::number(genZ);
    ui->console->append(QString::number(genCount) + "." + "    x: " + stringx + "     y: " + stringy + "    z: " + stringz);
    falconThreads->mControl.trajectory.addPath(genX, genY, genZ);
    falconThreads->mControl.trajectory.genPoints = genCount;  //
    qDebug() << "genCount: " << falconThreads->mControl.trajectory.genPoints << endl;
    falconThreads->mControl.trajectory.genVectX.push_back(genX);
    falconThreads->mControl.trajectory.genVectY.push_back(genY);
    falconThreads->mControl.trajectory.genVectZ.push_back(genZ);
    genCount++;
}
void MainWindow::on_genTraj_clicked()
{
    falconThreads->mControl.trajectory.generateTrajectory();
    for(int i = 0; i < falconThreads->mControl.trajectory.genCount; i++)
    {
        ui->genLog->append(/*QString::number(falconThreads->mControl.genTrajTh1[i]) + "       " + */QString::number(falconThreads->mControl.trajectory.genX[i]));
    }

}
void MainWindow::on_genNew_clicked()
{
    ui->console->clear();
    ui->pathX->clear();
    ui->pathY->clear();
    ui->pathZ->clear();
    genCount = 0;
    //falconThreads->mControl.trajectory
}
void MainWindow::on_replayMode_clicked()
{
    if(!ui->replayMode->isCheckable())
    {
        showMessage.information(this, "", "First you need to logging and/or load the path by clicking on the 'Log path' and/or 'Open log' button.");
    }
}

void MainWindow::on_FalconFeedback_clicked()
{
    if (falconThreads->mControl.args.isItConnected)
    {
        ui->labelFalconConnected->setText("Falcon found");
    }
    if (falconThreads->mControl.args.isItFound && falconThreads->mControl.args.isItConnected)
    {
        ui->labelFalconConnected->setText("Falcon connected");

    }

    if (falconThreads->mControl.args.isItFound && falconThreads->mControl.args.isItConnected && falconThreads->mControl.args.isFirmWareLoaded)
    {
        ui->labelFalconInited->setText("Firmware Loaded");
        ui->labelFalconStatus->setText("Falcon Ready");
        ui->FalconFeedback->setText("ON");
        ui->FalconFeedback->setStyleSheet("background-color: green");
        ui->labelOperationMode->setText("Go home mode");
        isFalconReady = true;

    }
    else
    {
        ui->labelFalconInited->setText("Could not load firmware");
        QMessageBox msgBox;
        ui->FalconFeedback->setText("OFF");
        ui->FalconFeedback->setStyleSheet("background-color: red");
        msgBox.setText("Falcon could not be initialised properly, try restarting the program");
        msgBox.exec();
        isFalconReady = false;
    }


}


void MainWindow::on_buttonFalconShowCoords_clicked()
{
    if (posShowPauseIsOn==false)
    { posShowPauseIsOn = true; }
    if (posShowPauseIsOn == true)
    { posShowPauseIsOn= false;}

}


void MainWindow::slotToUpdateLabel()
{

if (!posShowPauseIsOn)
    {
        double xPos = falconThreads->mControl.returnpos()[0];
        double yPos = falconThreads->mControl.returnpos()[1];
        double zPos = falconThreads->mControl.returnpos()[2];
        QString xPosstr = QString::number(xPos,'f',6);
        QString yPosstr = QString::number(yPos,'f',6);
        QString zPosstr = QString::number(zPos,'f',6);
        ui->labelFalconXpos->setText(xPosstr);
        ui->labelFalconYpos->setText(yPosstr);
        ui->labelFalconZpos->setText(zPosstr);
    }
}
