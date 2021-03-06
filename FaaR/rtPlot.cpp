#include "faar.h"
#include "ui_mainwindow.h"

void MainWindow::setupencoderplot1()
{
  // include this section to fully disable antialiasing for higher performance:

  ui->encoderPlot1->setNotAntialiasedElements(QCP::aeAll);
  QFont font;
  font.setStyleStrategy(QFont::NoAntialias);
  ui->encoderPlot1->xAxis->setTickLabelFont(font);
  ui->encoderPlot1->yAxis->setTickLabelFont(font);
  ui->encoderPlot1->legend->setFont(font);

  ui->encoderPlot1->addGraph(); // blue line
  ui->encoderPlot1->graph(0)->setPen(QPen(Qt::blue));
  ui->encoderPlot1->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
  ui->encoderPlot1->graph(0)->setAntialiasedFill(false);
  ui->encoderPlot1->addGraph(); // red line
  ui->encoderPlot1->graph(1)->setPen(QPen(Qt::red));
  ui->encoderPlot1->graph(0)->setChannelFillGraph(ui->encoderPlot1->graph(1));

  ui->encoderPlot1->addGraph(); // blue dot
  ui->encoderPlot1->graph(2)->setPen(QPen(Qt::blue));
  ui->encoderPlot1->graph(2)->setLineStyle(QCPGraph::lsNone);
  ui->encoderPlot1->graph(2)->setScatterStyle(QCPScatterStyle::ssDisc);
  ui->encoderPlot1->addGraph(); // red dot
  ui->encoderPlot1->graph(3)->setPen(QPen(Qt::red));
  ui->encoderPlot1->graph(3)->setLineStyle(QCPGraph::lsNone);
  ui->encoderPlot1->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);

  ui->encoderPlot1->xAxis->setTickLabelType(QCPAxis::ltDateTime);
  ui->encoderPlot1->xAxis->setDateTimeFormat("hh:mm:ss");
  ui->encoderPlot1->xAxis->setAutoTickStep(false);
  ui->encoderPlot1->xAxis->setTickStep(2);
  ui->encoderPlot1->axisRect()->setupFullAxesBox();

  // make left and bottom axes transfer their ranges to right and top axes:
  //connect(ui->encoderPlot1->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->encoderPlot1->xAxis2, SLOT(setRange(QCPRange)));
  //connect(ui->encoderPlot1->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->encoderPlot1->yAxis2, SLOT(setRange(QCPRange)));

  // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(encoderplot1()));
  dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void MainWindow::encoderplot1()
{

#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  key = 0;
#else
  key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
  // calculate two new data points:

  switch(ui->plot1List->currentRow())
  {
      case 0: plot1Val = falconThreads->mControl.returnEncoderAngles()[0]; break;
      case 1: plot1Val = falconThreads->mControl.returnEncoderAngles()[1];break;
      case 2: plot1Val = falconThreads->mControl.returnEncoderAngles()[2];break;
      case 3: plot1Val = falconThreads->mControl.returnDesiredAng()[0];break;
      case 4: plot1Val = falconThreads->mControl.returnDesiredAng()[1];break;
      case 5: plot1Val = falconThreads->mControl.returnDesiredAng()[2];break;
      case 6: plot1Val = falconThreads->mControl.returnpos()[0];break;
      case 7: plot1Val = falconThreads->mControl.returnpos()[1];break;
      case 8: plot1Val = falconThreads->mControl.returnpos()[2];break;
      case 9: plot1Val = falconThreads->mControl.returnKp()[0];break;
      case 10:plot1Val = falconThreads->mControl.returnKd()[0];break;
      case 11:plot1Val = falconThreads->mControl.time; break;
      case 12:plot1Val = falconThreads->mControl.returnoutput()[0]; break;
  }


  static double lastPointKey = 0;
  if (key-lastPointKey > 0.001) // at most add point every 1000 ms
  {

    //double value0 = falconThreads->mControl.returnEncoderAngles()[0]; //qSin(key*1.6+qCos(key*1.7)*2)*10 + qSin(key*1.2+0.56)*20 + 26;
    //double value1 = qCos(key); //qSin(key*1.3+qCos(key*1.2)*1.2)*7 + qSin(key*0.9+0.26)*24 + 26;
    //std::cout <<plot1Val << std::endl;

    // add data to lines:
    ui->encoderPlot1->graph(0)->addData(key, plot1Val);
    //ui->encoderPlot1->graph(1)->addData(key, falconThreads->mControl.returnDesiredAng()[0]);
    // set data of dots:
    ui->encoderPlot1->graph(2)->clearData();
    ui->encoderPlot1->graph(2)->addData(key, plot1Val);
    ui->encoderPlot1->graph(3)->clearData();
    //ui->encoderPlot1->graph(3)->addData(key, falconThreads->mControl.returnDesiredAng()[0]);
    // remove data of lines that's outside visible range:
    ui->encoderPlot1->graph(0)->removeDataBefore(key-8);
    //ui->encoderPlot1->graph(1)->removeDataBefore(key-8);
    // rescale value (vertical) axis to fit the current data:
    ui->encoderPlot1->graph(0)->rescaleValueAxis();
    //ui->encoderPlot1->graph(1)->rescaleValueAxis(true);
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->encoderPlot1->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
  if(!ui->plot1Auto->isChecked())
  {
    ui->encoderPlot1->yAxis->setRange(plot1Min, plot1Max);
  }
  ui->encoderPlot1->replot();

  // calculate frames per second:
  static double lastFpsKey;
  static int frameCount;
  ++frameCount;
  if (key-lastFpsKey > 2) // average fps over 2 seconds
  {
    ui->statusBar->showMessage(
          QString("%1 FPS, Total Data points: %2")
          .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
          .arg(ui->encoderPlot1->graph(0)->data()->count()+ui->encoderPlot1->graph(1)->data()->count())
          , 0);
    lastFpsKey = key;
    frameCount = 0;
  }
}

void MainWindow::setupencoderplot2()
{

  // include this section to fully disable antialiasing for higher performance:

  ui->encoderPlot2->setNotAntialiasedElements(QCP::aeAll);
  QFont font;
  font.setStyleStrategy(QFont::NoAntialias);
  ui->encoderPlot2->xAxis->setTickLabelFont(font);
  ui->encoderPlot2->yAxis->setTickLabelFont(font);
  ui->encoderPlot2->legend->setFont(font);

  ui->encoderPlot2->addGraph(); // blue line
  ui->encoderPlot2->graph(0)->setPen(QPen(Qt::blue));
  ui->encoderPlot2->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
  ui->encoderPlot2->graph(0)->setAntialiasedFill(false);
  ui->encoderPlot2->addGraph(); // red line
  ui->encoderPlot2->graph(1)->setPen(QPen(Qt::red));
  ui->encoderPlot2->graph(0)->setChannelFillGraph(ui->encoderPlot2->graph(1));

  ui->encoderPlot2->addGraph(); // blue dot
  ui->encoderPlot2->graph(2)->setPen(QPen(Qt::blue));
  ui->encoderPlot2->graph(2)->setLineStyle(QCPGraph::lsNone);
  ui->encoderPlot2->graph(2)->setScatterStyle(QCPScatterStyle::ssDisc);
  ui->encoderPlot2->addGraph(); // red dot
  ui->encoderPlot2->graph(3)->setPen(QPen(Qt::red));
  ui->encoderPlot2->graph(3)->setLineStyle(QCPGraph::lsNone);
  ui->encoderPlot2->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);

  ui->encoderPlot2->xAxis->setTickLabelType(QCPAxis::ltDateTime);
  ui->encoderPlot2->xAxis->setDateTimeFormat("hh:mm:ss");
  ui->encoderPlot2->xAxis->setAutoTickStep(false);
  ui->encoderPlot2->xAxis->setTickStep(2);
  ui->encoderPlot2->axisRect()->setupFullAxesBox();

  // make left and bottom axes transfer their ranges to right and top axes:
  //connect(ui->encoderPlot1->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->encoderPlot1->xAxis2, SLOT(setRange(QCPRange)));
  //connect(ui->encoderPlot1->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->encoderPlot1->yAxis2, SLOT(setRange(QCPRange)));

  // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(encoderplot2()));
  dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void MainWindow::encoderplot2()
{

#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  key = 0;
#else
  key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
  // calculate two new data points:

  static double lastPointKey = 0;
  if (key-lastPointKey > 0.001) // at most add point every 1000 ms
  {

    //double value0 = falconThreads->mControl.returnEncoderAngles()[1]; //qSin(key*1.6+qCos(key*1.7)*2)*10 + qSin(key*1.2+0.56)*20 + 26;
    //double value1 = qCos(key); //qSin(key*1.3+qCos(key*1.2)*1.2)*7 + qSin(key*0.9+0.26)*24 + 26;
    //std::cout << pista << std::endl;

      switch(ui->plot2List->currentRow())
      {
          case 0: plot2Val = falconThreads->mControl.returnEncoderAngles()[0]; break;
          case 1: plot2Val = falconThreads->mControl.returnEncoderAngles()[1]; break;
          case 2: plot2Val = falconThreads->mControl.returnEncoderAngles()[2]; break;
          case 3: plot2Val = falconThreads->mControl.returnDesiredAng()[0]; break;
          case 4: plot2Val = falconThreads->mControl.returnDesiredAng()[1]; break;
          case 5: plot2Val = falconThreads->mControl.returnDesiredAng()[2]; break;
          case 6: plot2Val = falconThreads->mControl.returnpos()[0];break;
          case 7: plot2Val = falconThreads->mControl.returnpos()[1];break;
          case 8: plot2Val = falconThreads->mControl.returnpos()[2];break;
          case 9: plot2Val = falconThreads->mControl.returnKp()[0]; break;
          case 10: plot2Val = falconThreads->mControl.returnKd()[0]; break;
      }

    // add data to lines:
    ui->encoderPlot2->graph(0)->addData(key, plot2Val);
    //ui->encoderPlot1->graph(1)->addData(key, value1);
    // set data of dots:
    ui->encoderPlot2->graph(2)->clearData();
    ui->encoderPlot2->graph(2)->addData(key, plot2Val);
    //ui->encoderPlot1->graph(3)->clearData();
    //ui->encoderPlot1->graph(3)->addData(key, value1);
    // remove data of lines that's outside visible range:
    ui->encoderPlot2->graph(0)->removeDataBefore(key-8);
    //ui->encoderPlot1->graph(1)->removeDataBefore(key-8);
    // rescale value (vertical) axis to fit the current data:
    ui->encoderPlot2->graph(0)->rescaleValueAxis();
    //ui->encoderPlot1->graph(1)->rescaleValueAxis(true);
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->encoderPlot2->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
  if(!ui->plot2Auto->isChecked())
  {
    ui->encoderPlot2->yAxis->setRange(plot2Min, plot2Max);
  }
  ui->encoderPlot2->replot();
}


void MainWindow::setupencoderplot3()
{

  // include this section to fully disable antialiasing for higher performance:

  ui->encoderPlot3->setNotAntialiasedElements(QCP::aeAll);
  QFont font;
  font.setStyleStrategy(QFont::NoAntialias);
  ui->encoderPlot3->xAxis->setTickLabelFont(font);
  ui->encoderPlot3->yAxis->setTickLabelFont(font);
  ui->encoderPlot3->legend->setFont(font);

  ui->encoderPlot3->addGraph(); // blue line
  ui->encoderPlot3->graph(0)->setPen(QPen(Qt::blue));
  ui->encoderPlot3->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
  ui->encoderPlot3->graph(0)->setAntialiasedFill(false);
  ui->encoderPlot3->addGraph(); // red line
  ui->encoderPlot3->graph(1)->setPen(QPen(Qt::red));
  ui->encoderPlot3->graph(0)->setChannelFillGraph(ui->encoderPlot3->graph(1));

  ui->encoderPlot3->addGraph(); // blue dot
  ui->encoderPlot3->graph(2)->setPen(QPen(Qt::blue));
  ui->encoderPlot3->graph(2)->setLineStyle(QCPGraph::lsNone);
  ui->encoderPlot3->graph(2)->setScatterStyle(QCPScatterStyle::ssDisc);
  ui->encoderPlot3->addGraph(); // red dot
  ui->encoderPlot3->graph(3)->setPen(QPen(Qt::red));
  ui->encoderPlot3->graph(3)->setLineStyle(QCPGraph::lsNone);
  ui->encoderPlot3->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);

  ui->encoderPlot3->xAxis->setTickLabelType(QCPAxis::ltDateTime);
  ui->encoderPlot3->xAxis->setDateTimeFormat("hh:mm:ss");
  ui->encoderPlot3->xAxis->setAutoTickStep(false);
  ui->encoderPlot3->xAxis->setTickStep(2);
  ui->encoderPlot3->axisRect()->setupFullAxesBox();

  // make left and bottom axes transfer their ranges to right and top axes:
  //connect(ui->encoderPlot1->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->encoderPlot1->xAxis2, SLOT(setRange(QCPRange)));
  //connect(ui->encoderPlot1->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->encoderPlot1->yAxis2, SLOT(setRange(QCPRange)));

  // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(encoderplot3()));
  dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}



void MainWindow::encoderplot3()
{

#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  key = 0;
#else
  key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
  // calculate two new data points:

  static double lastPointKey = 0;
  if (key-lastPointKey > 0.001) // at most add point every 1000 ms
  {

    //double value0 = falconThreads->mControl.returnEncoderAngles()[2]; //qSin(key*1.6+qCos(key*1.7)*2)*10 + qSin(key*1.2+0.56)*20 + 26;
    //double value1 = qCos(key); //qSin(key*1.3+qCos(key*1.2)*1.2)*7 + qSin(key*0.9+0.26)*24 + 26;
    //std::cout << pista << std::endl;

    // add data to lines:
      switch(ui->plot3List->currentRow())
      {
          case 0: plot3Val = falconThreads->mControl.returnEncoderAngles()[0]; break;
          case 1: plot3Val = falconThreads->mControl.returnEncoderAngles()[1]; break;
          case 2: plot3Val = falconThreads->mControl.returnEncoderAngles()[2]; break;
          case 3: plot3Val = falconThreads->mControl.returnDesiredAng()[0]; break;
          case 4: plot3Val = falconThreads->mControl.returnDesiredAng()[1]; break;
          case 5: plot3Val = falconThreads->mControl.returnDesiredAng()[2]; break;
          case 6: plot3Val = falconThreads->mControl.returnpos()[0];break;
          case 7: plot3Val = falconThreads->mControl.returnpos()[1];break;
          case 8: plot3Val = falconThreads->mControl.returnpos()[2];break;
          case 9: plot3Val = falconThreads->mControl.returnKp()[0]; break;
          case 10: plot3Val = falconThreads->mControl.returnKd()[0]; break;
      }




    ui->encoderPlot3->graph(0)->addData(key, plot3Val);
    //ui->encoderPlot1->graph(1)->addData(key, value1);
    // set data of dots:
    ui->encoderPlot3->graph(2)->clearData();
    ui->encoderPlot3->graph(2)->addData(key, plot3Val);
    //ui->encoderPlot1->graph(3)->clearData();
    //ui->encoderPlot1->graph(3)->addData(key, value1);
    // remove data of lines that's outside visible range:
    ui->encoderPlot3->graph(0)->removeDataBefore(key-8);
    //ui->encoderPlot1->graph(1)->removeDataBefore(key-8);
    // rescale value (vertical) axis to fit the current data:
    ui->encoderPlot3->graph(0)->rescaleValueAxis();
    //ui->encoderPlot1->graph(1)->rescaleValueAxis(true);
    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->encoderPlot3->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
  if(!ui->plot3Auto->isChecked())
  {
    ui->encoderPlot3->yAxis->setRange(plot3Min, plot3Max);
  }
  ui->encoderPlot3->replot();
}


void MainWindow::on_startPlot_clicked()
{
    stopPlot = false;
    setWindowTitle("Real time plot");
        setupencoderplot1();
        setupencoderplot2();
        setupencoderplot3();

        /*if(stopPlot)
          {
              ui->encoderPlot1->replot();
          }*/
          //std::cout << key << std::endl;
}


void MainWindow::on_stopPlot_clicked()
{

}


void MainWindow::on_plot1List_itemDoubleClicked(QListWidgetItem *item)
{
    item = ui->plot1List->currentItem();
}


void MainWindow::on_plot2List_itemDoubleClicked(QListWidgetItem *item)
{
    item = ui->plot2List->currentItem();

}


void MainWindow::on_plot3List_itemDoubleClicked(QListWidgetItem *item)
{
    item = ui->plot3List->currentItem();

}


void MainWindow::on_plot1Min_valueChanged(double arg1)
{
    plot1Min = arg1;
}


void MainWindow::on_plot1Max_valueChanged(double arg1)
{
    plot1Max = arg1;
}


void MainWindow::on_plot2Min_valueChanged(double arg1)
{
    plot2Min = arg1;
}


void MainWindow::on_plot2Max_valueChanged(double arg1)
{
    plot2Max = arg1;
}


void MainWindow::on_plot3Min_valueChanged(double arg1)
{
    plot3Min = arg1;
}


void MainWindow::on_plot3Max_valueChanged(double arg1)
{
    plot3Max = arg1;
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
