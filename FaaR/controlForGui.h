#ifndef controlForGui_H
#define controlForGui_H

#include <cmath>
#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include <vector>
#include <QDebug>


#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono.hpp>
#include <boost/bind/bind.hpp>

#include <fstream>

#include "maketrajectory.h"

using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;



class controlForGui
{


public:

    MakeTrajectory trajectory;
    controlForGui();
    controlForGui(FalconDevice& device, double &pX, double &pY, double &pZ, double &pKp, double &pKd, bool &plowPassIsOn );

    /// Communication
    void runIOLoop();
    void read_encoder(gmtl::Vec3d &encoderAngle);
    gmtl::Vec3d addImpedance(gmtl::Vec3d &desiredPos,gmtl::Vec3d& posWithImp);
    void setDesAng(Angle angles, gmtl::Vec3d &desiredAng);
    /// Control
    void FalconLoop();
    void PID(gmtl::Vec3d desired, gmtl::Vec3d encoderAngle);
    void setPid(double nKp, double nKd);
    void limitTorque();
    void sendTorque();
    void sendZeroTorque();
    void readPreviousTime();
    void startNewTime();

    /// Trajectory
    void interPol(gmtl::Vec3d encoderAng, gmtl::Vec3d &goalAngle);
    void resetFirstRun();
    void generateTrajectory();
    void trajectoryPath();
    void getNextPoint(std::vector<double> ang1, std::vector<double> ang2, std::vector<double> ang3);
    void genTrajectoryPath();


    /// Low - pass filter for the D member
    void avgOfArray();
    void lowPassStep(gmtl::Vec3d);

    /// LED control
    void setLedGreen();
    void setLedBlue();
    void setLedRed();

    /// Return functions
    boost::chrono::duration<double> returnLooptime();
    gmtl::Vec3d returnDesiredAng();
    gmtl::Vec3d returnEncoderAngles();
    gmtl::Vec3d returnKp();
    gmtl::Vec3d returnKd();


    /// Initialisation
    void init();
    void set_vectorzero(gmtl::Vec3d vect);

    ///Kinematics
    void FK(const gmtl::Vec3d &theta0);
    void FK(const gmtl::Vec3d &theta0, gmtl::Vec3d &posOut);
    gmtl::Matrix33d jacobian(const Angle &angles);
    void IKoriginal(Angle &angles, const gmtl::Vec3d &worldPosition);
    void IK(Angle &angles, const gmtl::Vec3d& worldPosition, gmtl::Vec3d &importantAng);

    /// Return functions

    gmtl::Vec3d returnoutput();
    gmtl::Vec3d returnpos();
    double returnCount();
    double readTime();

    int l; /// not sure what it's used for

/// Some public variables for easier access -> should write accessor / modifier functions and move theese to private later.
    enum State { goHomeMode, stayMode , followPathMode, logPathMode, replayMode, genTrajMode }; State currentState;

    double time;
    int replayCount ;
    bool firstRun;

    Angle trajAng;
    Angle genTrajAng;

    gmtl::Vec3d homeAng;
    gmtl::Vec3d trajPos;
    gmtl::Vec3d trajTh;
    gmtl::Vec3d genTrajPos;

    std::vector<double> genTrajTh1;
    std::vector<double> genTrajTh2;
    std::vector<double> genTrajTh3;
    std::vector<double> trajTh1;
    std::vector<double> trajTh2;
    std::vector<double> trajTh3;
    std::vector<double> posx;
    std::vector<double> posy;
    std::vector<double> posz;
    double refAngles[][3];
    bool justStayHome;
    ///Logging read
    std::vector<double> logTh0;
    std::vector<double> logTh1;
    std::vector<double> logTh2;

private:
    /// Accessing falcon communication :
    boost::shared_ptr<FalconFirmware> f;
    FalconKinematic* k;
    FalconDevice dev;

    /// Used for PID ///
    Angle angles;
    gmtl::Vec3d pos;
    gmtl::Vec3d mPVect;
    gmtl::Vec3d mDVect;
    gmtl::Vec3d Kdfilter;
    gmtl::Vec3d encoderAngles;
    gmtl::Vec3d prevEncoderAng;
    gmtl::Vec3d desAng;
    gmtl::Vec3d errorVect;
    gmtl::Vec3d prevErrorVect;
    gmtl::Vec3d output;
    gmtl::Vec3d torque;
    gmtl::Vec3d absolutRefPos;
    double dt;
    double Kp;
    double Kd;
    double *mKp;
    double *mKd;



    /// Homing
    gmtl::Vec3d homePos;
    gmtl::Vec3d wayToGo;
    gmtl::Vec3d wayToGoEnd;
    gmtl::Vec3d lamda;
    gmtl::Vec3d errorVectHome;
    gmtl::Vec3d startEncoder;
    gmtl::Vec3d thisIsHome;
    gmtl::Vec3d* goThere;
    gmtl::Vec3d endPos;

    bool* lowPassIsOn;
    bool isAtHome;
    double loopCount;
    bool* isBullshitOn;



    /// FiFo
    double arr[3][10];
    gmtl::Vec3d AVG;
    gmtl::Vec3d SummKd;

    /// Impedance model
    double* mPosX;
    double* mPosY;
    double* mPosZ;

};

#endif // controlForGui_H
