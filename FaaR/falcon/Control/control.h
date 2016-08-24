#ifndef CONTROL_H
#define CONTROL_H


#include <cmath>
#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;


class control
{
public:
    control();
    void read_encoder();
    void Jakobi();
    void postomm();
    void origo(double,double,double);
    void PID();
    void set_PID(double newKp,double newKd,double newKi,double newdt);
    void Dinamics();
    void Nyomatek_hatarolas();
    void Ero_kuld();
    void set_vectorzero(gmtl::Vec3d vect);
    void add_impedance(double,double,double);
    void convertPosToAng();
   // Get input from workspace
    void get_output(double,double,double);
    void getpos(gmtl::Vec3d vect);
    void getangles(Angle ang);
    void get_desired_pos(gmtl::Vec3d despos);
    void get_matyiszarja(gmtl::Vec3d Tg);
   // Return data to workspace
    gmtl::Vec3d return_encoder_angles();
    gmtl::Vec3d return_pos();
    gmtl::Vec3d return_des_angles();
    Angle return_angles();




private:
    boost::array<int, 3> encoderPos;
    gmtl::Vec3d encoderAngles;
    gmtl::Vec3d pos;
    Angle angles;
    gmtl::Matrix33d J;
    gmtl::Vec3d offsetPos;
    gmtl::Vec3d torque;
    gmtl::Vec3d Tg;


    gmtl::Vec3d destination;
    gmtl::Vec3d desAng;
    gmtl::Vec3d prev_desAng;
    gmtl::Vec3d output;
    gmtl::Vec3d hibavektor;
    gmtl::Vec3d elozo_hibavektor;
    gmtl::Vec3d posmm;
    gmtl::Vec3d elozo_posmm;
    gmtl::Vec3d Kpf;
    gmtl::Vec3d Kdf;
    gmtl::Vec3d Kif;
    double dt ;
    double dt2 ;
    double max ;
    double min ;
    // szabályzó paraméterek //
    double Kp ;
    double Kd ;
    double Ki ;
    double t ;
    gmtl::Vec3d Kdfprev;

    boost::shared_ptr<FalconFirmware> firm;
    FalconKinematic* kinematics;

};


void printvect(gmtl::Vec3d);
bool initialise();
void IK(Angle& angles, const gmtl::Vec3d& worldPosition);
gmtl::Matrix33d jacobian(const Angle& angles);
void FK(const gmtl::Vec3d& theta0, gmtl::Vec3d& pos);

#endif // CONTROL_H
