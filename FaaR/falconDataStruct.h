#ifndef FALCONDATASTRUCT_H
#define FALCONDATASTRUCT_H
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

using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;

struct falconData{
    FalconDevice *device;


 /// Impedance input ///
    double *posX;
    double *posY;
    double *posZ;

 /// PID ///
    double *Kp;
    double *Kd;

    bool *lowPassIsOn;
};

#endif // FALCONDATASTRUCT_H
