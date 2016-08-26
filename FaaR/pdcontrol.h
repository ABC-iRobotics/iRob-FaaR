#ifndef PDCONTROL_H
#define PDCONTROL_H
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


class PDControl
{
public:
    PDControl(double basecKp, double baseKd, double baset); // Constructor
    gmtl::Vec3d Iteration(gmtl::Vec3d setPoint, gmtl::Vec3d currentPoint);
    void setValues(double newKp,double newKd);
    void avgOfArray();
    void lowPassStep(gmtl::Vec3d KD);
private:
    gmtl::Vec3d mError;
    gmtl::Vec3d mPrevError;
    gmtl::Vec3d mErrorChange;
    gmtl::Vec3d mOutPut;
    gmtl::Vec3d mD; gmtl::Vec3d mDFilter;
    gmtl::Vec3d mP;

    double mKp, mKd,mdt;
    bool isLowPassOn;

    double arr[3][10];
    gmtl::Vec3d mAVG;
    gmtl::Vec3d mSummKd;
};

#endif // PDCONTROL_H
