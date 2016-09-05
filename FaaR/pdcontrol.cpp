#include "pdcontrol.h"

PDControl::PDControl(double baseKp, double baseKd, double baset)
{
mKp=baseKp;
mKd =baseKd;
mdt = baset;
}

gmtl::Vec3d PDControl::Iteration(gmtl::Vec3d setPoint, gmtl::Vec3d currentPoint)
{
mError = setPoint-currentPoint; // calculate the difference
mP = mError*mKp;                // scale it
mErrorChange=mError-mPrevError; // calcualte derivate
lowPassStep(mErrorChange);      // add last element to array, and calculate last 10 element


//Decide wheter to apply low-pass filter or not//
if (isLowPassOn == true)
{
    mD = mErrorChange* mKd / mdt;
}
if (isLowPassOn == false)
{
    mD = mDFilter* mKd /mdt;
}

mPrevError = mError;            // store current error to the prev
mOutPut=mP+mD;                  // sum members
return mOutPut;
}

void PDControl::setValues(double newKp, double newKd)
{
    // Overwrite Kp and Kd values //
    
    mKp = newKp;
    mKd = newKd;
}

void PDControl::avgOfArray()
{
    /// Calculates the average of the the array
    for (int j = 0; j<3 ; j++)
    {   mSummKd[j] = 0;
        for (int i = 0; i< 10 ; i++)        { mSummKd[j] += arr[j][i];  }
        mAVG[j] = mSummKd[j]/10;
        mDFilter[j] = mAVG[j];
    }
}
void PDControl::lowPassStep(gmtl::Vec3d KD)
{
    /// deletes last element in an array, pushes the others back by 1 place, read new value to the 0. cell
     for (int j = 0 ; j<3 ; j++)
   {    for (int i = 9; i > 0 ; i--) {arr[j][i] = arr[j][i-1];}
    arr[j][0]=KD[j];
   }
 avgOfArray();
}
