#ifndef MAKETRAJECTORY_H
#define MAKETRAJECTORY_H

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "kdl/velocityprofile_trap.hpp"
#include "falcon/gmtl/gmtl.h"
#include <QDebug>
#include <iostream>
#include <fstream>

#include <cmath>

class MakeTrajectory
{

public:
    KDL::Path_RoundedComposite* genPath = new KDL::Path_RoundedComposite(0.0001 ,0.0001, new KDL::RotationalInterpolation_SingleAxis());
    MakeTrajectory();
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> vx;
    std::vector<double> vy;
    std::vector<double> vz;
    std::vector<double> ax;
    std::vector<double> ay;
    std::vector<double> az;
    std::vector<int>n;

    ///Generate traj.
    std::vector<double> genX;
    std::vector<double> genY;
    std::vector<double> genZ;
    std::vector<double> genVx;
    std::vector<double> genVy;
    std::vector<double> genVz;
    std::vector<double> genAx;
    std::vector<double> genAy;
    std::vector<double> genAz;
    std::vector<double> genN;
    std::vector<double> genVectX;
    std::vector<double> genVectY;
    std::vector<double> genVectZ;
    int genCount;
    int szam;
    int genPoints;
    void addPath(double x, double y, double z);




    void makeTrajectory();
    void makePathToHome(gmtl::Vec3d startPos);
    int count;


    void generateTrajectory();
private:

};

#endif // MAKETRAJECTORY_H
