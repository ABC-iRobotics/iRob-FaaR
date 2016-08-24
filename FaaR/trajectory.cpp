#include "trajectory.h"

Trajectory::Trajectory()
{
}

using namespace KDL;
// Create the trajectory:
// use try/catch to catch any exceptions thrown.
// NOTE:  exceptions will become obsolete in a future version.
try {
    // Path_RoundedComposite defines the geometric path along
    // which the robot will move.
    //
    Path_RoundedComposite* path = new Path_RoundedComposite(0.2,0.01,new RotationalInterpolation_SingleAxis());
    // The routines are now robust against segments that are parallel.
    // When the routines are parallel, no rounding is needed, and no attempt is made
    // add constructing a rounding arc.
    // (It is still not possible when the segments are on top of each other)
    // Note that you can only rotate in a deterministic way over an angle less then M_PI!
    // With an angle == M_PI, you cannot predict over which side will be rotated.
    // With an angle > M_PI, the routine will rotate over 2*M_PI-angle.
    // If you need to rotate over a larger angle, you need to introduce intermediate points.
    // So, there is a common use case for using parallel segments.
    path->Add(Frame(Rotation::RPY(0,0,0), Vector(-0.020,0.030,0.100)));
    path->Add(Frame(Rotation::RPY(0,0,0), Vector(0.040,-0.015,0.140)));

    // always call Finish() at the end, otherwise the last segment will not be added.
    path->Finish();

    // Trajectory defines a motion of the robot along a path.
    // This defines a trapezoidal velocity profile.
    VelocityProfile* velpref = new VelocityProfile_Trap(0.05,0.08);
    velpref->SetProfile(0,path->PathLength());
    Trajectory* traject = new Trajectory_Segment(path, velpref);


    Trajectory_Composite* ctraject = new Trajectory_Composite();
    ctraject->Add(traject);
    ctraject->Add(new Trajectory_Stationary(1.0,Frame(Rotation::RPY(0,0,0), Vector(1,1,0))));



    // use the trajectory
    double dt=0.01;
    std::ofstream of("./trajectory.dat");
    for (double t=0; t <= traject->Duration(); t+= dt)
    {
        of << t << "\t";
        Frame current_pose;
        Twist current_vel;
        Twist current_acc;
        current_pose = ctraject->Pos(t);
        current_vel = ctraject->Vel(t);
        current_acc = ctraject->Acc(t);
        for (int i=0;i<3;++i)
            for (int j=3;j<4;++j)
                of << current_pose(i,j)  <<"\t" << current_vel(i) << "\t" << current_acc(i) << "\t";
        of << "\n";
        // also velocities and accelerations are available !
        //traject->Vel(t);
        //traject->Acc(t);
    }
    of.close();

    // you can get some meta-info on the path:
    for (int segmentnr=0;  segmentnr < path->GetNrOfSegments(); segmentnr++) {
        double starts,ends;
        Path::IdentifierType pathtype;
        if (segmentnr==0) {
            starts = 0.0;
        } else {
            starts = path->GetLengthToEndOfSegment(segmentnr-1);
        }
        ends = path->GetLengthToEndOfSegment(segmentnr);
        pathtype = path->GetSegment(segmentnr)->getIdentifier();
        std::cout << "segment " << segmentnr << " runs from s="<<starts << " to s=" <<ends;
        switch(pathtype) {
            case Path::ID_CIRCLE:
                std::cout << " circle";
                break;
            case Path::ID_LINE:
                std::cout << " line ";
                break;
            default:
                std::cout << " unknown ";
                break;
        }
        std::cout << std::endl;
    }
    std::cout << " trajectory written to the ./trajectory.dat file " << std::endl;

    delete ctraject;
} catch(Error& error) {
    std::cout <<"I encountered this error : " << error.Description() << std::endl;
    std::cout << "with the following type " << error.GetType() << std::endl;
}
