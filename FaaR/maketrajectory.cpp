#include "maketrajectory.h"

MakeTrajectory::MakeTrajectory()
{

}
        std::ofstream genOf;
void MakeTrajectory::makeTrajectory()
{
    using namespace KDL;
    // Create the trajectory:
    // use try/catch to catch any exceptions thrown.
    // NOTE:  exceptions will become obsolete in a future version.
    try {
        // Path_RoundedComposite defines the geometric path along
        // which the robot will move.
        //
        Path_RoundedComposite* path = new Path_RoundedComposite(0.005,0.001,new RotationalInterpolation_SingleAxis());
        // The routines are now robust against segments that are parallel.
        // When the routines are parallel, no rounding is needed, and no attempt is made
        // add constructing a rounding arc.
        // (It is still not possible when the segments are on top of each other)
        // Note that you can only rotate in a deterministic way over an angle less then M_PI!
        // With an angle == M_PI, you cannot predict over which side will be rotated.
        // With an angle > M_PI, the routine will rotate over 2*M_PI-angle.
        // If you need to rotate over a larger angle, you need to introduce intermediate points.
        // So, there is a common use case for using parallel segments.

        ///4 point
        /*path->Add(Frame(Rotation::Identity(), Vector(0,0,0.110)));
        path->Add(Frame(Rotation::Identity(), Vector(0.0,0.0,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(0.03,0.0,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(0.0,0.03,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(-0.03,0.0,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(0.0,-0.03,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(0.03,0.0,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(0.0,0.03,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(-0.03,0.0,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(0.0,-0.03,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(0.03,0.0,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(0.0,0.03,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(-0.03,0.0,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(0.0,-0.03,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(0.03,0.0,0.120)));
        path->Add(Frame(Rotation::Identity(), Vector(0.0,0.0,0.120)));*/


        ///Circle
        //path->Add(Frame(Rotation::Identity(), Vector(0,0,0.110)));
        for(int i = 0; i < 1440; i++)
        {
            path->Add(Frame(Rotation::Identity(),
                            Vector(0.04*sin(3.14/180*i),0.04*cos(3.14/180*i),0.120)));
        }



        // always call Finish() at the end, otherwise the last segment will not be added.
        path->Finish();

        // Trajectory defines a motion of the robot along a path.
        // This defines a trapezoidal velocity profile.
        VelocityProfile* velpref = new VelocityProfile_Trap(0.1,0.07);
        velpref->SetProfile(0,path->PathLength());
        Trajectory* traject = new Trajectory_Segment(path, velpref);


        Trajectory_Composite* ctraject = new Trajectory_Composite();
        ctraject->Add(traject);
        //ctraject->Add(new Trajectory_Stationary(1.0,Frame(Rotation::RPY(0,0,0), Vector(1,1,0))));
        //ctraject->Add(new Trajectory_Stationary(1.0,Frame(Rotation::RPY(0,0,0), Vector(0.040,-0.015,0.140))));
        //ctraject->Add(new Trajectory_Stationary(1.0,Frame(Rotation::RPY(0,0,0), Vector(-0.020,0.030,0.110))));


        // use the trajectory
        double dt=0.001;
        std::ofstream of("./trajectory.dat");
        count = 0;
        for (double t=0; t <= traject->Duration(); t+= dt)
        {
            of << count << "    ";
            of << t << "\t";
            Frame current_pose;
            Twist current_vel;
            Twist current_acc;
            current_pose = traject->Pos(t);
            current_vel = traject->Vel(t);
            current_acc = traject->Acc(t);
            for (int i=0;i<3;++i)
                for (int j=3;j<4;++j)
                    of << current_pose(i,j)  <<"\t" << current_vel(i) << "\t" << current_acc(i) << "\t";
            of << "\n";
            // also velocities and accelerations are available !
            //traject->Vel(t);
            //traject->Acc(t);
            x.push_back(current_pose.p.x());
            y.push_back(current_pose.p.y());
            z.push_back(current_pose.p.z());
            vx.push_back(current_vel.vel[0]);
            vy.push_back(current_vel.vel[1]);
            vz.push_back(current_vel.vel[2]);
            ax.push_back(current_acc.vel[0]);
            ay.push_back(current_acc.vel[1]);
            az.push_back(current_acc.vel[2]);
            n.push_back(count);
            //std::cout << "current pose x: " << current_acc.vel[2] << std::endl;
            count++;
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
            //std::cout << "segment " << segmentnr << " runs from s="<<starts << " to s=" <<ends;
            switch(pathtype) {
                case Path::ID_CIRCLE:
                    //std::cout << " circle";
                    break;
                case Path::ID_LINE:
                    //std::cout << " line ";
                    break;
                case Path::ID_COMPOSITE:
                    //std::cout << " composite";
                    break;
                case Path::ID_ROUNDED_COMPOSITE:
                    //std::cout << " rounded composite";
                    break;
                case Path::ID_POINT:
                    //std::cout << " point";
                    break;
                case Path::ID_CYCLIC_CLOSED:
                    //std::cout << " cyclic closed";
                    break;
                default:
                    std::cout << " unknown ";
                    break;
            }
            std::cout << std::endl;
        }
        //std::cout << " trajectory written to the ./trajectory.dat file " << std::endl;

        delete ctraject;
    } catch(Error& error) {
        std::cout <<"I encountered this error : " << error.Description() << std::endl;
        std::cout << "with the following type " << error.GetType() << std::endl;
    }
}
void MakeTrajectory::makePathToHome(gmtl::Vec3d startPos)
{
    using namespace KDL;
    // Create the trajectory:
    // use try/catch to catch any exceptions thrown.
    // NOTE:  exceptions will become obsolete in a future version.
    try {

        Path_RoundedComposite* path = new Path_RoundedComposite(0.005,0.001,new RotationalInterpolation_SingleAxis());

        /// Jelenlegi pozició -> home pozició
        path->Add(Frame(Rotation::Identity(), Vector(startPos[0],startPos[1],startPos[2])));
        path->Add(Frame(Rotation::Identity(), Vector(0,0,0.110)));

        // always call Finish() at the end, otherwise the last segment will not be added.
        path->Finish();

        // Trajectory defines a motion of the robot along a path.
        // This defines a trapezoidal velocity profile.
        VelocityProfile* velpref = new VelocityProfile_Trap(0.1,0.07);
        velpref->SetProfile(0,path->PathLength());
        Trajectory* traject = new Trajectory_Segment(path, velpref);


        Trajectory_Composite* ctraject = new Trajectory_Composite();
        ctraject->Add(traject);

        // use the trajectory
        double dt=0.001;
        std::ofstream of("./trajectory.dat");
        count = 0;
        for (double t=0; t <= traject->Duration(); t+= dt)
        {
            of << count << "    ";
            of << t << "\t";
            Frame current_pose;
            Twist current_vel;
            Twist current_acc;
            current_pose = traject->Pos(t);
            current_vel = traject->Vel(t);
            current_acc = traject->Acc(t);
            for (int i=0;i<3;++i)
                for (int j=3;j<4;++j)
                    of << current_pose(i,j)  <<"\t" << current_vel(i) << "\t" << current_acc(i) << "\t";
            of << "\n";
            // also velocities and accelerations are available !
            //traject->Vel(t);
            //traject->Acc(t);
            x.push_back(current_pose.p.x());
            y.push_back(current_pose.p.y());
            z.push_back(current_pose.p.z());
            vx.push_back(current_vel.vel[0]);
            vy.push_back(current_vel.vel[1]);
            vz.push_back(current_vel.vel[2]);
            ax.push_back(current_acc.vel[0]);
            ay.push_back(current_acc.vel[1]);
            az.push_back(current_acc.vel[2]);
            n.push_back(count);
            //std::cout << "current pose x: " << current_acc.vel[2] << std::endl;
            count++;
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
            //std::cout << "segment " << segmentnr << " runs from s="<<starts << " to s=" <<ends;
            switch(pathtype) {
                case Path::ID_CIRCLE:
                    //std::cout << " circle";
                    break;
                case Path::ID_LINE:
                    //std::cout << " line ";
                    break;
                case Path::ID_COMPOSITE:
                    //std::cout << " composite";
                    break;
                case Path::ID_ROUNDED_COMPOSITE:
                    //std::cout << " rounded composite";
                    break;
                case Path::ID_POINT:
                    //std::cout << " point";
                    break;
                case Path::ID_CYCLIC_CLOSED:
                    //std::cout << " cyclic closed";
                    break;
                default:
                    std::cout << " unknown ";
                    break;
            }
            std::cout << std::endl;
        }
        //std::cout << " trajectory written to the ./trajectory.dat file " << std::endl;

        delete ctraject;
    } catch(Error& error) {
        std::cout <<"I encountered this error : " << error.Description() << std::endl;
        std::cout << "with the following type " << error.GetType() << std::endl;
    }
}






void MakeTrajectory::addPath(double x, double y, double z)
{
    genPath->Add(KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(x, y, z)));
}

void MakeTrajectory::generateTrajectory()
{
    using namespace KDL;
    // Create the trajectory:
    // use try/catch to catch any exceptions thrown.
    // NOTE:  exceptions will become obsolete in a future version.
    try {
        // Path_RoundedComposite defines the geometric path along
        // which the robot will move.
        //
        //Path_RoundedComposite* genPath = new Path_RoundedComposite(0.005,0.001,new RotationalInterpolation_SingleAxis());
        // The routines are now robust against segments that are parallel.
        // When the routines are parallel, no rounding is needed, and no attempt is made
        // add constructing a rounding arc.
        // (It is still not possible when the segments are on top of each other)
        // Note that you can only rotate in a deterministic way over an angle less then M_PI!
        // With an angle == M_PI, you cannot predict over which side will be rotated.
        // With an angle > M_PI, the routine will rotate over 2*M_PI-angle.
        // If you need to rotate over a larger angle, you need to introduce intermediate points.
        // So, there is a common use case for using parallel segments.
         //genPath->Add(Frame(Rotation::Identity(), Vector(0.02, 0.03, 0.130)));
         //genPath->Add(Frame(Rotation::Identity(), Vector(-0.03, -0.03, 0.110)));

        for(int i = 0; i < genPoints; i++)
        {   //qDebug() << genVectX[i] << "    " << genVectY[i] << "   " << genVectZ[i] << endl;
            genPath->Add(KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(genVectX[i], genVectY[i], genVectZ[i])));
        }


        // always call Finish() at the end, otherwise the last segment will not be added.
        genPath->Finish();

        // Trajectory defines a motion of the robot along a path.
        // This defines a trapezoidal velocity profile.
        VelocityProfile* velpref = new VelocityProfile_Trap(0.2,0.02);
        std::cout << "Path length: " << genPath->PathLength() << std::endl;
        velpref->SetProfile(0,genPath->PathLength());
        Trajectory* traject = new Trajectory_Segment(genPath, velpref);


        Trajectory_Composite* ctraject = new Trajectory_Composite();
        ctraject->Add(traject);
        //ctraject->Add(new Trajectory_Stationary(1.0,Frame(Rotation::RPY(0,0,0), Vector(1,1,0))));
        //ctraject->Add(new Trajectory_Stationary(1.0,Frame(Rotation::RPY(0,0,0), Vector(0.040,-0.015,0.140))));
        //ctraject->Add(new Trajectory_Stationary(1.0,Frame(Rotation::RPY(0,0,0), Vector(-0.020,0.030,0.110))));


        // use the trajectory
        double dt=0.001;
        //std::ofstream genOf("./gentrajectory.dat");
        genOf.open("./gentrajectory.dat", std::ios_base::trunc);
        genCount = 0;
        for (double t=0; t <= traject->Duration(); t+= dt)
        {
            szam = traject->Duration();
            genOf << genCount << "    ";
            genOf << t << "\t";
            Frame current_pose;
            Twist current_vel;
            Twist current_acc;
            current_pose = traject->Pos(t);
            current_vel = traject->Vel(t);
            current_acc = traject->Acc(t);
            for (int i=0;i<3;++i)
                for (int j=3;j<4;++j)
                    genOf << current_pose(i,j) << /* <<"\t" << current_vel(i) << "\t" << current_acc(i) << */"\t";
            genOf << "\n";
            // also velocities and accelerations are available !
            //traject->Vel(t);
            //traject->Acc(t);
            genX.push_back(current_pose.p.x());
            genY.push_back(current_pose.p.y());
            genZ.push_back(current_pose.p.z());
            genVx.push_back(current_vel.vel[0]);
            genVy.push_back(current_vel.vel[1]);
            genVz.push_back(current_vel.vel[2]);
            genAx.push_back(current_acc.vel[0]);
            genAy.push_back(current_acc.vel[1]);
            genAz.push_back(current_acc.vel[2]);
            genN.push_back(genCount);
            //std::cout << "current pose x: " << current_acc.vel[2] << std::endl;
            genCount++;
        }
        genOf.close();



        // you can get some meta-info on the path:
        for (int segmentnr=0;  segmentnr < genPath->GetNrOfSegments(); segmentnr++) {
            double starts,ends;
            Path::IdentifierType pathtype;
            if (segmentnr==0) {
                starts = 0.0;
            } else {
                starts = genPath->GetLengthToEndOfSegment(segmentnr-1);
            }
            ends = genPath->GetLengthToEndOfSegment(segmentnr);
            pathtype = genPath->GetSegment(segmentnr)->getIdentifier();
            //std::cout << "segment " << segmentnr << " runs from s="<<starts << " to s=" <<ends;
            switch(pathtype) {
                case Path::ID_CIRCLE:
                    //std::cout << " circle";
                    break;
                case Path::ID_LINE:
                    //std::cout << " line ";
                    break;
                case Path::ID_COMPOSITE:
                    //std::cout << " composite";
                    break;
                case Path::ID_ROUNDED_COMPOSITE:
                    //std::cout << " rounded composite";
                    break;
                case Path::ID_POINT:
                    //std::cout << " point";
                    break;
                case Path::ID_CYCLIC_CLOSED:
                    //std::cout << " cyclic closed";
                    break;
                default:
                    std::cout << " unknown ";
                    break;
            }
            std::cout << std::endl;
        }
        //std::cout << " trajectory written to the ./gentrajectory.dat file " << std::endl;

        delete ctraject;
    } catch(Error& error) {
        std::cout <<"I encountered this error : " << error.Description() << std::endl;
        std::cout << "with the following type " << error.GetType() << std::endl;
    }

    std::cout << "Trajectory Generated." << std::endl;
}
