#include "falcon/trajectoryGeneration/maketrajectory.h"

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
                            Vector(0.03*sin(3.14/180*i),-0.06,0.120+0.03*cos(3.14/180*i))));
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

        /*for(int i = 0; i < genPoints; i++)
        {   //qDebug() << genVectX[i] << "    " << genVectY[i] << "   " << genVectZ[i] << endl;
            genPath->Add(KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(genVectX[i], genVectY[i], genVectZ[i])));
        }*/

        ///cross lines and circle test
        /*
        genPath->Add(Frame(Rotation::Identity(),Vector(0,0,0.120)));
        genPath->Add(Frame(Rotation::Identity(),Vector(0,-0.0299,0.09)));
        genPath->Add(Frame(Rotation::Identity(),Vector(0,-0.0299,0.16)));
        genPath->Add(Frame(Rotation::Identity(),Vector(0,0,0.120)));
        genPath->Add(Frame(Rotation::Identity(),Vector(0.04,0,0.120)));
        genPath->Add(Frame(Rotation::Identity(),Vector(0.04,-0.0299,0.120)));
        genPath->Add(Frame(Rotation::Identity(),Vector(-0.04,-0.0299,0.120)));
        genPath->Add(Frame(Rotation::Identity(),Vector(-0.04,0,0.120)));
        ///Circle
        for(int i = 0; i < 72; i++)
        {
            genPath->Add(Frame(Rotation::Identity(),Vector(0.02*sin(3.14/10*i),-0.0299,0.120+0.02*cos(3.14/10*i))));
        }*/
        ///

        // always call Finish() at the end, otherwise the last segment will not be added.
        genPath->Finish();

        // Trajectory defines a motion of the robot along a path.
        // This defines a trapezoidal velocity profile.
        VelocityProfile* velpref = new VelocityProfile_Trap(0.07,0.009);
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
        ///check that the log directory is existing
        QDir dir(".");
        QDir logPath("./Logs/generated");
        if(!logPath.exists())
        {
            dir.mkpath("./Logs/generated");
        }
        ///
        time = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh:mm:ss"); // time for filename
        QFile myFile("./Logs/generated/" + time + ".dat"); // create log file with current date name
        myFile.open(QFile::WriteOnly | QFile::ReadOnly | QFile::Text); // open log file
        if(!myFile.isOpen())
        {
            std::cout << "File is not opened" << std::endl;
        }
        QTextStream out(&myFile); // text object is created to write data to the files
        genCount = 0;
        for (double t=0; t <= traject->Duration(); t+= dt)
        {
            szam = traject->Duration();
            Frame current_pose;
            Twist current_vel;
            Twist current_acc;
            current_pose = traject->Pos(t);
            current_vel = traject->Vel(t);
            current_acc = traject->Acc(t);
            for (int i=0;i<3;++i)
                for (int j=3;j<4;++j)
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
            pos[0] = current_pose.p.x();
            pos[1] = current_pose.p.y();
            pos[2] = current_pose.p.z();
            IK(angles,pos);
            out << angles.theta1[0] << " " << angles.theta1[1] <<" "<< angles.theta1[2] <<endl;
            genCount++;
        }
        myFile.flush();
        myFile.close(); // close opened log file.



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

    genPath = new KDL::Path_RoundedComposite(0.0001 ,0.0001, new KDL::RotationalInterpolation_SingleAxis());

}

void MakeTrajectory::IK(Angle& angles,const gmtl::Vec3d& worldPosition)
{
    //////////////////////////////////////////////////////////
    //Inverse kinematics. All as in Stamper's PhD except for
    //the addition of a second offset direction 's' per arm
    //First we need the offset vector from the origin of the XYZ coordinate frame to the
    //UVW coordinate frame:
    gmtl::Vec3d offset(-libnifalcon::r,-libnifalcon::s,0);


    //Next lets convert the current end effector position into the UVW coordinates
    //of each leg:
    gmtl::Matrix33d R;
    R(0,0)=cos(libnifalcon::phy[0]);	R(0,1)=sin(libnifalcon::phy[0]);	R(0,2)=0;
    R(1,0)=-sin(libnifalcon::phy[0]);	R(1,1)=cos(libnifalcon::phy[0]);	R(1,2)=0;
    R(2,0)=0;							R(2,1)=0;							R(2,2)=1;
    gmtl::Vec3d P1 = R*worldPosition + offset;

    R(0,0)=cos(libnifalcon::phy[1]);	R(0,1)=sin(libnifalcon::phy[1]);	R(0,2)=0;
    R(1,0)=-sin(libnifalcon::phy[1]);	R(1,1)=cos(libnifalcon::phy[1]);	R(1,2)=0;
    R(2,0)=0;							R(2,1)=0;							R(2,2)=1;
    gmtl::Vec3d P2 = R*worldPosition + offset;

    R(0,0)=cos(libnifalcon::phy[2]);	R(0,1)=sin(libnifalcon::phy[2]);	R(0,2)=0;
    R(1,0)=-sin(libnifalcon::phy[2]);	R(1,1)=cos(libnifalcon::phy[2]);	R(1,2)=0;
    R(2,0)=0;							R(2,1)=0;							R(2,2)=1;
    gmtl::Vec3d P3 = R*worldPosition + offset;


    //Do the theta3's first. This is +/- but fortunately in the Falcon's case
    //only the + result is correct
    angles.theta3[0] = acos( (P1[1]+libnifalcon::f)/libnifalcon::b);
    angles.theta3[1] = acos( (P2[1]+libnifalcon::f)/libnifalcon::b);
    angles.theta3[2] = acos( (P3[1]+libnifalcon::f)/libnifalcon::b);


    //Next find the theta1's
    //In certain cases could query the theta1 values directly and save a bit of processing
    //Again we have a +/- situation but only + is relevent
    double l01 = P1[2]*P1[2] + P1[0]*P1[0] + 2*libnifalcon::c*P1[0] - 2*libnifalcon::a*P1[0] + libnifalcon::a*libnifalcon::a + libnifalcon::c*libnifalcon::c - libnifalcon::d*libnifalcon::d - libnifalcon::e*libnifalcon::e - libnifalcon::b*libnifalcon::b*sin(angles.theta3[0])*sin(angles.theta3[0]) - 2*libnifalcon::b*libnifalcon::e*sin(angles.theta3[0]) - 2*libnifalcon::b*libnifalcon::d*sin(angles.theta3[0]) - 2*libnifalcon::d*libnifalcon::e - 2*libnifalcon::a*libnifalcon::c;
    double l11 = -4*libnifalcon::a*P1[2];
    double l21 = P1[2]*P1[2] + P1[0]*P1[0] + 2*libnifalcon::c*P1[0] + 2*libnifalcon::a*P1[0] + libnifalcon::a*libnifalcon::a + libnifalcon::c*libnifalcon::c - libnifalcon::d*libnifalcon::d - libnifalcon::e*libnifalcon::e - libnifalcon::b*libnifalcon::b*sin(angles.theta3[0])*sin(angles.theta3[0]) - 2*libnifalcon::b*libnifalcon::e*sin(angles.theta3[0]) - 2*libnifalcon::b*libnifalcon::d*sin(angles.theta3[0]) - 2*libnifalcon::d*libnifalcon::e + 2*libnifalcon::a*libnifalcon::c;

    double l02 = P2[2]*P2[2] + P2[0]*P2[0] + 2*libnifalcon::c*P2[0] - 2*libnifalcon::a*P2[0] + libnifalcon::a*libnifalcon::a + libnifalcon::c*libnifalcon::c - libnifalcon::d*libnifalcon::d - libnifalcon::e*libnifalcon::e - libnifalcon::b*libnifalcon::b*sin(angles.theta3[1])*sin(angles.theta3[1]) - 2*libnifalcon::b*libnifalcon::e*sin(angles.theta3[1]) - 2*libnifalcon::b*libnifalcon::d*sin(angles.theta3[1]) - 2*libnifalcon::d*libnifalcon::e - 2*libnifalcon::a*libnifalcon::c;
    double l12 = -4*libnifalcon::a*P2[2];
    double l22 = P2[2]*P2[2] + P2[0]*P2[0] + 2*libnifalcon::c*P2[0] + 2*libnifalcon::a*P2[0] + libnifalcon::a*libnifalcon::a + libnifalcon::c*libnifalcon::c - libnifalcon::d*libnifalcon::d - libnifalcon::e*libnifalcon::e - libnifalcon::b*libnifalcon::b*sin(angles.theta3[1])*sin(angles.theta3[1]) - 2*libnifalcon::b*libnifalcon::e*sin(angles.theta3[1]) - 2*libnifalcon::b*libnifalcon::d*sin(angles.theta3[1]) - 2*libnifalcon::d*libnifalcon::e + 2*libnifalcon::a*libnifalcon::c;

    double l03 = P3[2]*P3[2] + P3[0]*P3[0] + 2*libnifalcon::c*P3[0] - 2*libnifalcon::a*P3[0] + libnifalcon::a*libnifalcon::a + libnifalcon::c*libnifalcon::c - libnifalcon::d*libnifalcon::d - libnifalcon::e*libnifalcon::e - libnifalcon::b*libnifalcon::b*sin(angles.theta3[2])*sin(angles.theta3[2]) - 2*libnifalcon::b*libnifalcon::e*sin(angles.theta3[2]) - 2*libnifalcon::b*libnifalcon::d*sin(angles.theta3[2]) - 2*libnifalcon::d*libnifalcon::e - 2*libnifalcon::a*libnifalcon::c;
    double l13 = -4*libnifalcon::a*P3[2];
    double l23 = P3[2]*P3[2] + P3[0]*P3[0] + 2*libnifalcon::c*P3[0] + 2*libnifalcon::a*P3[0] + libnifalcon::a*libnifalcon::a + libnifalcon::c*libnifalcon::c - libnifalcon::d*libnifalcon::d - libnifalcon::e*libnifalcon::e - libnifalcon::b*libnifalcon::b*sin(angles.theta3[2])*sin(angles.theta3[2]) - 2*libnifalcon::b*libnifalcon::e*sin(angles.theta3[2]) - 2*libnifalcon::b*libnifalcon::d*sin(angles.theta3[2]) - 2*libnifalcon::d*libnifalcon::e + 2*libnifalcon::a*libnifalcon::c;


    double T1a = (-l11 + sqrt( l11*l11 - 4* l01* l21) ) / (2*l21);
    double T2a = (-l12 + sqrt( l12*l12 - 4* l02* l22) ) / (2*l22);
    double T3a = (-l13 + sqrt( l13*l13 - 4* l03* l23) ) / (2*l23);

    double T1b = (-l11 - sqrt( l11*l11 - 4* l01* l21) ) / (2*l21);
    double T2b = (-l12 - sqrt( l12*l12 - 4* l02* l22) ) / (2*l22);
    double T3b = (-l13 - sqrt( l13*l13 - 4* l03* l23) ) / (2*l23);

    angles.theta1[0] = atan(T1b)*2;
    angles.theta1[1] = atan(T2b)*2;
    angles.theta1[2] = atan(T3b)*2;
    //std::cout << angles.theta1[0] << "   " << angles.theta1[1] << "  " << angles.theta1[2] << std::endl;

    //And finally calculate the theta2 values:
    angles.theta2[0] = acos( (-P1[0] + libnifalcon::a*cos(angles.theta1[0]) - libnifalcon::c)/(-libnifalcon::d - libnifalcon::e - libnifalcon::b*sin(angles.theta3[0]) )  );
    angles.theta2[1] = acos( (-P2[0] + libnifalcon::a*cos(angles.theta1[1]) - libnifalcon::c)/(-libnifalcon::d - libnifalcon::e - libnifalcon::b*sin(angles.theta3[1]) )  );
    angles.theta2[2] = acos( (-P3[0] + libnifalcon::a*cos(angles.theta1[2]) - libnifalcon::c)/(-libnifalcon::d - libnifalcon::e - libnifalcon::b*sin(angles.theta3[2]) )  );

  // setDesAng(angles,importantAng); // save important angles from "angles" and return them to "importantAng"

}
