#include "controlForGui.h"
std::ofstream logging("./log.dat");

controlForGui::controlForGui()
{

}

controlForGui::controlForGui(falconData argsIn)
{
    /// constructor, passes arguments by reference from the main thread and the falcon object which was created in the
    /// real-time thread
    /// init() initialises the connection, and sets the proper starting values for the variables

    // Passing the arguments from input struct to the member variables //
    //args = argsIn;

    mPosX=argsIn.posX;
    mPosY=argsIn.posY;
    mPosZ=argsIn.posZ;
    mKp=argsIn.Kp;
    mKd=argsIn.Kd;
    lowPassIsOn=argsIn.lowPassIsOn;
    dev=*argsIn.device;
//    mPosX =&pX;
//    mPosY =&pY;
//    mPosZ =&pZ;
//    mKp = &pKp;
//    mKd = &pKd;
//    lowPassIsOn=&plowPassIsOn;
//    dev=device;

    dev.setFalconFirmware<FalconFirmwareNovintSDK>();
    init();

    currentState =goHomeMode;

}

///Control part
    /// Communication + Reading + Impedance
void controlForGui::trajectoryPath()
{   std::cout << "running" << std::endl;
    for(int i = 0; i< trajectory.count; i++)
    {
        gmtl::Vec3d placeholder;
        trajPos[0] = trajectory.x[i]; // [mm] to [m]
        trajPos[1] = trajectory.y[i]; // [mm] to [m]
        trajPos[2] = trajectory.z[i]; // [mm] to [m]
        IK(trajAng, trajPos, placeholder);
        trajTh1.push_back(trajAng.theta1[0]);
        trajTh2.push_back(trajAng.theta1[1]);
        trajTh3.push_back(trajAng.theta1[2]);
    }

}

void controlForGui::genTrajectoryPath()
{   std::cout << "Gentraj running" << std::endl;
    int loopcount = 0;
    for(int i = 0; i< trajectory.genCount; i++)
    {
        gmtl::Vec3d placeholder;
        genTrajPos[0] = trajectory.genX[i]; // [mm] to [m]
        genTrajPos[1] = trajectory.genY[i]; // [mm] to [m]
        genTrajPos[2] = trajectory.genZ[i]; // [mm] to [m]
        qDebug() << genTrajPos[0] << endl;
        IK(genTrajAng, genTrajPos, placeholder);
        genTrajTh1.push_back(genTrajAng.theta1[0]);
        genTrajTh2.push_back(genTrajAng.theta1[1]);
        genTrajTh3.push_back(genTrajAng.theta1[2]);
    }

}

/// Trajectory generation from points
void controlForGui::generateTrajectory()  // Something is wrong here -gotta look into it later
{
    gmtl::Vec3d encoderAng;         /// store encoder values
    read_encoder(encoderAng);       /// copies the current values to the encoderAng variable
    FK(encoderAng);                 /// Forward kinematics
    std::cout << pos << std::endl;
}


void controlForGui::getNextPoint(std::vector<double> ang1, std::vector<double> ang2, std::vector<double> ang3)
{

    /// In every cicle loopcount increases, and copies the proper
    /// reference values to the "wayToGo" variable from the array
    /// in which we store the full path

    wayToGo[0] = ang1[loopCount];
    wayToGo[1] = ang2[loopCount];
    wayToGo[2] = ang3[loopCount];

    loopCount++;
}

    ///PD Control stuff

void controlForGui::FalconLoop()
{

    /// This is the main control loop, operating mode is changed
    /// using switch

    /// some variables used later on the loop, must be updated in each loop
    gmtl::Vec3d encoderAng;  //  to store encoder values
    gmtl::Vec3d refAng;      //  to store the setpoint encoder values
    Angle refPosAng;         //  to store IK() function output
    gmtl::Vec3d desiredOutput, posWithImpedance;

    switch (currentState)
    {
    case goHomeMode:
    {

        runIOLoop();                        ///  Exchange data with the falcon
        read_encoder(encoderAng);           ///  Copy encoder values to "encoderAng"
        IK(refPosAng,thisIsHome,refAng);    ///  Calculate the angles for the setpoint position and copy the proper values to "refAng"
        interPol(encoderAng,refAng);        ///  Simple linear path iteration, first call-> calculates path, and push proper values to "wayToGo"
        PID(wayToGo,encoderAng);            ///  PID controller refAng -> setpoint value, encoderAng -> current value
        setLedGreen();                      ///  Set the LED on the falcon green
        FK(encoderAng);                     ///  Forward kinematics


            // the Falcon should reach the home position as the loopCount
            // reaches 1000, switch mode to stay at the home position and
            // resets loopCount
        if (loopCount == 1000)
        {
            isAtHome=true;
            std::cout<< "[Home position reached] " << std::endl;
            loopCount = 0;
            endPos = pos;
            runIOLoop();
            read_encoder(homeAng);
            currentState = stayMode;

        }
        break;
    }
    case stayMode:
    {
        // In this mode, the falcon stays at the last position that was set

        setPid(*mKp,*mKd);              /// set PID values (Kd & Kp from GUI)
        runIOLoop();                    ///  Exchange data with the falcon
        read_encoder(encoderAng);       ///  Copy encoder values to "encoderAng"
        addImpedance(endPos,posWithImpedance);   /// Add Positions from impedance model ,the the new setpoint position is stored in the second argument
        IK(refPosAng,posWithImpedance,refAng);   /// Calculate the angles from the position of the updated "pos" value
        PID(refAng,encoderAng);         ///  PID controller refAng -> setpoint value, encoderAng -> current value
        setLedGreen();                  ///  Set the LED on the falcon green
        FK(encoderAng);                 ///  Forward kinematics
        break;
    }
    case followPathMode:
    {

        // In this mode the end-effector follows a
        // previously defined path in maketrajectory.cpp ( circle right now )

        setPid(*mKp,*mKd);              /// set PID values (Kd & Kp from GUI)
        runIOLoop();                    ///  Exchange data with the falcon
        read_encoder(encoderAng);       ///  Copy encoder values to "encoderAng"
        getNextPoint(trajTh1, trajTh2, trajTh3); /// acces the next setpoint from the array that stores the path
        FK(wayToGo,desiredOutput);      /// Forward kinematics , it's output is "desiredOutput"
        addImpedance(desiredOutput,posWithImpedance); /// Add Positions from impedance model ,the the new setpoint position is stored in the second argument
        IK(refPosAng,posWithImpedance,wayToGo);       /// Calculate the angles from the position of the updated setPoint value (here: posWithImpedance)
        PID(wayToGo,encoderAng);        ///  PID controller refAng -> setpoint value, encoderAng -> current value
        setLedGreen();                  ///  Set the LED on the falcon green
        FK(encoderAng);                 /// Forward kinematics , it's output is the global "pos"

        // ?? //
        posx.push_back(returnpos()[0]);
        posy.push_back(returnpos()[1]);
        posz.push_back(returnpos()[2]);


            // at the end of path switch back to homing mode
        if (loopCount == trajectory.count)
        {
            endPos = homeAng;
            isAtHome=true;
            std::cout<< "Arrived!" << std::endl;
            loopCount = 0;
            currentState=goHomeMode;  /// state swtich -> go to home position
            firstRun=true;
        }
        break;
    }
    case genTrajMode:
    {

        // In this mode the end-effector follows a
        // previously generated path

        setPid(*mKp,*mKd);              /// set PID values (Kd & Kp from GUI)
        runIOLoop();                    ///  Exchange data with the falcon
        read_encoder(encoderAng);       ///  Copy encoder values to "encoderAng"
        getNextPoint(genTrajTh1, genTrajTh2, genTrajTh3); /// acces the next setpoint from the array that stores the path
        FK(wayToGo,desiredOutput);       /// Forward kinematics , it's output is "desiredOutput"
        addImpedance(desiredOutput,posWithImpedance); /// Add Positions from impedance model ,the the new setpoint position is stored in the second argument
        IK(refPosAng,posWithImpedance,wayToGo);       /// Calculate the angles from the position of the updated setPoint value (here: posWithImpedance)
        PID(wayToGo,encoderAng);        ///  PID controller refAng -> setpoint value, encoderAng -> current value
        setLedGreen();                  ///  Set the LED on the falcon green
        FK(encoderAng);                 /// Forward kinematics , it's output is the global "pos"

        //?//
        posx.push_back(returnpos()[0]);
        posy.push_back(returnpos()[1]);
        posz.push_back(returnpos()[2]);

            // at the end of path switch back to homing mode
        if (loopCount == trajectory.genCount)
        {
            endPos = homeAng;
            isAtHome=true;
            std::cout<< "Odaértem! " << std::endl;
            loopCount = 0;
            currentState=goHomeMode;  /// state swtich -> go to home position
            firstRun=true;
        }
        break;
    }
    case logPathMode:
    {
        // only communication occours , so the falcon does not crash
        // only for logging data, no forces transmitted
        runIOLoop();              ///  Exchange data with the falcon
        read_encoder(encoderAng); ///  Copy encoder values to "encoderAng"
        break;
    }

    case replayMode:
    {
        // In this mode the end-effector follows a
        // previously logged path

        setPid(*mKp,*mKd);              /// set PID values (Kd & Kp from GUI)
        runIOLoop();                    ///  Exchange data with the falcon
        read_encoder(encoderAng);       ///  Copy encoder values to "encoderAng"
        getNextPoint(logTh0, logTh1, logTh2); /// acces the next setpoint from the array that stores the path
        FK(wayToGo,desiredOutput);       /// Forward kinematics , it's output is "desiredOutput"
        addImpedance(desiredOutput,posWithImpedance); /// Add Positions from impedance model ,the the new setpoint position is stored in the second argument
        IK(refPosAng,posWithImpedance,wayToGo);       /// Calculate the angles from the position of the updated setPoint value (here: posWithImpedance)
        PID(wayToGo,encoderAng);        ///  PID controller refAng -> setpoint value, encoderAng -> current value
        setLedGreen();                  ///  Set the LED on the falcon green
        FK(encoderAng);


        if (loopCount == replayCount)
        {
            endPos = pos;
            isAtHome=true;
            std::cout<< "[replay] End of path  " << std::endl;
            loopCount = 0;
            currentState=goHomeMode;  /// state swtich -> go to home position
            firstRun=true;
        }
        break;
    }

    }

    encoderAngles=encoderAng;  //copy to global
    desAng=refAng;             //copy to global
}
void controlForGui::PID(gmtl::Vec3d desired,gmtl::Vec3d encoderAngle)
{
   /// PID implementation  ( without "I" )

   // P member //
gmtl::Vec3d errorVect;
errorVect = desired-encoderAngle;
mPVect = errorVect*Kp;
    // D member //
gmtl::Vec3d posChange = errorVect - prevErrorVect;
lowPassStep(posChange);
if (*lowPassIsOn == true)
{
    mDVect = posChange* Kd / dt;
}
if (*lowPassIsOn == false)
{
    mDVect = Kdfilter* Kd /dt;
}

output = mDVect+mPVect;     /// sum members
prevErrorVect = errorVect;  /// remember previous error

sendTorque();     /// Convert output signal to torque, and write to falcon

}
void controlForGui::setPid(double nKp,double nKd)
{
    /// Changes the control values for the PID
    Kp  = nKp;
    Kd  = nKd;

}
void controlForGui::limitTorque()
{
    /// Limiting the max torque, and scaling down the others if neccessary
    double maxTorque=5.0;	//Rather random choice here, could be higher
    double largestTorqueValue=0.0;
    int largestTorqueAxis=-1;
    for(int i=0; i<3; i++)
    {
        if(abs(torque[i])>largestTorqueValue)
        {
            largestTorqueValue=abs(torque[i]);
            largestTorqueAxis=i;
        }
    }
    //If axis with the largest torque is over the limit, scale them all to
    //bring it back to the limit:
    if(largestTorqueValue>maxTorque)
    {
        double scale = largestTorqueValue/maxTorque;
        torque /= scale;
    }
}
void controlForGui::sendTorque()
{
    /// scales the output to torque and send it to the Falcon
    torque = output;
    limitTorque();
    torque *= 10000.0;
    boost::array<int, 3> enc_vec;
    enc_vec[0] = -torque[0];
    enc_vec[1] = -torque[1];
    enc_vec[2] = -torque[2];
    f->setForces(enc_vec);
}

/// Trajektória tervezés

void controlForGui::interPol(gmtl::Vec3d encoderAng, gmtl::Vec3d& goalAngle)
{
    if (firstRun==true)  // on first run slice path to 1000 equal part, and generate a vector
    {
    startEncoder = encoderAng;
    errorVectHome = goalAngle-encoderAng;
    lamda = errorVectHome / 1000.0;
    firstRun=false;
    }
                        // sets the setpoint vector to the proper value
    firstRun=false;
    wayToGo = startEncoder + (lamda * loopCount);
    loopCount++;

 }
void controlForGui::resetFirstRun()
{

    /// reseting some variables here
    firstRun = true;
    loopCount = 0;
    isAtHome = false;

}
/// Low-pass filter
void controlForGui::avgOfArray()
{
    /// Calculates the average of the the array
    for (int j = 0; j<3 ; j++)
    {
        SummKd[j] = 0;
        for (int i = 0; i< 10 ; i++)
        {
            SummKd[j] += arr[j][i];
        }
        AVG[j] = SummKd[j]/10;
        Kdfilter[j] = AVG[j];
      //  std::cout<< j << " :" << AVG[j] << std::endl;
    }
  //  std::cout << " ---------------------" << std::endl;
}
void controlForGui::lowPassStep(gmtl::Vec3d KD)
{
    /// deletes last element in an array, pushes the others back by 1 place, read new value to the 0. cell
     for (int j = 0 ; j<3 ; j++)
   {
    for (int i = 9; i > 0 ; i--)
    {
        arr[j][i] = arr[j][i-1];

    }
    arr[j][0]=KD[j];
   }
 avgOfArray();

}
/// Retrun functions for logging data
gmtl::Vec3d controlForGui::returnDesiredAng()
{
    return desAng;
}
gmtl::Vec3d controlForGui::returnEncoderAngles()
{
    return encoderAngles;
}
gmtl::Vec3d controlForGui::returnKp()
{
    return mPVect;
}
gmtl::Vec3d controlForGui::returnKd()
{
    return mDVect;
}
gmtl::Vec3d controlForGui::returnoutput()
{
    return mPVect+mDVect;
}
gmtl::Vec3d controlForGui::returnpos()
{
    return pos;
}

void controlForGui::IK(Angle& angles,const gmtl::Vec3d& worldPosition, gmtl::Vec3d &importantAng)
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

    setDesAng(angles,importantAng); // save important angles from "angles" and return them to "importantAng"

}
void controlForGui::FK(const gmtl::Vec3d& theta0)
{
  /// Forward kinematics -> output is the global "pos" variable
    Angle angles;
    gmtl::Vec3d previousPos(pos);
    gmtl::Vec3d currentPos(pos);
    gmtl::Matrix33d J;
    gmtl::Vec3d delta;

    double targetError = 0.01;
    double previousError = 10000.0;
    double gradientAdjustment = 0.5;
    int maxTries = 15;

    bool done = 0;
    for(int i=0; i<maxTries; i++)
    {

        //All we have initially are the three values for Theta0 and a guess of position
        gmtl::Vec3d placeholder;
        //We can use the position guess to generate the angles at this position:
        IK(angles, previousPos,placeholder);
        //And these angles to find the Jacobian at the current position:
        J = jacobian(angles);
        //Then we can use the Jacobian to tell us which direction we need to move
        //in to rotate each theta0 to towards our desired values

        //Then we can see the difference between the actual and guess theta0:
        delta[0] = theta0[0]-angles.theta1[0];
        delta[1] = theta0[1]-angles.theta1[1];
        delta[2] = theta0[2]-angles.theta1[2];

        //Now use the Jacobian to tell us the direction:
        delta = J*delta;

        //And now we move along the adjustment vector
        //Nb: A good gradient descent algorithm would use more
        //intelligent step size adjustment. Here it only seems
        //to take a couple of steps to converge normally so we
        //simply start with a sensible step size and reduce it
        //if necessary to avoid oscillation about the target error.

        //Take the step size into account:
        delta*=gradientAdjustment;
        //And move the position guess:
        currentPos = previousPos + delta;

        //Let's see if we have got close enough to the target:
        //double error = sqrt(gmtl::dot(delta,delta));
        delta[0] = theta0[0]-angles.theta1[0];
        delta[1] = theta0[1]-angles.theta1[1];
        delta[2] = theta0[2]-angles.theta1[2];
        double error = dot(delta,delta);
        error = sqrt( error );
        previousPos = currentPos;

        if(error<targetError)
        {
            //Error is low enough so return the current position estimate
            pos = previousPos;
            //cout << i << endl;
            return;
        }
        //Error isn't small enough yet, see if we have over shot
        if( (error>previousError) )
        {
            //Whoops, over shot, reduce the stepsize next time:
            gradientAdjustment /= 2.0;
        }

        previousError = error;

    }

    //Failed to converge, leave last position as it was
    //cout << "Failed to find the tool position in the max tries" << endl;

}
void controlForGui::FK(const gmtl::Vec3d& theta0,gmtl::Vec3d& posOut)
{
    /// Overloaded Forward kinematics function, output is "posOut" arg
    Angle angles;
    gmtl::Vec3d previousPos(pos);
    gmtl::Vec3d currentPos(pos);
    gmtl::Matrix33d J;
    gmtl::Vec3d delta;

    double targetError = 0.01;
    double previousError = 10000.0;
    double gradientAdjustment = 0.5;
    int maxTries = 15;

    bool done = 0;
    for(int i=0; i<maxTries; i++)
    {

        //All we have initially are the three values for Theta0 and a guess of position
        gmtl::Vec3d placeholder;
        //We can use the position guess to generate the angles at this position:
        IK(angles, previousPos,placeholder);
        //And these angles to find the Jacobian at the current position:
        J = jacobian(angles);
        //Then we can use the Jacobian to tell us which direction we need to move
        //in to rotate each theta0 to towards our desired values

        //Then we can see the difference between the actual and guess theta0:
        delta[0] = theta0[0]-angles.theta1[0];
        delta[1] = theta0[1]-angles.theta1[1];
        delta[2] = theta0[2]-angles.theta1[2];

        //Now use the Jacobian to tell us the direction:
        delta = J*delta;

        //And now we move along the adjustment vector
        //Nb: A good gradient descent algorithm would use more
        //intelligent step size adjustment. Here it only seems
        //to take a couple of steps to converge normally so we
        //simply start with a sensible step size and reduce it
        //if necessary to avoid oscillation about the target error.

        //Take the step size into account:
        delta*=gradientAdjustment;
        //And move the position guess:
        currentPos = previousPos + delta;

        //Let's see if we have got close enough to the target:
        //double error = sqrt(gmtl::dot(delta,delta));
        delta[0] = theta0[0]-angles.theta1[0];
        delta[1] = theta0[1]-angles.theta1[1];
        delta[2] = theta0[2]-angles.theta1[2];
        double error = dot(delta,delta);
        error = sqrt( error );
        previousPos = currentPos;

        if(error<targetError)
        {
            //Error is low enough so return the current position estimate
            posOut = previousPos;
            //cout << i << endl;
            return;
        }
        //Error isn't small enough yet, see if we have over shot
        if( (error>previousError) )
        {
            //Whoops, over shot, reduce the stepsize next time:
            gradientAdjustment /= 2.0;
        }

        previousError = error;

    }

    //Failed to converge, leave last position as it was
    //cout << "Failed to find the tool position in the max tries" << endl;

}
gmtl::Matrix33d controlForGui::jacobian(const Angle& angles)
{

    //Naming scheme:
    //Jx1 = rotational velocity of joint 1 due to linear velocity in x

    gmtl::Matrix33d J;

    //Arm1:
    double den = -libnifalcon::a*sin(angles.theta3[0])*(sin(angles.theta1[0])*cos(angles.theta2[0])-sin(angles.theta2[0])*cos(angles.theta1[0]));

    double Jx0 = cos(phy[0])*cos(angles.theta2[0])*sin(angles.theta3[0])/den-sin(phy[0])*cos(angles.theta3[0])/den;
    double Jy0 = sin(phy[0])*cos(angles.theta2[0])*sin(angles.theta3[0])/den+cos(phy[0])*cos(angles.theta3[0])/den;
    double Jz0 = (sin(angles.theta2[0])* sin(angles.theta2[0]))/(den);

    //Arm2:
    den = -libnifalcon::a*sin(angles.theta3[1])*(sin(angles.theta1[1])*cos(angles.theta2[1])-sin(angles.theta2[1])*cos(angles.theta1[1]));

    double Jx1 = cos(phy[1])*cos(angles.theta2[1])*sin(angles.theta3[1])/den-sin(phy[1])*cos(angles.theta3[1])/den;
    double Jy1 = sin(phy[1])*cos(angles.theta2[1])*sin(angles.theta3[1])/den+cos(phy[1])*cos(angles.theta3[1])/den;
    double Jz1 = (sin(angles.theta2[1])* sin(angles.theta2[1]))/(den);

    //Arm3:
    den = -libnifalcon::a*sin(angles.theta3[2])*(sin(angles.theta1[2])*cos(angles.theta2[2])-sin(angles.theta2[2])*cos(angles.theta1[2]));

    double Jx2 = cos(phy[2])*cos(angles.theta2[2])*sin(angles.theta3[2])/den-sin(phy[2])*cos(angles.theta3[2])/den;
    double Jy2 = sin(phy[2])*cos(angles.theta2[2])*sin(angles.theta3[2])/den+cos(phy[2])*cos(angles.theta3[2])/den;
    double Jz2 = (sin(angles.theta2[2])* sin(angles.theta2[2]))/(den);


    J(0,0) = Jx0; J(0,1) = Jy0; J(0,2) = Jz0;
    J(1,0) = Jx1; J(1,1) = Jy1; J(1,2) = Jz1;
    J(2,0) = Jx2; J(2,1) = Jy2; J(2,2) = Jz2;

    J.setState(J.FULL);
    invert(J);

    //ToDo: Check to see if Jacobian inverted properly.
    //If not we need to take action.

    return J;

}
// Initeléshez
void controlForGui::init()
{

    unsigned int num_falcons = 0;
    unsigned int count;

    f = dev.getFalconFirmware();
    if(!dev.getDeviceCount(num_falcons))
    {
        std::cout << "Cannot get device count" << std::endl;
        args.isItConnected=false;
        return;
    }
    count = 0;
    std::cout << "Falcons found: " << (int)num_falcons << std::endl;
    args.isItFound=true;
    if(num_falcons == 0)
    {
        std::cout << "No falcons found, exiting..." << std::endl;
        args.isItFound=false;
        return;
    }
        int z=0;
        std::cout << "Opening falcon " << z + 1  << std::endl;
        if(!dev.open(z))
        {
            std::cout << "Cannot open falcon - Error: " << std::endl; // << dev.getErrorCode() << std::endl;
            args.isItConnected=false;
            return;
        }
        std::cout << "Opened falcon" << std::endl;
        args.isItConnected=true;

        if(!dev.isFirmwareLoaded())
        {
            std::cout << "Loading firmware" << std::endl;
            for(int i = 0; i < 10; ++i)
            {
                if(!dev.getFalconFirmware()->loadFirmware(true, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
                {
                    std::cout << "Could not load firmware" << std::endl;
                    args.isFirmWareLoaded=false;
                    return;
                }
                else
                {
                    std::cout <<"Firmware loaded" << std::endl;
                    args.isFirmWareLoaded=true;
                    break;
                }
            }
            if(!dev.isFirmwareLoaded())
            {
                std::cout << "Firmware didn't load correctly. Try running findfalcons again" << std::endl;
                return;
            }
        }




set_vectorzero(mPVect);
set_vectorzero(mDVect);
set_vectorzero(encoderAngles);
set_vectorzero(desAng);
set_vectorzero(errorVect);
set_vectorzero(prevErrorVect);
set_vectorzero(output);
set_vectorzero(torque);
set_vectorzero(SummKd);
set_vectorzero(AVG);
dt = 0.001;
Kp = 2;
Kd = 0.025;

for (int j=0; j<3 ;j++)
{
    for (int i=0; i<10;i++)
    {
        arr[j][i] =0;
    }
}

//lowPassIsOn = true;
firstRun = true;
loopCount = 0;

homePos[0] = 0.435;
homePos[1] = 0.435;
homePos[2] = 0.435;

thisIsHome[0] = 0;
thisIsHome[1] = 0;
thisIsHome[2] = 0.12;

pos[0] = 0;
pos[1] = 0;
pos[2] = 0.12;


}
void controlForGui::set_vectorzero(gmtl::Vec3d vect)
{
    /// simple macro to set vectors to zero
    vect[0]=0;
    vect[1]=0;
    vect[2]=0;
}

// Makrók
void controlForGui::setDesAng(Angle angles,gmtl::Vec3d& desiredAng)
{
    /// Copy proper values from Angle to Vector

    for (int i=0 ; i<3 ; i++)    {  desiredAng[i] = angles.theta1[i];    }
}
void controlForGui::runIOLoop()
{
    /// read and write data from / to the falcon
    ///  double runIOLoop() to ensure that no bullshit readings are present

    dev.runIOLoop();
    usleep(10);
    dev.runIOLoop();
}
void controlForGui::read_encoder(gmtl::Vec3d& encoderAngle)
{
    /// read encoder angles, and convert to radian
    int encoder[3] = {f->getEncoderValues()[0], f->getEncoderValues()[1], f->getEncoderValues()[2]};

    for (int i = 0 ; i< 3 ; i++)    {    encoderAngle[i] = k->getTheta(encoder[i]);    }
    encoderAngle *= 0.0174532925; //  convert to radian

}
gmtl::Vec3d controlForGui::addImpedance(gmtl::Vec3d& desiredPos, gmtl::Vec3d& posWithImp)
{
    /// Add calculated position from impedance modell to setpoint
    posWithImp=desiredPos;

    posWithImp[0] += (*mPosX)/1;  /// /1 is a scaling factor, not important
    posWithImp[1] += (*mPosY)/1;
    posWithImp[2] -= (*mPosZ)/1;

}
/// LED control
void controlForGui::setLedGreen()
{
    /// Set the LED color to GREEN
    f->setLEDStatus(3);
}
void controlForGui::setLedBlue()
{
    /// Set the LED color to BLUE
    f->setLEDStatus(4);
}
void controlForGui::setLedRed()
{
    /// Set the LED color to RED
    f->setLEDStatus(2 << 2 % 3);
}


double controlForGui::readTime()
{
    return time;
}

bool controlForGui::isFirmWareLoaded()
{
    if (!dev.isFirmwareLoaded())
    {
        return false;
    }
    else
    {
        return true;
    }
}
