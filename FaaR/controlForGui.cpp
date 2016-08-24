#include "controlForGui.h"
std::ofstream logging("./log.dat");

controlForGui::controlForGui()
{

}

controlForGui::controlForGui(FalconDevice& device, double& pX, double& pY, double& pZ ,
                             double& pKp , double& pKd,bool &plowPassIsOn)
{
    /// Függvény konstruktor, átadja az iterációból számított koordinátákat, illetve a vezérléshez a falcon objectet
    /// init() inicializálja a kapcsolatot, és a szükséges kezdő értékeket beállítja


    mPosX =&pX;
    mPosY =&pY;
    mPosZ =&pZ;
    mKp = &pKp;
    mKd = &pKd;
    lowPassIsOn=&plowPassIsOn;
    dev=device;
    dev.setFalconFirmware<FalconFirmwareNovintSDK>();
    init();

    currentState =goHomeMode;
         l = 0;

}

///Control rész
    /// Kommunikáció + olvasás + impedance
void controlForGui::trajectoryPath()
{   std::cout << "running" << std::endl;
    int loopcount = 0;
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

///Trajektória generálás pontok beolvasásával
void controlForGui::generateTrajectory()
{
    gmtl::Vec3d encoderAng;         // encoder szögeket tároljuk benne
    read_encoder(encoderAng);       /// Beadjuk neki az encoderAng vektort, belemásolja a jelenlegi encoder szögeket
    FK(encoderAng);
    std::cout << pos << std::endl;
}
void controlForGui::pakoldAt(std::vector<double> ang1, std::vector<double> ang2, std::vector<double> ang3)
{
    /// minden lefutási ciklusban ugrik egyet a loopcount,
    /// és a következő célkoordinátát bemásolja a szükséges helyre
    /// a tömbből, amiben a pálya pontjait tároljuk

    wayToGo[0] = ang1[loopCount];
    wayToGo[1] = ang2[loopCount];
    wayToGo[2] = ang3[loopCount];

    /*wayToGoEnd[0] = trajTh1[trajectory.count-1];
    wayToGoEnd[1] = trajTh2[trajectory.count-1];
    wayToGoEnd[2] = trajTh3[trajectory.count-1];*/
    loopCount++;
}

    ///PID-es cuccok
void controlForGui::FalconLoop()
{
    gmtl::Vec3d encoderAng;  // encoder szögeket tároljuk benne
    gmtl::Vec3d refAng;      //  elérni kívánt (theta1) szögeket tároljuk benne
    Angle refPosAng;         //  IK -ebbe menti ki az összes számolt szöget
    gmtl::Vec3d desiredOutput, posWithImpedance;

    switch (currentState)
    {
    case goHomeMode:
    {

        runIOLoop();                    //  Kommunikáció megtörténik
        read_encoder(encoderAng);       /// Beadjuk neki az encoderAng vektort, belemásolja a jelenlegi encoder szögeket
        IK(refPosAng,thisIsHome,refAng);       /// A pozícióhoz tartozó szögeket meghatározzuk ->  refAng-ba kimásolja a megfelelő thetákat
        interPol(encoderAng,refAng);    /// Első hívásra felosztja a pályát, utána mindig növeli a wayToGo értéket egységnyivel
        PID(wayToGo,encoderAng);        //  PID controllert megkérjük szabályozzon a kívánt szögre, refAng -> cél , encoderAng -> jelenlegi pozíció
        setLedGreen();                  //  LED zöldre áll, jelezve hogy fut a program
        FK(encoderAng);
        //std::cout<<encoderAng<<std::endl;
        //std::cout<< "[goHomeMode] ide mennék : " << wayToGo << std::endl;
        if (loopCount == 1000)
        {
            isAtHome=true;
            std::cout<< "[Home position reached] " << std::endl;
            justStayHome = true;
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
        setPid(*mKp,*mKd);  /// PD értékeit átveszi GUI-ból -> átállítja
        runIOLoop();                    //  Kommunikáció megtörténik
        read_encoder(encoderAng);       /// Beadjuk neki az encoderAng vektort, belemásolja a jelenlegi encoder szögeket
        addImpedance(endPos,posWithImpedance);              //  Impedanciát hozzáadjuk a kívánt pozícióhoz -> új érték pos-ban tárolva
        IK(refPosAng,posWithImpedance,refAng);              /// Az impedanciával hozzáadott pozícióhoz tartozó szögeket meghatározzuk
        PID(refAng,encoderAng);         /// PID controllert megkérjük szabályozzon a kívánt szögre, refAng -> cél , encoderAng -> jelenlegi pozíció
        setLedGreen();                  ///  LED zöldre áll, jelezve hogy fut a program
        FK(encoderAng);
        //std::cout<<encoderAng<<std::endl;
        //std::cout<< "[stayMode] ide mennék : " << refAng << std::endl;
        break;
    }
    case followPathMode:
    {
        setPid(*mKp,*mKd);              /// PD értékeit átveszi GUI-ból -> átállítja
        runIOLoop();                    //  Kommunikáció megtörténik
        read_encoder(encoderAng);       /// Beadjuk neki az encoderAng vektort, belemásolja a jelenlegi encoder szögeke
        pakoldAt(trajTh1, trajTh2, trajTh3);
        FK(wayToGo,desiredOutput);
        addImpedance(desiredOutput,posWithImpedance);
        IK(refPosAng,posWithImpedance,wayToGo);
        PID(wayToGo,encoderAng);        //  PID controllert megkérjük szabályozzon a kívánt szögre, refAng -> cél , encoderAng -> jelenlegi pozíció
        setLedGreen();                  //  LED zöldre áll, jelezve hogy fut a program
        FK(encoderAng);
        posx.push_back(returnpos()[0]);
        posy.push_back(returnpos()[1]);
        posz.push_back(returnpos()[2]);
        //std::cout<<encoderAng<<std::endl;
        //std::cout<< "[followPathMode] ide mennék : " << wayToGo << std::endl;
        if (loopCount == trajectory.count)
        {
            endPos = homeAng;
            isAtHome=true;
            std::cout<< "Odaértem! " << std::endl;
            loopCount = 0;
            currentState=goHomeMode;  /// state swtich -> elmászik home pozicióba ha végzett a feladatával
            firstRun=true;
        }
        break;
    }
    case genTrajMode:
    {
        setPid(*mKp,*mKd);              /// PD értékeit átveszi GUI-ból -> átállítja
        runIOLoop();                    //  Kommunikáció megtörténik
        read_encoder(encoderAng);       /// Beadjuk neki az encoderAng vektort, belemásolja a jelenlegi encoder szögeke
        pakoldAt(genTrajTh1, genTrajTh2, genTrajTh3);
        FK(wayToGo,desiredOutput);
        addImpedance(desiredOutput,posWithImpedance);
        IK(refPosAng,posWithImpedance,wayToGo);
        PID(wayToGo,encoderAng);        //  PID controllert megkérjük szabályozzon a kívánt szögre, refAng -> cél , encoderAng -> jelenlegi pozíció
        setLedGreen();                  //  LED zöldre áll, jelezve hogy fut a program
        FK(encoderAng);
        posx.push_back(returnpos()[0]);
        posy.push_back(returnpos()[1]);
        posz.push_back(returnpos()[2]);
        //std::cout<<encoderAng<<std::endl;
        //std::cout<< "[followPathMode] ide mennék : " << wayToGo << std::endl;
        if (loopCount == trajectory.genCount)
        {
            endPos = homeAng;
            isAtHome=true;
            std::cout<< "Odaértem! " << std::endl;
            loopCount = 0;
            currentState=goHomeMode;  /// state swtich -> elmászik home pozicióba ha végzett a feladatával
            firstRun=true;
        }
        break;
    }
    case logPathMode:
    {

        runIOLoop();  /// kommunikáció
        read_encoder(encoderAng); /// szögek kiolvasása
        break;
    }

    case replayMode:
    {
        setPid(*mKp,*mKd);              /// PD értékeit átveszi GUI-ból -> átállítja
        runIOLoop();                    //  Kommunikáció megtörténik
        read_encoder(encoderAng);       /// Beadjuk neki az encoderAng vektort, belemásolja a jelenlegi encoder szögeke
        pakoldAt(logTh0, logTh1, logTh2);
        FK(wayToGo,desiredOutput);
        addImpedance(desiredOutput,posWithImpedance);
        IK(refPosAng,posWithImpedance,wayToGo);
        PID(wayToGo,encoderAng);        //  PID controllert megkérjük szabályozzon a kívánt szögre, refAng -> cél , encoderAng -> jelenlegi pozíció
        setLedGreen();                  //  LED zöldre áll, jelezve hogy fut a program
        FK(encoderAng);

        //std::cout<<"[replay Mode]"<<encoderAng<<std::endl;
        if (loopCount == replayCount)
        {
            endPos = pos;
            isAtHome=true;
            std::cout<< "[replay] Pálya vége  " << std::endl;
            loopCount = 0;
            currentState=goHomeMode;  /// state swtich -> elmászik home pozicióba ha végzett a feladatával
            firstRun=true;
        }
        break;
    }

    }

    encoderAngles=encoderAng;
    desAng=refAng;
}
void controlForGui::PID(gmtl::Vec3d desired,gmtl::Vec3d encoderAngle)
{
    /// Saját PID implementáció ( I tag nélkül )
gmtl::Vec3d hibavektor;
hibavektor = desired-encoderAngle;
Kpf = hibavektor*Kp;   // P - tag vektora   -> Hiba * Kp
// D-tag
gmtl::Vec3d posvaltozas = hibavektor - elozo_hibavektor;
leptetes(posvaltozas);
if (*lowPassIsOn == true)
{
    Kdf = posvaltozas* Kd / dt;
}
if (*lowPassIsOn == false)
{
    Kdf = Kdfilter* Kd /dt;
}

// Tagokat összeadjuk
output = Kpf + Kdf;
elozo_hibavektor = hibavektor;

sendTorque();                   /// PID a globális torque változónak átadja a kívánt erőket, lehatároljuk őket, majd kiírjuk

}
void controlForGui::setPid(double nKp,double nKd)
{
    /// Paraméterekhez való hozzáférést teszi lehetővé futás közben ez a függvény
    Kp  = nKp;
    Kd  = nKd;

}
void controlForGui::Nyomatekhatarolas()
{
    /// Max nyomatékot megállípítjuk, ha bármelyik csukló e-fölé mennek levágjuk, többit hozzá illesztjük
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
    //printvect(torque);
}
void controlForGui::sendTorque()
{
    /// Nyomatékot kiírjuk a falconnak
    torque = output;
    Nyomatekhatarolas();
    torque *= 10000.0;
    boost::array<int, 3> enc_vec;
    enc_vec[0] = -torque[0];
    enc_vec[1] = -torque[1];
    enc_vec[2] = -torque[2];
    f->setForces(enc_vec);
}
void controlForGui::sendZeroTorque()
{
/// Amíg pihentetjük a falcont -> 0 torque a csuklókra
        *mKp = 0; *mKd = 0;
        boost::array<int, 3> enc_vec;
        enc_vec[0] = -torque[0];
        enc_vec[1] = -torque[1];
        enc_vec[2] = -torque[2];
        f->setForces(enc_vec);

}

/// Trajektória tervezés

void controlForGui::interPol(gmtl::Vec3d encoderAng, gmtl::Vec3d& goalAngle)
{
    if (firstRun==true)
    {
    startEncoder = encoderAng;
    hibavektorHome = goalAngle-encoderAng;
    lamda = hibavektorHome / 1000.0;
    firstRun=false;
    }
    firstRun=false;
    wayToGo = startEncoder + (lamda * loopCount);
    loopCount++;

 }
void controlForGui::resetFirstRun()
{
    firstRun = true;
    loopCount = 0;
    isAtHome = false;

}
/// Low-pass filter
void controlForGui::sumtomb()
{
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
void controlForGui::leptetes(gmtl::Vec3d KD)
{ /// Adott egy n elemű tömb, n. elemet felülírjuk az n-1. elemmel, a 0. elem helyére pedig új értéket írunk be


 for (int j = 0 ; j<3 ; j++)
   {
    for (int i = 9; i > 0 ; i--)
    {
        arr[j][i] = arr[j][i-1];

    }
    arr[j][0]=KD[j];
   }
 sumtomb();

}
/// Értékvisszaadás logoláshoz
gmtl::Vec3d controlForGui::returnDesiredAng()
{
    return desAng;
}
gmtl::Vec3d controlForGui::returnEncoderAngles()
{
    ///Visszaadja az encoder szögeket
    return encoderAngles;
}
gmtl::Vec3d controlForGui::returnKp()
{
    return Kpf;
}
gmtl::Vec3d controlForGui::returnKd()
{
    return Kdf;
}
gmtl::Vec3d controlForGui::returnoutput()
{
    return Kpf + Kdf;
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

    setDesAng(angles,importantAng); // a fontos szögeket kimentjük -> kiadjuk


}
void controlForGui::FK(const gmtl::Vec3d& theta0)
{

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
        return;
    }
    count = 0;
    std::cout << "Falcons found: " << (int)num_falcons << std::endl;
    if(num_falcons == 0)
    {
        std::cout << "No falcons found, exiting..." << std::endl;
        return;
    }
        int z=0;
        std::cout << "Opening falcon " << z + 1  << std::endl;
        if(!dev.open(z))
        {
            std::cout << "Cannot open falcon - Error: " << std::endl; // << dev.getErrorCode() << std::endl;
            return;
        }
        std::cout << "Opened falcon" << std::endl;

        if(!dev.isFirmwareLoaded())
        {
            std::cout << "Loading firmware" << std::endl;
            for(int i = 0; i < 10; ++i)
            {
                if(!dev.getFalconFirmware()->loadFirmware(true, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
                {
                    std::cout << "Could not load firmware" << std::endl;
                    return;
                }
                else
                {
                    std::cout <<"Firmware loaded" << std::endl;
                    break;
                }
            }
            if(!dev.isFirmwareLoaded())
            {
                std::cout << "Firmware didn't load correctly. Try running findfalcons again" << std::endl;
                return;
            }
        }




set_vectorzero(Kpf);
set_vectorzero(Kdf);
set_vectorzero(Kdfprev);
set_vectorzero(encoderAngles);
set_vectorzero(desAng);
set_vectorzero(hibavektor);
set_vectorzero(output);
set_vectorzero(torque);
set_vectorzero(elozo_hibavektor);
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
thisIsHome[2] = 0.11;

pos[0] = 0;
pos[1] = 0;
pos[2] = 0.11;


}
void controlForGui::set_vectorzero(gmtl::Vec3d vect)
{/// egyszerű makró, inicializáláshoz, gmtl::Vec3d vektort kinulláz
    vect[0]=0;
    vect[1]=0;
    vect[2]=0;
}

// Makrók
void controlForGui::setDesAng(Angle angles,gmtl::Vec3d& desiredAng)
{
    /// kimentjük egy vektorba a kívánt pozícióból számolt szögeket
    /// angle (IK függvényből visszakapott struct, ami tartalmazza
    /// az összes szögét az összes karnak , nekünk a theta1 -ek
    /// kellenek

    for (int i=0 ; i<3 ; i++)    {  desiredAng[i] = angles.theta1[i];    }
}
void controlForGui::runIOLoop()
{
    /// Megkérjük a falcont, hogy had olvassunk és írjunk bele
    /// Bullshit readingek elkerülése érdekében lefuttatjuk 2x

    dev.runIOLoop();
    usleep(10);
    dev.runIOLoop();

    // loopCount = f->getLoopCount();

    //std::cout<< loopCount << std::endl;
}
void controlForGui::read_encoder(gmtl::Vec3d& encoderAngle)
{
    /// Kiolvassuk encoderből a szögeket, és konvertáljuk radiánba
    int encoder[3] = {f->getEncoderValues()[0], f->getEncoderValues()[1], f->getEncoderValues()[2]};

    for (int i = 0 ; i< 3 ; i++)    {    encoderAngle[i] = k->getTheta(encoder[i]);    }
    encoderAngle *= 0.0174532925; // konverzió radiánba

}
gmtl::Vec3d controlForGui::addImpedance(gmtl::Vec3d& desiredPos, gmtl::Vec3d& posWithImp)
{
    ///A beadott pozíció vektorhoz hozzáadja az elvárt dinamikai
    /// modellből érkező koordinátákat
    posWithImp=desiredPos;

    posWithImp[0] += (*mPosX)/1;
    posWithImp[1] += (*mPosY)/1;
    posWithImp[2] -= (*mPosZ)/1;

}
/// LED színkezelés
void controlForGui::setLedGreen()
{
    /// Falcon led színét zöldre állítja
    f->setLEDStatus(3);
}
void controlForGui::setLedBlue()
{
    /// Falcon led színét kékre állítja
    f->setLEDStatus(4);
}
void controlForGui::setLedRed()
{
    /// Falcon led színét pirosra állítja
    f->setLEDStatus(2 << 2 % 3);
}


double controlForGui::readTime()
{
    return time;
}
