#include "falcon/OMD/optosource.h"

OptoSource::OptoSource()
{

}


mytime_t Now()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    mytime_t millisecs = t.tv_sec * 1000;
    millisecs += t.tv_nsec / (1000 * 1000);
    return millisecs;
}


mytime_t NowMicro()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    mytime_t microsecs = t.tv_sec * 1000 * 1000;
    microsecs += t.tv_nsec / (1000);
    return microsecs;
}


mytime_t ElapsedTime(mytime_t p_Time)
{
    return Now() - p_Time;
}


mytime_t ElapsedTimeMicro(mytime_t p_Time)
{
    return NowMicro() - p_Time;
}


void MySleep(unsigned long p_uMillisecs)
{
    usleep(p_uMillisecs * 1000);
}

/*
 * Set the config to the DAQ
 * it is a blocking function; returns true, if the sending of
 * configuration is succeeded otherwise it returns false
 */
bool SetConfig(OptoDAQ & p_optoDAQ, int p_iSpeed, int p_iFilter)
{
    SensorConfig sensorConfig;
    sensorConfig.setSpeed(p_iSpeed);
    sensorConfig.setFilter(p_iFilter);
    mytime_t tNow = Now();

    bool bSuccess = false;
    do {
        bSuccess = p_optoDAQ.sendConfig(sensorConfig);
        if (bSuccess) {
            return true;
        }
        if (ElapsedTime(tNow) > 1000) {
            // 1 sec timeout
            return false;
        }
        MySleep(1);
    } while (bSuccess == false);
    return false;
}


void ShowInformation(OptoDAQ & p_optoDAQ, OPort & p_Port)
{
    std::string deviceName = std::string(p_Port.deviceName);
    std::string name = std::string(p_Port.name);
    std::string serialNumber = std::string (p_Port.serialNumber);
    int version = p_optoDAQ.getVersion();
    std::cout<<"Device Name: "<<deviceName<<std::endl;
    std::cout<<"Name: "<<name<<std::endl;
    std::cout<<"Serial Number: "<<serialNumber<<std::endl;
    std::cout<<"Version: "<<version<<std::endl;
}



/*
 * Opens the desired port
 * it returns true if port could be opened otherwise returns false
 */
bool OpenPort(OptoDAQ & p_optoDAQ, OptoPorts & p_Ports, int p_iIndex)
{
    MySleep(500); // We wait some ms to be sure about OptoPorts enumerated PortList
    OPort * portList = p_Ports.listPorts(true);
    int iLastSize = p_Ports.getLastSize();
    if (p_iIndex >= iLastSize) {
        // index overflow
        return false;
    }
    bool bSuccess = p_optoDAQ.open(portList[p_iIndex]);
    if (bSuccess) {
        ShowInformation(p_optoDAQ, portList[p_iIndex]);
    }
    return bSuccess;
}


void Config_default(OptoDAQ & p_optodaq, OptoPorts & p_optoPorts, int iPortindex)
{
    // Changeable values, feel free to play with them
    int iPortIndex = 0;  // The index of the port which will be opened
    int iSpeed = 1000; // Speed in Hz
    int iFilter = 150;  // Filter in Hz
    ///////////////////
    if (OpenPort(p_optodaq, p_optoPorts, iPortIndex) == false) {
        std::cout << "Could not open port" << std::endl;
    }
    bool bConfig = SetConfig(p_optodaq, iSpeed, iFilter);
    if (bConfig == false) {
        std::cout << "Could not set config" << std::endl;
        p_optodaq.close();
        return;
    }
    if (bConfig == true) {
        std::cout << "Config feltöltve" << std::endl;
        p_optodaq.getConfig();

    }
}

    /// take 5 samples, discard first one ( it's usually bullshit )
    /// calculate average -> output offset values
void Offseteles(OptoDAQ & poptodaq, double &Offx, double &Offy, double &Offz)
{
    OptoPackage pack;
    double Fxavg = 0;	double Fyavg = 0;	double Fzavg = 0;

    int sample = 5;
    for (int i = 0; i < sample; i++)
    {
        int bshit = poptodaq.read(pack, 0);
        double Fx = pack.x;
        double Fy = pack.y;
        double Fz = pack.z;
        std::cout << "Fx: " << Fx << "  Fy:" << Fy << " Fz:" << Fz << std::endl;
        if (i != 0)
        {
            Fxavg += Fx;
            Fyavg += Fy;
            Fzavg += Fz;
            MySleep(50);
        }
    }
    double Xoffset = Fxavg / (sample-1);
    double Yoffset = Fyavg / (sample-1);
    double Zoffset = Fzavg / (sample-1);

    std::cout << Xoffset << "  " << Yoffset << "  " << Zoffset << std::endl;

    MySleep(100);


    Offx = Xoffset;
    Offy = Yoffset;
    Offz = Zoffset;

    std::cout << " Az offsetek beállítva, szimuláció indul! " << std::endl;
}

    /// Function to read force, and subtract offset values and convert mN to N
void ReadForce(OptoDAQ & p_optodaq, double offsetek[], double* Fx, double* Fy, double* Fz)
{
    OptoPackage pack;

    int bullshit = p_optodaq.read(pack,0);

    if (bullshit == 1)
    {
    //std::cout << std::endl;
    //std::cout<<bullshit << std::endl;

    *Fx = (pack.x - offsetek[0])/1000;
    *Fy = (pack.y - offsetek[1])/1000;
    *Fz = (pack.z - offsetek[2])/1000;

    //std::cout<<"Beolvasott RAW erők: "<< *Fx << " ///////////// " << *Fy << " /////////// " << *Fz << std::endl;
    }
}

    /// Not sure why we have this here
void OffsetAll(OptoDAQ & optoDaq,double & Offsetx,double & Offsety,double & Offsetz)
{
    Offseteles(optoDaq, Offsetx, Offsety, Offsetz);
}
