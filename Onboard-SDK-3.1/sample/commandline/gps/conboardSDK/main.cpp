#include <iostream>
#include <sstream>
#include <string.h>
#include <thread>
#include <fstream>

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "DJI_API.h"
#include "DJI_Type.h"

#include "calculatePath.h"
#include "quaternion.h"

using namespace std;
using namespace DJI::onboardSDK;

WayPointData SetPoint(int index, double lat, double lon, double alt);
double latitude(const Flight* flight);
double longitude(const Flight* flight);
double altitude(const Flight* flight);

int main(int argc, char *argv[])
{
    HardDriverManifold driver("/dev/ttyAMA0", 230400);
    driver.init();
    
    CoreAPI api(&driver);
    
    ConboardSDKScript sdkScript(&api);
    ScriptThread st(&sdkScript);

    APIThread send(&api, 1);
    APIThread read(&api, 2);
    send.createThread();
    read.createThread();

    TaskData *taskdata;
    taskdata = (TaskData*)malloc(sizeof(TaskData));

    FlightData *flightdata;
    flightdata = (FlightData*)malloc(sizeof(FlightData));

    CallBackHandler FME;
    api.setFromMobileCallback(FME);
    api.ackdata = 0;
    
    ActivateData *data;
    data = (ActivateData *)malloc(sizeof(ActivateData));
        
    data->version = SDK_VERSION;
    data->ID = 1026279;
    char *key = "bba4b69aaef4bba3ebb38806cce9575ba2b43676d93cfc4fe7bf8a8e11b3e850";
    data->encKey = key;
    data->reserved = 2;
    
    //activate
    api.activate(data);

    //setcontrol
    unsigned char enable = 1;
    api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);

    sleep(2);

    const Flight *flight = new Flight(&api);
    PositionData posData;
    posData = flight->getPosition();// refer DJI_Type.h for parameter name
    cout<<"lati:"<<posData.latitude<<"\n";
    cout<<"longi:"<<posData.longitude<<"\n";


    WayPoint waypoint(&api);

    //init Waypoint
    WayPointInitData ifdata;
    ifdata.maxVelocity = 15;
    ifdata.idleVelocity = 15;
    ifdata.finishAction = 2;
    ifdata.executiveTimes = 1;
    ifdata.yawMode = 2;
    ifdata.traceMode = 0;
    ifdata.RCLostAction = 0;
    ifdata.gimbalPitch = 0;
    ifdata.latitude = flight->getPosition().latitude;
    ifdata.longitude = flight->getPosition().longitude;
    ifdata.altitude = 0;
    ifdata.indexNumber = 2;
    waypoint.init(&ifdata);
    cout<<"init"<<endl;
    sleep(3);

    //Set Waypoint
    WayPointData fdata;

    cout<<"Press ENTER to set 1st point..."<<endl;fgetc(stdin);
    fdata = SetPoint(0,latitude(flight),longitude(flight),altitude(flight));
    waypoint.uploadIndexData(&fdata);

    cout<<"Press ENTER to set 2nd point..."<<endl;fgetc(stdin);
    fdata = SetPoint(1,latitude(flight),longitude(flight),altitude(flight));
    waypoint.uploadIndexData(&fdata);

    waypoint.start();
    cout<< "Mission Start" <<endl;

    cout<<"Pass ENTER to pause ..."<<endl;fgetc(stdin);
    waypoint.pause(true);

    cout<<"Pass ENTER to resume ..."<<endl;fgetc(stdin);
    waypoint.pause(false);

    cout<<"Waitting ..."<<endl;fgetc(stdin);

    waypoint.stop();
    cout << "Mission Completed" << endl;

    cout<<"Pass ENTER to continue ..."<<endl;fgetc(stdin);

    return 0;
}

double latitude(const Flight *flight)
{
    return flight->getPosition().latitude;
}
double longitude(const Flight *flight)
{
    return flight->getPosition().longitude;
}
double altitude(const Flight *flight)
{
    return flight->getPosition().altitude;
}
WayPointData SetPoint(int index, double lat,double lon, double alt)
{
    WayPointData fdata;
    fdata.damping = 0;
    fdata.yaw = 0;
    fdata.gimbalPitch = 0;
    fdata.turnMode = 0;
    fdata.hasAction = 0;
    fdata.actionTimeLimit = 10;
    fdata.actionNumber = 0;
    fdata.actionRepeat = 0;
    for (int i = 0; i < 16; ++i)
    {
        fdata.commandList[i] = 0;
        fdata.commandParameter[i] = 0;
    }
    fdata.index = index;
    fdata.latitude = lat;
    fdata.longitude = lon;
    fdata.altitude = alt;

    return fdata;
}
WayPointData SetPoint(int index,const Flight* flight)
{
    return SetPoint(index,latitude(flight),longitude(flight),altitude(flight));
}
