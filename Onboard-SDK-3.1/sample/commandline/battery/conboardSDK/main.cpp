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

void control(VirtualRC *vrc ,int pitch,int roll);
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

    cout<<"Pass ENTER to continue ..."<<endl;fgetc(stdin);

    VirtualRC vrc(&api);
    vrc.setControl(true,VirtualRC::CutOff::CutOff_ToRealRC);

    for(int i=0;i<3;i++)
    {
        control(&vrc,300,300);
	sleep(1);
    }
    for(int i=0;i<3;i++)
    {
        control(&vrc,-600,0);
	sleep(1);
    }
    for(int i=0;i<3;i++)
    {
        control(&vrc,0,-600);
	sleep(1);
    }
    for(int i=0;i<3;i++)
    {
        control(&vrc,300,100);
	sleep(1);
    }


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
void control(VirtualRC *vrc ,int pitch,int roll)
{
    VirtualRCData vdata;
    vdata.yaw = 1024;
    vdata.throttle = 1024;
    vdata.pitch = 1024 + pitch;
    vdata.roll = 1024 + roll;
    vrc->sendData(vdata);
}
