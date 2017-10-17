#include <iostream>
#include <sstream>
#include <thread>
#include <fstream>
#include <string.h>
#include <time.h>

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "DJI_API.h"
#include "DJI_WayPoint.h"

#include "calculatePath.h"
#include "quaternion.h"

using namespace std;
using namespace DJI::onboardSDK;

int i;
int id;
bool Exit = false;
bool CExit = false;

Attitude *head;

string settingInfo = "setting";

fstream fs;
fstream fp;
//fstream fg;

typedef struct threadData
{
    void* api;
    char *filename;
}treadData;

typedef struct PredictData
{
    void* api;
    calculatePath *cp;
}PredictData;

void outputsetting(void);
void *CollectData(void *ptr);
void *CompareData(void *ptr);
void *SelfPrediction(void *ptr);
WayPointData SetPoint(int index, double lat, double lon, double alt);

int main(int argc, char *argv[])
{
    time_t t = time(NULL);
    tm *ptm = localtime(&t);
    char datafilename[100];
    strftime(datafilename, 100, "./%m%d_%H%M-data.txt", ptm);

    cout << datafilename << endl;

    outputsetting();

    calculatePath *cp = new calculatePath();
    //Attitude *head;
    head = (Attitude*)malloc(sizeof(Attitude));
    head->nextAttitude = NULL;
    head->preAttitude = NULL;

    cp->getNowAttitude()->nextAttitude = head;

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
    
    ActivateData *data;
    data = (ActivateData *)malloc(sizeof(ActivateData));
        
    data->version = SDK_VERSION;
    data->ID = 1026279;
    char *key = "bba4b69aaef4bba3ebb38806cce9575ba2b43676d93cfc4fe7bf8a8e11b3e850";
    data->encKey = key;
    data->reserved = 2;
    
    //activate
    API_LOG(&driver, STATUS_LOG, "LOG Test\n");
    api.activate(data);
    sleep(2);
    //cout << "api.ackdata = " << api.ackdata << endl;
    
    //setcontrol
    unsigned char enable = 1;
    //api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);
    //sleep(5);
    api.ackdata = 0;

    api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);

    cout << "Receive from mobile" << endl;

    CallBackHandler FME;
    api.setFromMobileCallback(FME);
    
    //int a[1000];
    //int i = 0;
    while(true){
        api.ackdata = 0; 
        while(api.ackdata == 0){
	    //api.send(0, 0, SET_BROADCAST, CODE_FROMMOBILE, (unsigned char *)&(api.ackdata), sizeof(api.ackdata));  //FromMobile
	    api.send(0, 0, SET_BROADCAST, CODE_FROMMOBILE, (unsigned char *)&(api.ackdata), sizeof(api.ackdata), CoreAPI::setControlCallback, 500, 3);  //FromMobile
	    //a[i] = api.ackdata;
	    //i++;
	    //cout << api.ackdata << endl;
        }
        printf("\napi.ackdata = %c\n", api.ackdata);
    }

    if (api.ackdata == 100){
	return 0;
    }
    cout << "Got data" << endl;

    cout << "Set control" << endl;
    //api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);
    sleep(5);

    threadData *tfirstdata;
    tfirstdata = (threadData*)malloc(sizeof(threadData));
    tfirstdata->api = (void*)&api;
    tfirstdata->filename = datafilename;

    pthread_t id;
    int ret = pthread_create(&id, NULL, CompareData, (void*)tfirstdata);
    if(ret!=0){
         printf ("Create pthread error!\n");
         exit (1);
    }
    else{
         CExit = true;
    }

    //Takeoff
    TaskData *taskData;
    taskData = (TaskData*)malloc(sizeof(TaskData));
    taskData->cmdSequence++;
    taskData->cmdData = 4;
    api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskData), sizeof(TaskData), Flight::taskCallback, 100, 3); 
    sleep(5);

    WayPoint waypoint(&api);

    //init Waypoint
    WayPointInitData ifdata;
    ifdata.maxVelocity = 10;
    ifdata.idleVelocity = 0;
    ifdata.finishAction = 1;
    ifdata.executiveTimes = 1;
    ifdata.yawMode = 0;
    ifdata.traceMode = 0;
    ifdata.RCLostAction = 0;
    ifdata.gimbalPitch = 0;
    ifdata.latitude = api.getBroadcastData().pos.latitude;
    ifdata.longitude = api.getBroadcastData().pos.longitude;
    ifdata.altitude = 0;
    ifdata.indexNumber = 2;
    waypoint.init(&ifdata);
    cout << endl;
    sleep(5);

    //Set Waypoint
    int index = 0;
    double lat = api.getBroadcastData().pos.latitude;
    double lon = api.getBroadcastData().pos.longitude;
    double alt = 3.0;
    cout << "lar : " << lat << endl << "lon : " << lon << endl;

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
    fdata.index = index++;
    fdata.latitude = lat;
    fdata.longitude = lon;
    fdata.altitude = alt;

    waypoint.uploadIndexData(&fdata);
    cout << endl;
    sleep(3);

    fdata.index = index++;
    fdata.latitude = lat + 0.000001;
    fdata.longitude = lon;
    fdata.altitude = alt;
    waypoint.uploadIndexData(&fdata);
    cout << endl;
    sleep(3);

    fdata.index = index++;
    fdata.latitude = lat + 0.000001;
    fdata.longitude = lon + 0.000001;
    fdata.altitude = alt;
    waypoint.uploadIndexData(&fdata);
    cout << endl;
    sleep(3);

    fdata.index = index++;
    fdata.latitude = lat;
    fdata.longitude = lon + 0.000001;
    fdata.altitude = alt;
    //waypoint.uploadIndexData(&fdata);
    cout << endl;
    sleep(3);

    //waypoint.uploadIndexData(&fdata);
    cout << endl;
    sleep(5);

    //Waypoint Mission Start
    uint8_t start = 0;
    api.send(2, 0, SET_MISSION, CODE_WAYPOINT_SETSTART, &start, sizeof(start), DJI::onboardSDK::missionCallback, 500, 2);
    //Waypoint Mission Finished
    cout << "Mission Completed" << endl;

    sleep(5);

    CExit = false;

    cp->freeAttitude(head);
    //free(cp);
    //free(head);
    //pthread_exit(&id);

    return 0;
}

void outputsetting(void)
{
    fs.open(settingInfo, ios::out);
    if(!fs){
        cout << "Fail to open file: " << settingInfo  << endl;
    }
    else{
        cout << settingInfo << " open successfully." << endl;
    }
    fs << "Attitude sampling rate : 0.7 sec" << endl;
    fs << "GPS sampling rate : 0.1 sec" << endl;
    fs << "Experiment target : height error" << endl;
    fs.close();
}

void *CollectData(void *ptr)
{
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 700000000L;

    threadData *tData = (threadData*)ptr;

    fp.open(tData->filename, ios::out);
    if(!fp){
        cout << "Fail to open file: " << tData->filename << endl;
        Exit = false;
    }
    else{
        cout << tData->filename << " open successfully." << endl;
    }

    const Flight *flight = new Flight((CoreAPI*)(tData->api));
    //int i = 0;

    calculatePath *cp = new calculatePath();
    cp->getNowAttitude()->nextAttitude = head;

    float q_q0, q_q1, q_q2, q_q3;
    float Vx, Vy, Vz, yaw;
    float lat, lon, height;

    while(Exit)
    {
        //cout << "In thread" << endl;
        //fp << "i = " << i << "-------------------------" << endl;
        fp << "-------------------------" << endl;
        cout << "id = " << id++ << endl;

        q_q0 = flight->getQuaternion().q0;
        q_q1 = flight->getQuaternion().q1;
        q_q2 = flight->getQuaternion().q2;
        q_q3 = flight->getQuaternion().q3;

        lat = flight->getPosition().latitude;
        //cout << "lat = " << lat << endl;
        lon = flight->getPosition().longitude;
        //cout << "lon = " << lon << endl;
        height = flight->getPosition().height;

        Vx = flight->getVelocity().x;
        Vy = flight->getVelocity().y;
        Vz = flight->getVelocity().z;

        yaw = flight->getApi()->getBroadcastData().rc.yaw;
        //cout << "yaw = " << yaw << endl;

        fp << "q0 = " << q_q0 << endl;
        fp << "q1 = " << q_q1 << endl;
        fp << "q2 = " << q_q2 << endl;
        fp << "q3 = " << q_q3 << endl;

        fp << "latitude = " << lat << endl;
        fp << "longitude = " << lon << endl;
        fp << "altitude = " << flight->getPosition().altitude << endl;
        fp << "height = " << height << endl;
        //printf("health = %d\n", flight->getPosition().health);
	//cout << "Time = " << flight->getApi()->getTime().time << endl;
        //printf("Time = %d\n nanoTime = %d\nsyncFlag = %d\n", flight->getApi()->getTime().time, flight->getApi()->getTime().nanoTime, flight->getApi()->getTime().syncFlag);

        fp << "Vx = " << Vx << endl;
        fp << "Vy = " << Vy << endl;
        fp << "Vz = " << Vz << endl;
        fp << "yaw = " << yaw << endl;

        cp->addAttitude(cp->getNowAttitude(), q_q0, q_q1, q_q2, q_q3, Vx, Vy, Vz, yaw);

        //sleep(1);
        nanosleep(&tim , &tim2);
        printf("fly status : %d\n", flight->getApi()->getBroadcastData().status);
        if( flight->getApi()->ackdata == 0 && (flight->getApi()->getBroadcastData().status == 1 || flight->getApi()->getBroadcastData().status == 0)){
             char a = '1';
             flight->getApi()->send(0, 0, SET_ACTIVATION, CODE_TOMOBILE, (unsigned char *)&a, sizeof(a)); //To Mobile
	     cout << "Send to Mobile" << endl;
        }
	
        if(flight->getApi()->getBroadcastData().status == NULL){
             flight->getApi()->ackdata = 100;
             //break;
        }
    }
    //fp.close();
    cout << "CollectDataThread finished" << endl;

}

void *CompareData(void *ptr)
{
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 100000000L;

    threadData *tData = (threadData*)ptr;

    fstream fg;
    fg.open(tData->filename, ios::out);
    if(!fg){
        cout << "Fail to open file: " << tData->filename << endl;
    }
    else{
        cout << tData->filename << " open successfully." << endl;
    }

    const Flight *flight = new Flight((CoreAPI*)(tData->api));
    //int i = 0;

    float q_q0, q_q1, q_q2, q_q3;
    float Vx, Vy, Vz, yaw;
    float lat, lon, height;

    while(CExit)
    {
        fg << "i = " << i++ << "-------------------------" << endl;
        //cout << "i = " << i++ << endl;

        q_q0 = flight->getQuaternion().q0;
        q_q1 = flight->getQuaternion().q1;
        q_q2 = flight->getQuaternion().q2;
        q_q3 = flight->getQuaternion().q3;

        lat = flight->getPosition().latitude;
        lon = flight->getPosition().longitude;
        height = flight->getPosition().height;

        Vx = flight->getVelocity().x;
        Vy = flight->getVelocity().y;
        Vz = flight->getVelocity().z;

        yaw = flight->getApi()->getBroadcastData().rc.yaw;

        //fg << "q0 = " << q_q0 << endl;
        //fg << "q1 = " << q_q1 << endl;
        //fg << "q2 = " << q_q2 << endl;
        //fg << "q3 = " << q_q3 << endl;

	fg << "latitude = " << lat << endl;
        fg << "longitude = " << lon << endl;
        fg << "altitude = " << flight->getPosition().altitude << endl;
        fg << "height = " << height << endl;

        //cout << "Time = " << flight->getApi()->getTime().time << endl;
        //printf("Time = %d\n nanoTime = %d\nsyncFlag = %d\n", flight->getApi()->getTime().time, flight->getApi()->getTime().nanoTime, flight->getApi()->getTime().syncFlag);

        //fg << "Vx = " << Vx << endl;
        //fg << "Vy = " << Vy << endl;
        //fg << "Vz = " << Vz << endl;
        //fg << "yaw = " << yaw << endl;

        //sleep(1);
        nanosleep(&tim , &tim2);
    }
    cout << "compareThread is finished." << endl;
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

