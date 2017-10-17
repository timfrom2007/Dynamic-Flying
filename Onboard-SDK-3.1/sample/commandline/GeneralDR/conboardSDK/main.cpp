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

#include "calculatePath.h"
#include "quaternion.h"
#include "Vincenty.h"
#include "slamac.h"

using namespace std;
using namespace DJI::onboardSDK;

int i;
int id;
bool Exit = false;
bool CExit = false;

Attitude *head;

string settingInfo = "/var/setting.txt";

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

int main(int argc, char *argv[])
{
    char datafilename[100];
    char firstfile[100];
    char secondfile[100];

    fstream ft;
    ft.open("/var/ExperimentTime.txt", ios::in);
    if(!ft){
        cout << "Fail to open file: /var/ExperimentTime.txt" << endl;
    }
    else{
        cout << "/var/ExperimentTime.txt open successfully." << endl;
    }
    string s;
    stringstream ss;
    char tmp[50];
    getline(ft, s);
    ss << s.c_str();
    ss >> tmp;
    ss.clear();
    
    ss << "/var/" << tmp << "-data.txt";
    ss >> datafilename;
    ss.clear();
    
    ss << "/var/" << tmp << "-gpsdata.txt";
    ss >> firstfile;
    ss.clear();
    ss << "/var/" << tmp << "-comparedata.txt";
    ss >> secondfile;
    ss.clear();
    
    cout << datafilename << endl;
    cout << firstfile << endl;
    cout << secondfile<< endl;
    ft.close();

    ft.open("/var/ExperimentTime.txt", ios::out);
    if(!ft){
        cout << "Fail to open file: /var/ExperimentTime.txt" << endl;
    }
    else{
        cout << "/var/ExperimentTime.txt open successfully." << endl;
    }
    int op = atoi((s.substr(s.length()-1)).c_str());
    cout << s << endl;
    if(op == 9){
	op = atoi((s.substr(s.length()-2)).c_str());
	op++;
	ss << s.substr(0, s.length()-2) << op;
	ss >> tmp;
    }
    else{
	op++;
	//cout << op << endl;
    	ss << s.substr(0, s.length()-1) << op;
    	ss >> tmp;
    }
    cout << tmp << endl;
    ft << tmp;
    ft.close();
    /*
    double *lat2out = new double();
    double *lon2out = new double();
    Vincenty *v = new Vincenty();
    v->destVincenty(0.0, 0.0, 0, 10000, lat2out, lon2out);
    cout << "lat2out = " << *lat2out << ", lon2out = " << *lon2out << endl;
    double dis =  v->distVincenty(0.0, 0.0, *lat2out, *lon2out); 
    cout << "distance = " << dis << endl;
    */
    outputsetting();

    calculatePath *cp = new calculatePath();

    HardDriverManifold driver("/dev/ttyAMA0", 230400);
    driver.init();
    driver.freeMemory();
 
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
    api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);
    sleep(2);

    api.ackdata = 0;
    
    //Takedff
    taskdata->cmdSequence++;
    taskdata->cmdData = 4;
    api.ackdata = 0;
    for(int i = 0; i < 10; i++){
         cout << "Landing" << endl;
         api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(TaskData), Flight::taskCallback, 100, 3);
         if(api.ackdata == 2){
		api.ackdata = 0;
                break;
         }
    }
    
    flightdata->flag = 0x40;
    flightdata->x = 0;
    flightdata->y = 0;
    flightdata->z = 0;
    flightdata->yaw = 0;
    for(int i = 0; i < 10; i++){
	 api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData), Flight::taskCallback, 100, 3);
    }
    
    threadData *tfirstdata;
    tfirstdata = (threadData*)malloc(sizeof(threadData));
    tfirstdata->api = (void*)&api;
    tfirstdata->filename = datafilename;
    //pthread
    pthread_t id;
    int ret;
    ret = pthread_create(&id, NULL, CollectData, (void*)tfirstdata);
    if(ret!=0){
         printf ("Create pthread error!\n");
         exit (1);
    }
    else{
	 Exit = true;
    }
    char a = '1';

    double lat = api.getBroadcastData().pos.latitude;
    double lon = api.getBroadcastData().pos.longitude;
    double height = api.getBroadcastData().pos.height;
    cout << "hoem lat = " << lat << ", lon = " << lon << ", height = " << height << endl;
    cp->GpsPoint(api, lat, lon, height, lat+0.0002, lon+0.0002, 4);
    sleep(1);
    api.send(0, 0, SET_ACTIVATION, CODE_TOMOBILE, (unsigned char *)&a, sizeof(a)); //To Mobile

    a = '2';
    //lat = api.getBroadcastData().pos.latitude;
    //lon = api.getBroadcastData().pos.longitude;
    height = api.getBroadcastData().pos.height;
    cp->GpsPoint(api, lat+0.0002, lon+0.0002, 4, lat+0.0004, lon, 6);
    sleep(1);
    api.send(0, 0, SET_ACTIVATION, CODE_TOMOBILE, (unsigned char *)&a, sizeof(a)); //To Mobile

    a = '3';
    //lat = api.getBroadcastData().pos.latitude;
    //lon = api.getBroadcastData().pos.longitude;
    height = api.getBroadcastData().pos.height;
    cp->GpsPoint(api, lat+0.0004, lon, 6, lat+0.0002, lon-0.0002, 8);
    sleep(1);
    api.send(0, 0, SET_ACTIVATION, CODE_TOMOBILE, (unsigned char *)&a, sizeof(a)); //To Mobile

    a = '4';
    //lat = api.getBroadcastData().pos.latitude;
    //lon = api.getBroadcastData().pos.longitude;
    height = api.getBroadcastData().pos.height;
    cp->GpsPoint(api, lat+0.0002, lon-0.0002, 8, lat, lon, 10);
    sleep(1);
    api.send(0, 0, SET_ACTIVATION, CODE_TOMOBILE, (unsigned char *)&a, sizeof(a)); //To Mobile

    Exit = false;

    //Landing
    taskdata->cmdSequence++;
    taskdata->cmdData = 6;
    api.ackdata = 0;
    for(int i = 0; i < 10; i++){
         cout << "Landing" << endl;
         api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(TaskData), Flight::taskCallback, 100, 3);
         if(api.ackdata == 2){
                break;
         }
    }

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
    //fs << "Attitude sampling rate : 0.7 sec" << endl;
    fs << "GPS sampling rate : 0.1 sec" << endl;
    fs << "Experiment target : waypoint" << endl;
    fs.close();
}

void *CollectData(void *ptr)
{
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 100000000L;

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

    while(true)
    {
        //cout << "In thread" << endl;
        //fp << "i = " << i << "-------------------------" << endl;
        fp << "-------------------------" << endl;
        //cout << "tid = " << id++ << endl;

        q_q0 = flight->getQuaternion().q0;
        q_q1 = flight->getQuaternion().q1;
        q_q2 = flight->getQuaternion().q2;
        q_q3 = flight->getQuaternion().q3;

        lat = flight->getPosition().latitude;
        //cout << "lat = " << lat << endl;
        lon = flight->getPosition().longitude;
        //cout << "lon = " << lon << endl;
        height = flight->getPosition().height;
        cout << "lat = " << lat << ", lon = " << lon << ", height = " << height << endl;

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

	//cout << "Vx = " << Vx << endl;
        //cout << "Vy = " << Vy << endl;
        //cout << "Vz = " << Vz << endl;
        //cout << "yaw = " << yaw << endl;

        //cp->addAttitude(cp->getNowAttitude(), q_q0, q_q1, q_q2, q_q3, Vx, Vy, Vz, yaw);

        //sleep(1);
        //nanosleep(&tim , &tim2);
        //printf("fly status : %d\n", flight->getApi()->getBroadcastData().status);
        if( flight->getApi()->ackdata == 0 && (flight->getApi()->getBroadcastData().status == 1 || flight->getApi()->getBroadcastData().status == 0)){
             char a = '1';
	     //cout << "send to mobile" << endl;
             //flight->getApi()->send(0, 0, SET_ACTIVATION, CODE_TOMOBILE, (unsigned char *)&a, sizeof(a)); //To Mobile
             if(id > 60){
		   flight->getApi()->ackdata = 100;
	           //break;
	     }
        }

        if(flight->getApi()->getBroadcastData().status == NULL){
             flight->getApi()->ackdata = 100;
             //break;
        }
	nanosleep(&tim , &tim2);
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
        cout << "i = " << i++ << endl;

        q_q0 = flight->getQuaternion().q0;
        q_q1 = flight->getQuaternion().q1;
        q_q2 = flight->getQuaternion().q2;
        q_q3 = flight->getQuaternion().q3;

        lat = flight->getPosition().latitude;
        lon = flight->getPosition().longitude;
        height = flight->getPosition().height;
        //cout << "lat = " << lat << ", lon = " << lon << ", height = " << height << endl;

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

void *SelfPrediction(void *ptr)
{
    
}







