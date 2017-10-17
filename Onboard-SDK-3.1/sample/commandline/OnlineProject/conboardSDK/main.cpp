#include <iostream>
#include <sstream>
#include <thread>
#include <fstream>
#include <string.h>
#include <time.h>
#include <cmath>

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

int rangeLength = 30;
int rangeWidth = 30;
int trip = 7;

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

typedef struct Waypoint
{
    void* api;
    calculatePath *cp;
    float homeLat, homeLon, homeAlt, targetLat, targetLon, targetAlt;
}Waypoint;

void outputsetting(void);
void *CollectData(void *ptr);
void *CompareData(void *ptr);
void *SelfPrediction(void *ptr);
void *DRD(void *ptr);
void *compareData(void *ptr);
void *Project(void *ptr);

int main(int argc, char *argv[])
{
    char datafilename[100];
    //char firstfile[100];
    //char secondfile[100];

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
    
    //ss << "/var/" << tmp << "-gpsdata.txt";
    //ss >> firstfile;
    //ss.clear();
    //ss << "/var/" << tmp << "-comparedata.txt";
    //ss >> secondfile;
    //ss.clear();
    
    cout << datafilename << endl;
    //cout << firstfile << endl;
    //cout << secondfile<< endl;
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
    //driver.freeMemory();
 
    CoreAPI api(&driver);
    driver.freeMemory();
    
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

    //Takeoff
    taskdata->cmdSequence++;
    taskdata->cmdData = 4;
    for(int i = 0; i < 5; i++){
    	 api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(TaskData), Flight::taskCallback, 100, 3);
	 if(api.ackdata == 2){
		break;
	 }
    }
    sleep(7);
    api.ackdata = 0;
    //Fly Test
    flightdata->flag = 0x40; //HORIZONTAL_VELOCITY
    //flightdata->flag = 0x10; //VERTICAL_POSITION

    for(int i = 0; i < 7; i++){
	 flightdata->x = 0;
         flightdata->y = 0;
         flightdata->z = 0;   
         flightdata->yaw = 0;
         api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
	 sleep(1);
    }

    flightdata->flag = 0x40; //HORIZONTAL_VELOCITY
    //flightdata->flag = 0x10; //VERTICAL_POSITION

    //flight heigt
    for(int i = 0; i < 9; i++){
         flightdata->x = 0;
         flightdata->y = 0;
         flightdata->z = 1;
         flightdata->yaw = 0;
         api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
         sleep(1);
    }

    /*//compareData
    threadData *t1;
    t1 = (threadData*)malloc(sizeof(threadData));
    t1->api = (void*)&api;
    t1->filename = datafilename;
    pthread_t cD;
    int ret;
    ret = pthread_create(&cD, NULL, compareData, (void*)t1);
    if(ret!=0){
         printf ("Create pthread error!\n");
         exit (1);
    }
    else{
         CExit = true;
    }*/

    //Project Thread
    threadData *t2;
    t2 = (threadData*)malloc(sizeof(threadData));
    t2->api = (void*)&api;
    t2->filename = "";
    pthread_t PT;
    int ret;
    ret = pthread_create(&PT, NULL, Project, (void*)t2);
    if(ret!=0){
         printf ("Create pthread error!\n");
         exit (1);
    }
    else{
         CExit = true;
    }
    void *rec;
    pthread_join(PT, &rec);


    //Project Context
    //rangeLength rangeWidth trip
    /*
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 700000000L;

    int velocity = 6; //flight speed
    float tripWidth = rangeWidth/trip;
    flightdata->flag = 0x40;
    for(int i = 0; i < trip; i++){
	int Vx = velocity;
	int Vy = 0;
	int Vz = 0;
	float count = rangeLength / (Vx*0.7);
	for(int j = 0; j < count; j++){
	    if(i%2 != 0){
		Vx = -(velocity);
	    }
	    flightdata->x = Vx;
	    flightdata->y = Vy;
            flightdata->z = Vz;
            flightdata->yaw = 0;
	    api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
	    nanosleep(&tim , &tim2);
	}
	sleep(1);
	Vx = 0;
	Vy = 2;
	Vz = 0;
	count = tripWidth / (Vy*0.7);
	for(int k = 0; k < count; k++){
            flightdata->x = Vx;
            flightdata->y = Vy;
            flightdata->z = Vz;
            flightdata->yaw = 0;
	    api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
	    nanosleep(&tim , &tim2);
	}
	sleep(2);
    }
    int Vx = velocity;
    float count = rangeLength / (Vx*0.7);
    for(int j = 0; j < count; j++){
        if(trip%2 != 0){
            Vx = -(velocity);
        }
        flightdata->x = Vx;
        flightdata->y = 0;
        flightdata->z = 0;
        flightdata->yaw = 0;
        api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
        nanosleep(&tim , &tim2);
    }
    sleep(2);
    
    //Project End
    */

    /* DRD
    char a = '1';
    cout << a << endl;
    double lat = api.getBroadcastData().pos.latitude;
    double lon = api.getBroadcastData().pos.longitude;
    double height = api.getBroadcastData().pos.height;
    //cp->GpsPoint(api, lat, lon, height, lat+0.0002, lon+0.0002, 3);
    Waypoint *point1;
    point1 = (Waypoint*)malloc(sizeof(Waypoint));
    point1->api = (void*)&api;
    point1->cp = cp;
    point1->homeLat = lat;
    point1->homeLon = lon;
    point1->homeAlt = height;
    point1->targetLat = lat + 0.0002;
    point1->targetLon = lon + 0.0002;
    point1->targetAlt = 4;
    pthread_t p1;
    //int ret;
    void *rec; 
    ret = pthread_create(&p1, NULL, DRD, (void*)point1);
    if(ret!=0){
         printf ("Create pthread error!\n");
         exit (1);
    }
    else{
         //Exit = true;
    }

    pthread_join(p1,&rec);
    cout << (int)rec << endl;
    api.send(0, 0, SET_ACTIVATION, CODE_TOMOBILE, (unsigned char *)&a, sizeof(a)); //To Mobile
    sleep(3);

    a = '2';
    cout << a << endl;
    Waypoint *point2;
    //point2 = (Waypoint*)malloc(sizeof(Waypoint));
    //point2->api = (void*)&api;
    //point2->cp = cp;
    point1->homeLat = lat+0.0002;
    point1->homeLon = lon+0.0002;
    point1->homeAlt = 4;
    point1->targetLat = lat + 0.0004;
    point1->targetLon = lon;
    point1->targetAlt = 6;
    //pthread_t p2;
    ret = pthread_create(&p1, NULL, DRD, (void*)point1);
    if(ret!=0){
         printf ("Create pthread error!\n");
         exit (1);
    }
    else{
         //Exit = true;
    }
    cout << "Thread Executing..." << endl;
    pthread_join(p1,&rec);
    cout << (int)rec << endl;
    //cp->GpsPoint(api, lat+0.0002, lon+0.0002, 3, lat+0.0004, lon, 4);
    api.send(0, 0, SET_ACTIVATION, CODE_TOMOBILE, (unsigned char *)&a, sizeof(a)); //To Mobile
    sleep(3);

    a = '3';
    cout << a << endl;
    Waypoint *point3;
    point3 = (Waypoint*)malloc(sizeof(Waypoint));
    point3->api = (void*)&api;
    point3->cp = cp;
    point3->homeLat = lat+0.0004;
    point3->homeLon = lon;
    point3->homeAlt = 6;
    point3->targetLat = lat + 0.0002;
    point3->targetLon = lon - 0.0002;
    point3->targetAlt = 8;
    pthread_t p3;
    ret = pthread_create(&p3, NULL, DRD, (void*)point3);
    if(ret!=0){
         printf ("Create pthread error!\n");
         exit (1);
    }
    else{
         //Exit = true;
    }
    pthread_join(p3,&rec);
    cout << (int)rec << endl;
    //cp->GpsPoint(api, lat+0.0004, lon, 4, lat+0.0002, lon-0.0002, 5);
    api.send(0, 0, SET_ACTIVATION, CODE_TOMOBILE, (unsigned char *)&a, sizeof(a)); //To Mobile
    sleep(3);

    a = '4';
    cout << a << endl;
    Waypoint *point4;
    point4 = (Waypoint*)malloc(sizeof(Waypoint));
    point4->api = (void*)&api;
    point4->cp = cp;
    point4->homeLat = lat+0.0002;
    point4->homeLon = lon-0.0002;
    point4->homeAlt = 8;
    point4->targetLat = lat;
    point4->targetLon = lon;
    point4->targetAlt = 10;
    pthread_t p4;
    ret = pthread_create(&p4, NULL, DRD, (void*)point4);
    if(ret!=0){
         printf ("Create pthread error!\n");
         exit (1);
    }
    else{
         //Exit = true;
    }
    pthread_join(p4,&rec);
    cout << (int)rec << endl;
    //cp->GpsPoint(api, lat+0.0002, lon-0.0002, 5, lat, lon, 6);
    api.send(0, 0, SET_ACTIVATION, CODE_TOMOBILE, (unsigned char *)&a, sizeof(a)); //To Mobile
    sleep(3);
    */
    
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

    CExit = false;
    //sleep(7);
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

void *DRD(void *ptr)   //Calibration Thread
{
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 700000000L;

    Waypoint *point1 = (Waypoint*)ptr;
    CoreAPI *api = (CoreAPI*)(point1->api);
    const Flight *flight = new Flight(api);
    
    Attitude *n = point1->cp->GpsPoint(point1->homeLat, point1->homeLon, point1->homeAlt, point1->targetLat, point1->targetLon, point1->targetAlt);
    Attitude *tmp;
    tmp = (Attitude*)malloc(sizeof(Attitude));
 
    FlightData *flightdata;
    flightdata = (FlightData*)malloc(sizeof(FlightData));
    flightdata->flag = 0x40;

    //cout << "test " << api->getBroadcastData().pos.latitude << ", " << api->getBroadcastData().pos.longitude << endl;

    while(n->nextAttitude != NULL){
	 flightdata->x = n->nextAttitude->speedX;
         flightdata->y = n->nextAttitude->speedY;
         flightdata->z = n->nextAttitude->speedZ;   // velocity
         flightdata->yaw = n->nextAttitude->yaw;
	 if (flightdata->x > 10)     //protection
		  flightdata->x = 10;
	 else if (flightdata->x < -10)
		  flightdata->x = -10;
         if (flightdata->y > 10)
                  flightdata->y = 10;
         else if (flightdata->y < -10)
                  flightdata->y = -10;
         if (flightdata->z > 10)
                  flightdata->z = 10;
         else if (flightdata->z < -10)
                  flightdata->z = -10;

         cout << "set vx = " << flightdata->x << ", vy = " << flightdata->y << ", height = " << flightdata->z << ", yaw = " << flightdata->yaw << endl;
         api->send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));

	 if (n->nextAttitude->nextAttitude == NULL) break;

	 nanosleep(&tim , &tim2);
	 //sleep(1);
	 if(n->nextAttitude->ID == 1 || n->nextAttitude->ID == 0){
		  n->nextAttitude = n->nextAttitude->nextAttitude;
		  continue;
	 }
	 float Vx = flight->getVelocity().x;
         float Vy = flight->getVelocity().y;
         float Vz = flight->getVelocity().z;
	 float DifX = (Vx - n->nextAttitude->speedX);
	 float DifY = (Vy - n->nextAttitude->speedY);
	 float DifZ = (Vz - n->nextAttitude->speedZ);
	 //cout << "In thread: " << Vx << ", " << Vy << ", " << Vz << endl;
	 //cout << "Dif: " << DifX << ", " << DifY << ", " << DifZ << endl;

	 //cout << api->getBroadcastData().pos.latitude << ", " << api->getBroadcastData().pos.longitude << ", " << api->getBroadcastData().pos.altitude << endl; 
         //cout << api->getBroadcastData().v.x << ", " << api->getBroadcastData().v.y << ", " << api->getBroadcastData().v.z << endl;
	 float persent = 0.5;
	 
	 if(fabs(DifX) > fabs(n->nextAttitude->speedX*persent)){
		  n->nextAttitude->nextAttitude->speedX -= DifX;
		  tmp->nextAttitude = n->nextAttitude->nextAttitude;
		  while(tmp->nextAttitude != NULL){
			   tmp->nextAttitude->speedX -= DifX;
			   tmp->nextAttitude = tmp->nextAttitude->nextAttitude;
		  }
	 }
	 if(fabs(DifY) > fabs(n->nextAttitude->speedY*persent)){
                  n->nextAttitude->nextAttitude->speedY -= DifY;
                  tmp->nextAttitude = n->nextAttitude->nextAttitude;
                  while(tmp->nextAttitude != NULL){
                           tmp->nextAttitude->speedY -= DifY;
                           tmp->nextAttitude = tmp->nextAttitude->nextAttitude;
                  }
         }
	 if(fabs(DifZ) > fabs(n->nextAttitude->speedZ*persent)){
                  n->nextAttitude->nextAttitude->speedZ -= DifZ;
                  tmp->nextAttitude = n->nextAttitude->nextAttitude;
                  while(tmp->nextAttitude != NULL){
                           tmp->nextAttitude->speedZ -= DifZ;
                           tmp->nextAttitude = tmp->nextAttitude->nextAttitude;
                  }
         }
	 n->nextAttitude = n->nextAttitude->nextAttitude;
    }
    pthread_exit((void *)1234);
}

void *compareData(void *ptr)
{
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 100000000L;

    CoreAPI *api = (CoreAPI*)((threadData*)ptr)->api;
    const Flight *flight = new Flight(api);

    //CallBackHandler FME;
    //api->setFromMobileCallback(FME);
    //api->ackdata = 0;

    /*
    while(api.ackdata == 0){
        api.send(0, 0, SET_BROADCAST, CODE_FROMMOBILE, (unsigned char *)&(api.ackdata), sizeof(api.ackdata));  //FromMobile
        //cout << api.ackdata << endl;
    }
    printf("\napi.ackdata = %d\n", api.ackdata);*/

    fp.open(((threadData*)ptr)->filename, ios::out);
    if(!fp){
        cout << "Fail to open file: " << ((threadData*)ptr)->filename << endl;
        CExit = false;
    }
    else{
        cout << ((threadData*)ptr)->filename << " open successfully." << endl;
    }
    float lat, lon, height;
    while(CExit){
	 /*while(api->ackdata == 0){
              api->send(0, 0, SET_BROADCAST, CODE_FROMMOBILE, (unsigned char *)&(api->ackdata), sizeof(api->ackdata));  //FromMobile
              //cout << api.ackdata << endl;
    	 }*/
    	 //printf("\napi->ackdata = %d\n", api->ackdata);
         lat = flight->getPosition().latitude;
         lon = flight->getPosition().longitude;
         height = flight->getPosition().height;
	 //float Vx = flight->getVelocity().x;
    	 //float Vy = flight->getVelocity().y;
     	 //float Vz = flight->getVelocity().z; 
    	 //cout << Vx << ", " << Vy << ", " << Vz << endl;

	 fp << "-------------------------" << endl;
	 fp << "latitude = " << lat << endl;
         fp << "longitude = " << lon << endl;
         fp << "altitude = " << flight->getPosition().altitude << endl;
         fp << "height = " << height << endl;

	 nanosleep(&tim , &tim2);
    }
    cout << "compareData Finished" << endl;
    //pthread_exit((void *)123);
}

void *Project(void *ptr)
{
    cout << "Project Start" << endl;
    CoreAPI *api = (CoreAPI*)((threadData*)ptr)->api;
    const Flight *flight = new Flight(api);
    FlightData *flightdata;
    flightdata = (FlightData*)malloc(sizeof(FlightData));

    CallBackHandler FME;
    api->setFromMobileCallback(FME);

    bool home = false;
    double homeLat = 0;
    double homeLon = 0;
    int c = 0;

    //Project Context
    //rangeLength rangeWidth trip
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 700000000L;

    int velocity = 6; //flight speed
    float tripWidth = rangeWidth/trip;
    flightdata->flag = 0x40;
    for(int i = 0; i < trip; i++){
        int Vx = velocity;
        int Vy = 0;
        int Vz = 0;
        float count = rangeLength / (Vx*0.7);
        for(int j = 0; j < count; j++){
            if(i%2 != 0){
                Vx = -(velocity);
            }
            flightdata->x = Vx;
            flightdata->y = Vy;
            flightdata->z = Vz;
            flightdata->yaw = 0;
            api->send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
	    cout << "fly" << endl;
            //nanosleep(&tim , &tim2);

	    //receive from mobile
	    //cout << "Waiting for mobile data";
	    printf("Waiting for mobile data\n");
	    api->ackdata = 0;
	    string ackBuff = "";
	    while(true){
		api->send(0, 0, SET_BROADCAST, CODE_FROMMOBILE, (unsigned char *)&(api->ackdata), sizeof(api->ackdata));  //FromMobile
		char tmp = api->ackdata;
		cout << tmp;
		if(api->ackdata != 64)
		ackBuff += tmp;
	        if(api->ackdata == 47){
		    cout << ackBuff << endl;
		    ackBuff = "";
	            //api->send(0, 0, SET_BROADCAST, CODE_FROMMOBILE, (unsigned char *)&(api->ackdata), sizeof(api->ackdata));  //FromMobile
		    c++;
		    if(c == 3) break;
	        }
	    }
	    c = 0;
	    //printf("\napi->ackdata = %d\n", api->ackdata);
	    nanosleep(&tim , &tim2);
        }
        sleep(1);
        Vx = 0;
        Vy = 2;
        Vz = 0;
        count = tripWidth / (Vy*0.7);
        for(int k = 0; k < count; k++){
            flightdata->x = Vx;
            flightdata->y = Vy;
            flightdata->z = Vz;
            flightdata->yaw = 0;
            api->send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
            nanosleep(&tim , &tim2);
        }
        sleep(2);
    }
    int Vx = velocity;
    float count = rangeLength / (Vx*0.7);
    for(int j = 0; j < count; j++){
        if(trip%2 != 0){
            Vx = -(velocity);
        }
        flightdata->x = Vx;
        flightdata->y = 0;
        flightdata->z = 0;
        flightdata->yaw = 0;
        api->send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
        nanosleep(&tim , &tim2);
    }
    sleep(2);

    //Project End
    pthread_exit(NULL);
}
