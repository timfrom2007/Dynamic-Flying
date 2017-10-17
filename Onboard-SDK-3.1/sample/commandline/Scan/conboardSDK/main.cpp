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

bool Exit = false;

void *CollectData(void *ptr)
{
    char filename[]="./data.txt";
    fstream fp;
    fp.open(filename, ios::out);
    if(!fp){
        cout << "Fail to open file: " << filename << endl;
    }
    else {
	cout << "Open file successfully" << endl;
    }

    const Flight *flight = new Flight((CoreAPI*)ptr);
    int i = 0;
    
    while(Exit)
    {
	fp << "i = " << i << "-------------------------" << endl;
	i++;
    	fp << "q0 = " << flight->getQuaternion().q0 << endl;
    	fp << "q1 = " << flight->getQuaternion().q1 << endl;
    	fp << "q2 = " << flight->getQuaternion().q2 << endl;
    	fp << "q3 = " << flight->getQuaternion().q3 << endl;

    	fp << "latitude = " << flight->getPosition().latitude << endl;
    	fp << "longitude = " << flight->getPosition().longitude << endl;
    	//fp << "altitude = " << flight->getPosition().altitude << endl;
    	fp << "height = " << flight->getPosition().height << endl;
    	//printf("health = %d\n", flight->getPosition().health);

    	fp << "Vx = " << flight->getVelocity().x << endl;
    	fp << "Vy = " << flight->getVelocity().y << endl;
    	fp << "Vz = " << flight->getVelocity().z << endl;

	fp << "yaw = " << flight->getApi()->getBroadcastData().rc.yaw << endl;

    	//printf("battery = %d\n", api.getBatteryCapacity());
	//printf("battery = %d\n", ((CoreAPI*)ptr)->getBatteryCapacity());

        //printf("%d, %d\n", api.getBroadcastData().status, api.getFlightStatus());
	
    	switch(((CoreAPI*)ptr)->getBroadcastData().status)
    	{
        	case 1:
                	fp << "stand by" << endl;
                	break;
	        case 2:
	                fp << "take off" << endl;
        	        break;
	        case 3:
        	        fp << "in air" << endl;
                	break;
	        case 4:
        	        fp << "landing" << endl;
                	break;
	        case 5:
        	        fp << "finish landing" << endl;
	                break;
 	}
	//fp << "---------------------------------------------" << endl;
	sleep(1);
    }
    fp.close();

}

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
    //api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);
    //sleep(5);
 
    //api.send(0, 0, SET_BROADCAST, CODE_TOMOBILE, (unsigned char *)&i, sizeof(i));  //SendToMobile

    while(api.ackdata == 0){
	api.send(0, 0, SET_BROADCAST, CODE_FROMMOBILE, (unsigned char *)&(api.ackdata), sizeof(api.ackdata));  //FromMobile
    }
    printf("\napi.ackdata = %d\n", api.ackdata);
    sleep(5);

    cout << "\nSet control" << endl;
    api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);  //setControl
    sleep(5);
   
    pthread_t id;
    int ret; 
    ret = pthread_create(&id, NULL, CollectData, (void*)&api);
    if(ret!=0){
         printf ("Create pthread error!\n");
         exit (1);
    }
    else{
	cout << "Create pthread successfully!" << endl;
	Exit = true;
    }

    cout << "\ntakeoff" << endl;
    taskdata->cmdData = 4;
    taskdata->cmdSequence++;
    //for(int i = 0; i < 5; i++){
        api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(*taskdata), Flight::taskCallback, 1000, 5); //takeoff
    //}
    sleep(10);

    cout << "\nRaising" << endl;
    flightdata->flag = 0x90;
    for(int i = 0; i < 10; i++){
	flightdata->x = 0;
        flightdata->y = 0;
        flightdata->z = 10;
        flightdata->yaw = 0;
        api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
        sleep(1);
    }
    sleep(10);

    cout << "\nMisson start" << endl;
    flightdata->flag = 0x40;
    for(int i = 0; i < 5; i++){
	for(int j = 0; j < 10; j++){
	    flightdata->x = 10;
            flightdata->y = 0;
            flightdata->z = 0;
            flightdata->yaw = 0;
            api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
	    sleep(1);
	}

	for(int j = 0; j < 2; j++){
            flightdata->x = 0;
            flightdata->y = 10;
            flightdata->z = 0;
            flightdata->yaw = 0;
            api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
            sleep(1);
        }

	for(int j = 0; j < 10; j++){
            flightdata->x = -10;
            flightdata->y = 0;
            flightdata->z = 0;
            flightdata->yaw = 0;
            api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
            sleep(1);
        }

	for(int j = 0; j < 2; j++){
            flightdata->x = 0;
            flightdata->y = 10;
            flightdata->z = 0;
            flightdata->yaw = 0;
            api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
            sleep(1);
        }
    }
    cout << "Mission finished" << endl;
    Exit = false;
    
    sleep(10);
    
    cout << "\nLanding" << endl;
    taskdata->cmdData = 6;
    taskdata->cmdSequence++;
    api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(*taskdata), Flight::taskCallback, 1000, 5); //Landing
    sleep(10);

    return 0;
}
