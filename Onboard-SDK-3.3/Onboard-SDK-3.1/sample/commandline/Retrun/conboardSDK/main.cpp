#include <iostream>
#include <sstream>
#include <string.h>
#include <thread>
#include <fstream>

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "DJI_API.h"

#include "calculatePath.h"
#include "quaternion.h"

using namespace std;
using namespace DJI::onboardSDK;

bool Exit = false;

Attitude *head;

char filename[]="./data.txt";
fstream fp;

void *CollectData(void *ptr)
{
    //char filename[]="./data.txt";
    //fstream fp;
    fp.open(filename, ios::out);
    if(!fp){
        cout << "Fail to open file: " << filename << endl;
    }
    else{
	cout << "File open successfully." << endl;
    }

    const Flight *flight = new Flight((CoreAPI*)ptr);
    int i = 0;

    calculatePath *cp = new calculatePath();
    cp->getNowAttitude()->nextAttitude = head;

    float q_q0, q_q1, q_q2, q_q3;
    float Vx, Vy, Vz, yaw;
    float lat, lon, height;
    
    while(Exit)
    {
	//cout << "In thread" << endl;
	fp << "i = " << i << "-------------------------" << endl;
	cout << "i = " << i++ << endl;

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

    	fp << "q0 = " << q_q0 << endl;
    	fp << "q1 = " << q_q1 << endl;
    	fp << "q2 = " << q_q2 << endl;
    	fp << "q3 = " << q_q3 << endl;

    	fp << "latitude = " << lat << endl;
    	fp << "longitude = " << lon << endl;
    	//fp << "altitude = " << flight->getPosition().altitude << endl;
    	fp << "height = " << height << endl;
    	//printf("health = %d\n", flight->getPosition().health);

	//cout << "Time = " << flight->getApi()->getTime().time << endl;
	//printf("Time = %d\n nanoTime = %d\nsyncFlag = %d\n", flight->getApi()->getTime().time, flight->getApi()->getTime().nanoTime, flight->getApi()->getTime().syncFlag);

    	fp << "Vx = " << Vx << endl;
    	fp << "Vy = " << Vy << endl;
    	fp << "Vz = " << Vz << endl;
	fp << "yaw = " << yaw << endl;

	if(yaw <= 180 && yaw >= -180){	
	     cp->addAttitude(cp->getNowAttitude(), q_q0, q_q1, q_q2, q_q3, Vx, Vy, Vz, yaw);
	}

	sleep(1);
    }
    //fp.close();

}

int main(int argc, char *argv[])
{
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

    //setcontrol
    unsigned char enable = 1;
    //api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);
    //sleep(5);
    
    pthread_t id;
    int ret;
    ret = pthread_create(&id, NULL, CollectData, (void*)&api);
    if(ret!=0){
         printf ("Create pthread error!\n");
         exit (1);
    }
    else{
	Exit = true;
    }

    CallBackHandler FME;
    api.setFromMobileCallback(FME);
    api.ackdata = 0;
    while(api.ackdata == 0){
	api.send(0, 0, SET_BROADCAST, CODE_FROMMOBILE, (unsigned char *)&(api.ackdata), sizeof(api.ackdata));  //FromMobile
    }
    Exit = false;
    fp.close();
    //pthread_exit(&id);
    printf("\napi.ackdata = %d\n", api.ackdata);
    sleep(5);

    cout << "Set control" << endl;
    api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);
    sleep(5);

    /*
    int exe;
    do{
	cout << "\n4:Take off, 22:Flight control" << endl;
	cin >> exe;
	switch(exe)
	{
		case 4:
			taskdata->cmdData = 4;
                        taskdata->cmdSequence++;
                        api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(*taskdata), Flight::taskCallback, 100, 3);
                        break;
		case 22:
			float32_t x, y, z, yaw;

                        if( flightdata->flag == 0){
                            flightdata->flag = 0x40;
                            printf("flag = %d\n", flightdata->flag);
                        }

                        if(flightdata->flag == 0x90)
                        {
                            cout << "x(m): ";
                            cin >> x;
                            cout << "y(m): ";
                            cin >> y;
                            cout << "z(m): ";
                            cin >> z;
                            cout << "yaw(+-180): ";
                            cin >> yaw;
                        }
                        else if(flightdata->flag == 0x40)
                        {
                            cout << "x(+-10m/s): ";
                            cin >> x;
                            cout << "y(+-10m/s): ";
                            cin >> y;
                            cout << "z(+-4m/s): ";
                            cin >> z;
                            cout << "yaw(+-180): ";
                            cin >> yaw;
                        }
                        flightdata->x = x;
                        flightdata->y = y;
                        flightdata->z = z;
                        flightdata->yaw = yaw;
                        api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData), Flight::taskCallback, 100, 3);
                        break;
	}
    }while(exe);*/

    //cp->showReverseAttitude(cp->getNowAttitude()->nextAttitude);

    cp->exeAttitudeR(cp->getNowAttitude(), api);

    cp->freeAttitude(head);
    //free(cp);
    //free(head);
    //pthread_exit(&id);

    return 0;
}
