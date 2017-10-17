#include <iostream>
#include <sstream>
#include <string.h>
#include <thread>
#include <fstream>

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "DJI_API.h"

using namespace std;
using namespace DJI::onboardSDK;

bool Exit = false;

CoreAPI api;
FlightData *flightdata;
//flightdata = (FlightData*)malloc(sizeof(FlightData));

char filename[]="./data.txt";

void ReTurn(void)
{
    cout << "Start return..." << endl;
    fstream fin;
    fin.open(filename, ios::in);
    if(!fin){
        cout << "Fail to open file: " << filename << endl;
    }
    else cout << "File open successfully." << endl;
    string s;
    int i = 0;

    //FlightData *flightdata;
    flightdata = (FlightData*)malloc(sizeof(FlightData));
    flightdata->flag = 0x40;
    
    while(getline(fin,s))
    {
	cout << s << endl;
	//int id = atoi((s.substr(5,6)).c_str());
	int id = i++;
	cout << "id = " << id << endl;
	
	getline(fin,s);
	float latitude = atof((s.substr(11)).c_str());
	cout << "latitude = " << latitude << endl;

	getline(fin,s);
        float longitude = atof((s.substr(12)).c_str());
	cout << "longitude = " << longitude << endl;

	getline(fin,s);
        float height = atof((s.substr(10)).c_str());
	cout << "height = " << height <<endl;

	getline(fin,s);
        float Vx = atof((s.substr(6)).c_str());
        cout << "Vx = " << Vx <<endl;
	
	getline(fin,s);
        float Vy = atof((s.substr(6)).c_str());
        cout << "Vy = " << Vy <<endl;
	
	getline(fin,s);
        float Vz = atof((s.substr(6)).c_str());
        cout << "Vz = " << Vz <<endl;

	getline(fin,s);
        float yaw = atof((s.substr(7)).c_str());
        cout << "yaw = " << yaw <<endl;

	cout << "set fly" << endl;
	flightdata->x = Vx;
	cout << "flightdata->x = " << flightdata->x << endl;
	cout << "size of x : " << sizeof(flightdata->x) << endl;
        flightdata->y = Vy;
	cout << "flightdata->y = " << flightdata->y << endl;
        flightdata->z = Vz;
	cout << "flightdata->z = " << flightdata->z << endl;
        flightdata->yaw = yaw;
	cout << "flightdata->yaw = " << flightdata->yaw << endl;
        if (id != 0) {
		api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData), Flight::taskCallback, 100, 3);
	}
	cout << "set over" << endl;

	getline(fin,s);
	cout << s <<endl;

	sleep(1);
    }
    
}

void *CollectData(void *ptr)
{
    //char filename[]="./data.txt";
    fstream fp;
    fp.open(filename, ios::out);
    if(!fp){
        cout << "Fail to open file: " << filename << endl;
    }

    const Flight *flight = new Flight((CoreAPI*)ptr);
    int i = 0;
    
    while(Exit)
    {
	fp << "i = " << i << "-------------------------" << endl;
	i++;
    	//fp << "q0 = " << flight->getQuaternion().q0 << endl;
    	//fp << "q1 = " << flight->getQuaternion().q1 << endl;
    	//fp << "q2 = " << flight->getQuaternion().q2 << endl;
    	//fp << "q3 = " << flight->getQuaternion().q3 << endl;

    	//fp << "latitude = " << flight->getPosition().latitude << endl;
    	//fp << "longitude = " << flight->getPosition().longitude << endl;
    	//fp << "altitude = " << flight->getPosition().altitude << endl;
    	//fp << "height = " << flight->getPosition().height << endl;
    	//printf("health = %d\n", flight->getPosition().health);

    	fp << "Vx = " << flight->getVelocity().x << endl;
    	fp << "Vy = " << flight->getVelocity().y << endl;
    	fp << "Vz = " << flight->getVelocity().z << endl;

	fp << "yaw = " << api.getBroadcastData().rc.yaw << endl;

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
	//sleep(1);
    }
    fp.close();

}

void collectData(DJI::onboardSDK::CoreAPI api)
{
    const Flight *flight = new Flight(&api);
    //flight = new Flight(&api);

    cout << "q0 = " << flight->getQuaternion().q0 << endl;
    cout << "q1 = " << flight->getQuaternion().q1 << endl;
    cout << "q2 = " << flight->getQuaternion().q2 << endl;
    cout << "q3 = " << flight->getQuaternion().q3 << endl;

    cout << "latitude = " << flight->getPosition().latitude << endl;
    cout << "longitude = " << flight->getPosition().longitude << endl;
    cout << "altitude = " << flight->getPosition().altitude << endl;
    cout << "height = " << flight->getPosition().height << endl;
    printf("health = %d\n", flight->getPosition().health);

    cout << "Vx = " << flight->getVelocity().x << endl;
    cout << "Vy = " << flight->getVelocity().y << endl;
    cout << "Vz = " << flight->getVelocity().z << endl;

    printf("battery = %d\n", api.getBatteryCapacity());

    //printf("%d, %d\n", api.getBroadcastData().status, api.getFlightStatus());
    switch(api.getBroadcastData().status)
    {
	case 1:
		cout << "stand by" << endl;
		break;
        case 2:
		cout << "take off" << endl;
                break;
        case 3:
		cout << "in air" << endl;
                break;
        case 4:
		cout << "landing" << endl;
                break;
        case 5:
		cout << "finish landing" << endl;
                break;
    }
}

int main(int argc, char *argv[])
{
    HardDriverManifold driver("/dev/ttyAMA0", 230400);
    driver.init();
    
    CoreAPI api(&driver);
    //api.CoreAPI(&driver);
    
    ConboardSDKScript sdkScript(&api);
    ScriptThread st(&sdkScript);

    APIThread send(&api, 1);
    APIThread read(&api, 2);
    send.createThread();
    read.createThread();
    
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
    //api.send(2, 0, SET_ACTIVATION, CODE_ACTIVATE, (unsigned char *)&(*data), sizeof(*data) - sizeof(char *), CoreAPI::activateCallback, 1000, 3);

    //setcontrol
    unsigned char enable = 1;
    api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);
    //api.setControl(true);

    int num;
    
    TaskData *taskdata;
    taskdata = (TaskData*)malloc(sizeof(TaskData));

    //FlightData *flightdata;
    flightdata = (FlightData*)malloc(sizeof(FlightData));

    //thread mThread(test, 10);
    //thread mThread(collectData, api);
    
    do {
    	printf("Enter an action (0:exit, 1:Go home, 4:Take off, 6:Landing, 10:Free memory\\Lock thread,\n11:Angle of gimbal, 21:Change flight flag, 22:Flight control, 33:Get data, 35:Start thread, 44:Return)\n");
	cin >> num;

	switch(num){
		case 0:
			//api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(*taskdata), Flight::taskCallback, 100, 3);
			break;
		case 1:
			taskdata->cmdData = 1;
                	taskdata->cmdSequence++;
			api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(*taskdata), Flight::taskCallback, 100, 3);
			break;
		case 4:
			taskdata->cmdData = 4;
                        taskdata->cmdSequence++;
			api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(*taskdata), Flight::taskCallback, 100, 3);
                        break;
		case 6:
			taskdata->cmdData = 6;
                        taskdata->cmdSequence++;
			api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(*taskdata), Flight::taskCallback, 100, 3);
                        break;
		case 10:
			driver.freeMemory();
			if(Exit)
			{
			    Exit = false;
			    //pthread_exit(&id);
			}
			else Exit = true;
			break;
		case 11:
			GimbalAngleData a;  //work
			cin >> a.yaw >> a.roll >> a.pitch >> a.duration; 
			a.mode = 1;
			api.send(0, 0, SET_CONTROL, Camera::CODE_GIMBAL_ANGLE, (unsigned char *)& a, sizeof(GimbalAngleData));
			break;
		case 21:
			cout << "flag = ";
			int flag;
			cin >> flag;
			if(flag == 1){
			    flightdata->flag = 0x90;
			}
			else {
			    flightdata->flag = 0x40;
			}
			break;
		case 22:
			float32_t x, y, z, yaw;

			if( flightdata->flag == 0){
			    //flightdata->flag = 0x90;
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
		case 33:
			collectData(api);
			break;
		case 35:
			pthread_t id;
    			int ret;
    			ret = pthread_create(&id, NULL, CollectData, (void*)&api);
    			if(ret!=0){
        			printf ("Create pthread error!\n");
       	 			exit (1);
    			}
			//pthread_exit(&id);
			break;
		case 44:
			ReTurn();
			break;
		//default:break;
	}

    }while(num);
    //pthread_exit(&id);

    return 0;
}
