#include <iostream>

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "DJI_API.h"

using namespace std;
using namespace DJI::onboardSDK;

void getData(CoreAPI api)
{
    cout << "q0 = " << api.getBroadcastData().q.q0 << endl;
    cout << "q1 = " << api.getBroadcastData().q.q1 << endl;
    cout << "q2 = " << api.getBroadcastData().q.q2 << endl;
    cout << "q3 = " << api.getBroadcastData().q.q3 << endl;

    cout << "latitude = " << api.getBroadcastData().pos.latitude << endl;
    cout << "longitude = " << api.getBroadcastData().pos.longitude << endl;
    cout << "altitude = " << api.getBroadcastData().pos.altitude << endl;
    cout << "height = " << api.getBroadcastData().pos.height << endl;
    cout << "health = " << api.getBroadcastData().pos.health << endl;
}

int main(int argc, char *argv[])
{
    HardDriverManifold driver("/dev/ttyAMA0", 230400);
    driver.init();
    
    CoreAPI api(&driver);
    
    ActivateData *data;
    data = (ActivateData *)malloc(sizeof(ActivateData));
        
    data->version = SDK_VERSION;
    data->ID = 1026279;
    char *key = "bba4b69aaef4bba3ebb38806cce9575ba2b43676d93cfc4fe7bf8a8e11b3e850";
    data->encKey = key;
    data->reserved = 2;
    
    //activate
    //api.activate(&(*data));
    
    API_LOG(&driver, STATUS_LOG, "LOG Test\n");
    //CoreAPI::send(2, 0, SET_ACTIVATION, CODE_ACTIVATE, (unsigned char *)& data, sizeof(data) - sizeof(char *), CoreAPI::activateCallback, 1000, 3);
    api.send(2, 0, SET_ACTIVATION, CODE_ACTIVATE, (unsigned char *)&(*data), sizeof(*data) - sizeof(char *), CoreAPI::activateCallback, 1000, 3);
        
    //setcontrol
    unsigned char enable = 1;
    api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);
    //api.setControl(true);
     
    //char* inputCmd = new char[512];
    //cin.getline(inputCmd, 512);
    
    int num;
    //bool out = true;

    TaskData *taskdata;
    taskdata = (TaskData*)malloc(sizeof(TaskData));

    FlightData *flightdata;
    flightdata = (FlightData*)malloc(sizeof(FlightData));

    do {
    	cout << "Enter an action (0:exit, 1:Go home, 4:Take off, 6:Landing, 11:Angle of gimbal, 22:Flight control, 33:Get data)" << endl;
	cin >> num;

	//Flight flight;
        //TaskData *taskdata;
	//taskdata = (TaskData*)malloc(sizeof(TaskData));
	switch(num){
		case 0:
			//out = false;
			api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(*taskdata), Flight::taskCallback, 100, 3);
			break;
		case 1: //Flight flight;
			//TaskData *taskdata;
			taskdata->cmdData = 1;
                	taskdata->cmdSequence++;
			api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(*taskdata), Flight::taskCallback, 100, 3);
			break;
		case 4: //Flight flight;
                        //TaskData *taskdata;
			taskdata->cmdData = 4;
                        taskdata->cmdSequence++;
			api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(*taskdata), Flight::taskCallback, 100, 3);
                        break;
		case 6: //Flight flight;
                        //TaskData *taskdata;
			taskdata->cmdData = 6;
                        taskdata->cmdSequence++;
			api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskdata), sizeof(*taskdata), Flight::taskCallback, 100, 3);
                        break;
		case 11:
			GimbalAngleData a;  //work
			cin >> a.yaw >> a.roll >> a.pitch >> a.duration; 
			a.mode = 1;
			//cout << a.yaw << a.roll << a.pitch << a.duration << endl;
			api.send(0, 0, SET_CONTROL, Camera::CODE_GIMBAL_ANGLE, (unsigned char *)& a, sizeof(GimbalAngleData));
			break;
		case 22:
			//Flight flight;
			//FlightData data;
			//data = (FlightData*)malloc(sizeof(data));
			float32_t x, y, z, yaw;

			//cout << "flag : ( 0 x (0,1248) (82,0))";
			//int flag;
			//cin >> oct >> flag;
			//flightdata->flag = flag;
			flightdata->flag = 0x00;

			cout << "x : ";
			cin >> x;
			cout << "y : ";
			cin >> y;
			cout << "z : ";
			cin >> z;
			cout << "yaw : ";
			cin >> yaw;
			
			flightdata->x = x;
                        flightdata->y = y;
                        flightdata->z = z;
                        flightdata->yaw = yaw;
			//flight.setFlight(&data);
			api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData), Flight::taskCallback, 100, 3);
			break;
		case 33:
			//getData(api);

			Flight *flight = new Flight();
			flight->setApi(&api);
			QuaternionData q = flight->getQuaternion();

			cout << "q0 = " << q.q0 << endl;
                        cout << "q1 = " << q.q1 << endl;
                        cout << "q2 = " << q.q2 << endl;
                        cout << "q3 = " << q.q3 << endl;

			printf("q0 = %f\nq1 = %f\nq2 = %f\nq3 = %f\n", q.q0, q.q1, q.q2, q.q3);

			//cout << "q0 = " << api.getBroadcastData().q.q0 << endl;
			//cout << "q1 = " << api.getBroadcastData().q.q1 << endl;
			//cout << "q2 = " << api.getBroadcastData().q.q2 << endl;
			//cout << "q3 = " << api.getBroadcastData().q.q3 << endl;

			cout << "latitude = " << api.getBroadcastData().pos.latitude << endl;
			cout << "longitude = " << api.getBroadcastData().pos.longitude << endl;
			cout << "altitude = " << api.getBroadcastData().pos.altitude << endl;
			cout << "height = " << api.getBroadcastData().pos.height << endl;
			cout << "health = " << api.getBroadcastData().pos.health << endl;

			VelocityData vd = flight->getVelocity();
			cout << "Vx = " << vd.x << endl;
                        cout << "Vy = " << vd.y << endl;
                        cout << "Vz = " << vd.z << endl;

			cout << "battery = " << api.getBatteryCapacity() << endl;
			printf("%d\n", api.getBatteryCapacity());

			break;
		//default:break;
	}

    }while(num);
    

    return 0;
}
