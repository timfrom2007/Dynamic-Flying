#include <iostream>
#include <sstream>
#include <string.h>
#include <thread>
#include <fstream>
#include <pthread.h>

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "DJI_API.h"
#include "DJI_Type.h"

#include "search.h"
#include "rssi.h"

using namespace std;
using namespace DJI::onboardSDK;

int main(int argc, char *argv[])
{

    /* ---- initial ---- */ 

    HardDriverManifold driver("/dev/ttyAMA0", 230400);
    driver.init();
    CoreAPI api(&driver);
    APIThread send(&api, 1);
    APIThread read(&api, 2);
    send.createThread();
    read.createThread();
    api.ackdata = 0;
    ActivateData *data;
    data = (ActivateData *)malloc(sizeof(ActivateData));
    data->version = SDK_VERSION;
    data->ID = 1026279;
    data->encKey = "bba4b69aaef4bba3ebb38806cce9575ba2b43676d93cfc4fe7bf8a8e11b3e850";
    data->reserved = 2;
    api.activate(data);
    unsigned char enable = 1;
    api.send(2, 0, SET_CONTROL, CODE_SETCONTROL, &enable, 1, CoreAPI::setControlCallback, 500, 2);
    sleep(2);

    /* ---- initial ---- */


    const Flight *flight = new Flight(&api);
    printf("lat:%lf\n",latitude(flight));
    printf("lon:%lf\n",longitude(flight));

    //double a,b;
    //cin>>a>>b;
    //cout<<getFakeRSSI(flight,a,b,0);

    /*  ---- flight ---- */ 

    cout<<"Pass ENTER to goFind() ..."<<endl;fgetc(stdin);
    vector<PointData> record = planPath(&api);
    cout<< "goFind() finish" << endl;

    FILE *log = fopen("./log.txt","w");
    for(int i=0;i<record.size();i++){
	printf("%lf %lf %lf %lf\n",record[i].latitude,record[i].longitude,record[i].altitude,record[i].RSSI);
	//cout<<record[i].timeStamp<<endl;
	fprintf(log,"%lf %lf %lf %lf\n",record[i].latitude,record[i].longitude,record[i].altitude,record[i].RSSI);
    }
    fclose(log);
    PointData p = calculatePos(&record);
    printf("lat:%lf lon:%lf\n",p.latitude,p.longitude);
    p = calculatePos2(&record);
    printf("lat:%lf lon:%lf\n",p.latitude,p.longitude);
    p =calculatePos3(&record);
    printf("lat:%lf lon:%lf\n",p.latitude,p.longitude);
    p =calculatePos4(&record);
    printf("lat:%lf lon:%lf\n",p.latitude,p.longitude);
 
    /* ---- flight ----  */


    cout<<"Pass ENTER to exit ..."<<endl;fgetc(stdin);
    return 0;
}
