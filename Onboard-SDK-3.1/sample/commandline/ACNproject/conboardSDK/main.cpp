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

using namespace std;
using namespace DJI::onboardSDK;

int main(int argc, char *argv[])
{

    /* ---- initial ---- */

    HardDriverManifold driver("/dev/ttyAMA0", 230400);  //利用ttyAMA0來跟無人機溝通，這邊指UART
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


    const Flight *flight = new Flight(&api);  //用以讀取無人機目前狀態
    printf("lat:%.10lf\n",latitude(flight));
    printf("lon:%.10lf\n",longitude(flight));
    printf("Alt:%.10lf\n",altitude(flight));
    printf("Height:%.10lf\n",flight_height(flight));


    /*  ---- flight ---- */

    cout<<"Pass ENTER to goFind() ..."<<endl;fgetc(stdin);
    vector<PointData> record = planPath(&api);  //使用search.h內的planPath開始搜尋
    cout<< "goFind() finish" << endl;

    FILE *log = fopen("./log.txt","w");
    for(int i=0;i<record.size();i++){  //Lat, Lon ,Alti, RSSI, moveDistance, gusLat, gusLon, errDist, time

            printf("%.13lf %.13lf %.13lf %.13lf %.13lf %.13lf %.13lf %.13lf %d\n",record[i].latitude,record[i].longitude,record[i].altitude, record[i].RSSI, record[i].total_moveDist, record[i].guessLatitude, record[i].guessLongitude, record[i].error_dist, record[i].ctimeStamp);
            fprintf(log,"%.13lf %.13lf %.13lf %.13lf %.13lf %.13lf %.13lf %.13lf %d\n",record[i].latitude,record[i].longitude,record[i].altitude,record[i].RSSI, record[i].total_moveDist, record[i].guessLatitude, record[i].guessLongitude, record[i].error_dist, record[i].ctimeStamp);


    }
    fclose(log);


    cout<<"Pass ENTER to exit ..."<<endl;fgetc(stdin);
    return 0;
}
