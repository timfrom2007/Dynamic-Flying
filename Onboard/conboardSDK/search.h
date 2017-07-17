#include <iostream>
#include <vector>
#include <pthread.h>
#include <time.h>
#include <cmath>

#include "DJI_API.h"
#include "DJI_Type.h"
#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"

using namespace std;
using namespace DJI::onboardSDK;

struct PointData
{
    float64_t latitude = 0;
    float64_t longitude = 0;
    float64_t altitude = 0;
    double RSSI = 0;
    time_t timeStamp = 0;
    clock_t ctimeStamp = 0;
    
};
struct CollectThreadParams
{
    vector<PointData> *record;
    const Flight *flight;
    bool isFlying = true;
};

double latitude(const Flight* flight);
double longitude(const Flight* flight);
double altitude(const Flight* flight);
void control(VirtualRC *vrc ,int pitch,int roll);
PointData calculatePos(vector<PointData> *record); //Pure RSSI
PointData calculatePos2(vector<PointData> *record); //Differential RSSI
PointData calculatePos3(vector<PointData> *record); //Trilateration
PointData calculatePos4(vector<PointData> *record); //Modified Differential RSSI
void takeOff(VirtualRC *vrc);
void *collectRSSI(void *ptr);
vector<PointData> goFind(CoreAPI *api,const char *pathFile);
double getFakeRSSI(const Flight *flight,double la,double lo,double al);
double distance(double lat1, double lon1, double lat2, double lon2, char unit);
double rssiToDist(double rssi, double altitude);
vector<PointData> planPath(CoreAPI *api); //Planning Path
double normalDistribution();
