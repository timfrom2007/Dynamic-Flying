#include <iostream>
#include <vector>
#include <pthread.h>
#include <time.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iwlib.h>
#include <algorithm>

#include "DJI_API.h"
#include "DJI_Type.h"
#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "DJI_WayPoint.h"

using namespace std;
using namespace DJI::onboardSDK;

struct PointData
{
    float64_t latitude = 0;
    float64_t longitude = 0;
    float64_t altitude = 0;
    double RSSI = 0.0;
    time_t timeStamp = 0;
    clock_t ctimeStamp = 0;
    int startSearch = 0;

};
struct CollectThreadParams
{
    vector<PointData> *record;
    const Flight *flight;
    bool isFlying = true;
};

double median_filter(int* rssi);
double latitude(const Flight* flight);
double longitude(const Flight* flight);
double altitude(const Flight* flight);
void control(VirtualRC *vrc ,int pitch,int roll);
//PointData calculatePos(vector<PointData> *record); //Pure RSSI
//PointData calculatePos2(vector<PointData> *record); //Differential RSSI
//PointData calculatePos3(vector<PointData> *record); //Trilateration
//PointData calculatePos4(vector<PointData> *record); //Modified Differential RSSI
void takeOff(VirtualRC *vrc);
void *collectRSSI(void *ptr);
vector<PointData> goFind(CoreAPI *api,const char *pathFile);
double getFakeRSSI(const Flight *flight,double la,double lo,double al,int startSearch);
double earth_distance(double lat1, double lon1, double lat2, double lon2, char unit);
double rssiToDist(double rssi, double altitude);
vector<PointData> planPath(CoreAPI *api); //Planning Path
double normalDistribution();
void rotation_matrix(double currX, double currY, double preX, double preY, vector<double> &r_matrix);
double moveDistance_to_speed(double move_distance);
double distance(int x1, int y1, int x2, int y2);
void addWeight(double currX, double currY, double preX, double preY, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix);
double calConstant(double x, double y, double m);
int turnDecision(double currX, double currY, double preX, double preY, int* preturn, int* bool_predecision, int* turnCase, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix);
void nextWayPoint(WayPoint waypoint, double lat, double lon);
void initWayPoint(WayPoint waypoint);
//double coordinateChanger(double xt, double yt, double xa, double ya, double xb, double yb, vector<PointData> *preRecord, vector<PointData> *curRecord);
void flightMove(double* currentX, double* currentY, double* preX, double* preY, int turnCases, int descision, double move_distance);
double median_filter(int* rssi);
