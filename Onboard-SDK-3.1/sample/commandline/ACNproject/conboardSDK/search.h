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
#include <typeinfo>
#include <iomanip>


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
    float64_t latitude = 0.0;
    float64_t longitude = 0.0;
    float64_t altitude = 0.0;
    double RSSI = 0.0;
    double total_moveDist=0.0;
    double error_dist = 0.0;
    time_t timeStamp = 0;
    clock_t ctimeStamp = 0;
    int startSearch = 0;
    float64_t guessLatitude =0.0;
    float64_t guessLongitude = 0.0;
};

struct GuessPosition
{
    float64_t latitude = 0.0;
    float64_t longitude = 0.0;
    int xt;
    int yt;
    double threshold=0.0;
};

struct CollectThreadParams
{
    vector<PointData> *record;
    const Flight *flight;
    bool isFlying = true;
};

double median_filter(double* rssi);
double latitude(const Flight* flight);
double longitude(const Flight* flight);
double altitude(const Flight* flight);
double getYaw(const Flight* flight);
void control(VirtualRC *vrc ,int pitch,int roll);
//PointData calculatePos(vector<PointData> *record); //Pure RSSI
//PointData calculatePos2(vector<PointData> *record); //Differential RSSI
//PointData calculatePos3(vector<PointData> *record); //Trilateration
//PointData calculatePos4(vector<PointData> *record); //Modified Differential RSSI
void takeOff(VirtualRC *vrc);
void *collectRSSI(void *ptr);
vector<PointData> goFind(CoreAPI *api,const char *pathFile, double distPercent);
double getFakeRSSI(const Flight *flight,double la,double lo,double al,int startSearch);
double earth_distance(double lat1, double lon1, double lat2, double lon2, char unit);
double rssiToDist(double rssi, double altitude);
vector<PointData> planPath(CoreAPI *api); //Planning Path
double normalDistribution();
void rotation_matrix(double currX, double currY, double preX, double preY, vector<double> &r_matrix, double curYaw);
double moveDistance_to_speed(double move_distance);
double distance(int x1, int y1, int x2, int y2);
void addWeight(double currX, double currY, double preX, double preY, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix, int height);
double calConstant(double x, double y, double m);
int turnDecision(double currX, double currY, double preX, double preY, int* preturn, int* bool_predecision, int* turnCase, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix, int height);
//void nextWayPoint(WayPoint waypoint, double lat, double lon);
//void initWayPoint(WayPoint waypoint);
void coordinateChanger(double xt, double yt, vector<double> x_matrix, vector<double> y_matrix, vector<double> lat_matrix, vector<double> lon_matrix);
void flightMove(double* currentX, double* currentY, double* preX, double* preY, int descision, double move_distance, double curYaw);
vector<GuessPosition> predictPos(double** map_weight, int** map_count, double currR, double currX, double currY, int height);
double leastSquare(double xyt, vector<double> &xy_matrix, vector<double> &latlon_matrix);
void gaussFilter(vector<PointData> record, int *predictCount);
double kalman_filter(double* rssi);
int out4in5(double x);
