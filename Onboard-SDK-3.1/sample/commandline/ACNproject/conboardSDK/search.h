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


//紀錄飛行時所經過各節點資訊
struct PointData
{
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double RSSI = 0.0;
    double total_moveDist=0.0;
    double error_dist = 0.0;
    time_t timeStamp = 0;
    clock_t ctimeStamp = 0;
    int startSearch = 0;
    double guessLatitude =0.0;
    double guessLongitude = 0.0;
};

//紀錄飛行時所經過各節點後，根據節點訊息猜測定位位置
struct GuessPosition
{
    float64_t latitude = 0.0;
    float64_t longitude = 0.0;
    double xt=0.0;
    double yt=0.0;
    double threshold=0.0;
};

struct CollectThreadParams
{
    vector<PointData> *record;
    const Flight *flight;
    bool isFlying = true;
};
//中位數濾波，傳入陣列
double median_filter(double* rssi);

//取得無人機緯度。預設小數點後六位，實際可以到15位
double latitude(const Flight* flight);

//取得無人機經度。預設小數點後六位，實際可以到15位
double longitude(const Flight* flight);

//取得無人機高度。 在模擬時可精準取得，然而在實際飛行時會得到負數
double altitude(const Flight* flight);
double flight_height(const Flight* flight);  //取得無人機高度。與altitude不同，在實際飛行時使用這個
double getYaw(const Flight* flight);  //取得目前無人機自身旋轉角度
void control(VirtualRC *vrc ,int pitch,int roll);  //用以控制無人機前後左右移動
void *collectRSSI(void *ptr);  //用以蒐集RSSI
vector<PointData> goFind(CoreAPI *api,const char *pathFile, double distPercent);  //根據所提供飛行腳本，丟入control()執行飛行
double getFakeRSSI(const Flight *flight,double la,double lo,double al,int startSearch);  //根據距離得到一個包含雜訊的RSSI
double earth_distance(double lat1, double lon1, double lat2, double lon2, char unit); //依兩點經緯度計算其距離
double rssiToDist(double rssi, double altitude);  //將RSSI轉換成距離
vector<PointData> planPath(CoreAPI *api); //Planning Path
double normalDistribution();  //給一個常態分布的數值
void rotation_matrix(double currX, double currY, double preX, double preY, vector<double> &r_matrix, double curYaw);  //根據curYaw，無人機自己旋轉角度，
double moveDistance_to_speed(double move_distance);
double distance(int x1, int y1, int x2, int y2);
void addWeight(double startX, double startY, double currX, double currY, double preX, double preY, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix, int height);
double calConstant(double x, double y, double m);
int turnDecision(double startX, double startY, double currX, double currY, double preX, double preY, int* preturn, int* bool_predecision, int* turnCase, double cur_radius, double pre_radius, double** map_weight, int** map_count, vector<double> r_matrix, int height);
//void nextWayPoint(WayPoint waypoint, double lat, double lon);
//void initWayPoint(WayPoint waypoint);
void coordinateChanger(double xt, double yt, vector<double> x_matrix, vector<double> y_matrix, vector<double> lat_matrix, vector<double> lon_matrix);
void flightMove(double* currentX, double* currentY, double* preX, double* preY, int descision, double move_distance, double curYaw);
GuessPosition predictPos(double** map_weight, int** map_count, double currR, double currX, double currY, int height);
double leastSquare(double xyt, vector<double> &xy_matrix, vector<double> &latlon_matrix);
double gaussFilter(vector<double> gauss);
double kalman_filter(double* rssi);
int out4in5(double x);
