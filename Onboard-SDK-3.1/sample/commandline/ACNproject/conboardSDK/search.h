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


//��������ɩҸg�L�U�`�I��T
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

//��������ɩҸg�L�U�`�I��A�ھڸ`�I�T���q���w���m
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
//������o�i�A�ǤJ�}�C
double median_filter(double* rssi);

//���o�L�H���n�סC�w�]�p���I�᤻��A��ڥi�H��15��
double latitude(const Flight* flight);

//���o�L�H���g�סC�w�]�p���I�᤻��A��ڥi�H��15��
double longitude(const Flight* flight);

//���o�L�H�����סC �b�����ɥi��Ǩ��o�A�M�Ӧb��ڭ���ɷ|�o��t��
double altitude(const Flight* flight);
double flight_height(const Flight* flight);  //���o�L�H�����סC�Paltitude���P�A�b��ڭ���ɨϥγo��
double getYaw(const Flight* flight);  //���o�ثe�L�H���ۨ����ਤ��
void control(VirtualRC *vrc ,int pitch,int roll);  //�ΥH����L�H���e�ᥪ�k����
void *collectRSSI(void *ptr);  //�ΥH�`��RSSI
vector<PointData> goFind(CoreAPI *api,const char *pathFile, double distPercent);  //�ھکҴ��ѭ���}���A��Jcontrol()���歸��
double getFakeRSSI(const Flight *flight,double la,double lo,double al,int startSearch);  //�ھڶZ���o��@�ӥ]�t���T��RSSI
double earth_distance(double lat1, double lon1, double lat2, double lon2, char unit); //�̨��I�g�n�׭p���Z��
double rssiToDist(double rssi, double altitude);  //�NRSSI�ഫ���Z��
vector<PointData> planPath(CoreAPI *api); //Planning Path
double normalDistribution();  //���@�ӱ`�A�������ƭ�
void rotation_matrix(double currX, double currY, double preX, double preY, vector<double> &r_matrix, double curYaw);  //�ھ�curYaw�A�L�H���ۤv���ਤ�סA
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
