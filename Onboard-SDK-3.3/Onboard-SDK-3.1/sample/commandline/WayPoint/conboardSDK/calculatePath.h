#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "DJI_API.h"

using namespace std;

typedef struct flightAttitude
{
    int ID;
    float q0;
    float q1;
    float q2;
    float q3;
    float speedX;
    float speedY;
    float speedZ;
    float yaw;
    //int exeTime;
    struct flightAttitude *nextAttitude;
    struct flightAttitude *preAttitude;
} Attitude;

class calculatePath
{
   public :
          calculatePath();
          void deletePoint(); //smooth
          void deleteRoom(); 
          void addTime(Attitude *now, int exTime);
          void pointTransform(Attitude *now, char filename[]);
          Attitude* getNowAttitude();
          void exeAttitude(Attitude *h, DJI::onboardSDK::CoreAPI api);
          void addAttitude(Attitude *now, float q0, float q1, float q2, float q3, float speedX, float speedY, float speedZ, float yaw);
	  void addAttitude(Attitude *now, float speedX, float speedY, float speedZ, float yaw);
          void showAllAttitude(Attitude *h);
          void freeAttitude(Attitude *h);
	  
	  void exeAttitudeR(Attitude *tail, DJI::onboardSDK::CoreAPI api);
	  void showReverseAttitude(Attitude *n);
	  void SelfPredict(Attitude *tail, DJI::onboardSDK::CoreAPI api);
	  void ResetSpeed(Attitude *pre, float speedXerr, float speedYerr, float speedZerr);
};

