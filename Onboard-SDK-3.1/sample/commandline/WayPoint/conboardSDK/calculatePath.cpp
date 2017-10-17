#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "DJI_API.h"

#include "calculatePath.h"

using namespace std;

Attitude *now;
int recordNum = 3;
int curID = 0;

calculatePath::calculatePath()
{
     now = (Attitude*)malloc(sizeof(Attitude));
     now->ID = -1;
     now->nextAttitude = NULL;
     now->preAttitude = NULL;
}

void calculatePath::deletePoint() //smooth
{
     /*
     
     for(int i = 0; i < recordNum; i++){
         
     }
     */
}

void calculatePath::deleteRoom()
{
     
}

void calculatePath::pointTransform(Attitude *now, char filename[])  //data transform to point
{
     cout << "pointTransform..." << endl;
     //input record data
     fstream fin;
     fin.open(filename, ios::in);
     if(!fin){
         cout << "Fail to open file: " << filename << endl;
     }
     else{
	cout << "File open successfully." << endl;
     }

     string s;
     int i = 0;
     while(getline(fin,s))
     {
         //cout << s << endl;
         int id = i++;
         //cout << "id = " << id << endl;

	 getline(fin, s);
	 float q0 = atof((s.substr(5)).c_str());

	 getline(fin, s);
         float q1 = atof((s.substr(5)).c_str());

	 getline(fin, s);
         float q2 = atof((s.substr(5)).c_str());

	 getline(fin, s);
         float q3 = atof((s.substr(5)).c_str());
 
         getline(fin,s);
         float latitude = atof((s.substr(11)).c_str());
         //cout << "latitude = " << latitude << endl;

         getline(fin,s);
         float longitude = atof((s.substr(12)).c_str());
         //cout << "longitude = " << longitude << endl;

         getline(fin,s);
         float height = atof((s.substr(10)).c_str());
         //cout << "height = " << height <<endl;

         getline(fin,s);
	 //cout << s << endl;
         float Vx = atof((s.substr(5)).c_str());
         //cout << "Vx = " << Vx <<endl;

         getline(fin,s);
	 //cout << s << endl;
         float Vy = atof((s.substr(5)).c_str());
         //cout << "Vy = " << Vy <<endl;

         getline(fin,s);
	 //cout << s << endl;
         float Vz = atof((s.substr(5)).c_str());
         //cout << "Vz = " << Vz <<endl;

	 getline(fin,s);
	 //cout << s << endl;
         float yaw = atof((s.substr(6)).c_str());
         //cout << "yaw = " << yaw << endl; 
 
         //getline(fin,s);
         //cout << s <<endl;

	 if(yaw <= 180 && yaw >= -180){	
	      addAttitude(now, q0, q1, q2, q3, Vx, Vy, Vz, yaw);
	 }
	 sleep(1);
     }
}

void calculatePath::addAttitude(Attitude *now, float q0, float q1, float q2, float q3, float speedX, float speedY, float speedZ, float yaw)
{
     Attitude *newAttitude;
     newAttitude = (Attitude*)malloc(sizeof(Attitude));
     now->nextAttitude->nextAttitude = newAttitude;
     newAttitude->preAttitude = now->nextAttitude;

     newAttitude->ID = curID++;
     newAttitude->q0 = q0;
     newAttitude->q1 = q1;
     newAttitude->q2 = q2;
     newAttitude->q3 = q3;
     newAttitude->speedX = speedX;
     newAttitude->speedY = speedY;
     newAttitude->speedZ = speedZ;
     newAttitude->yaw = yaw;

     newAttitude->nextAttitude = NULL;  
     now->nextAttitude = newAttitude;
}

void calculatePath::addAttitude(Attitude *now, float speedX, float speedY, float speedZ, float yaw)
{
     Attitude *newAttitude;
     newAttitude = (Attitude*)malloc(sizeof(Attitude));
     now->nextAttitude->nextAttitude = newAttitude;
     newAttitude->preAttitude = now->nextAttitude;
     newAttitude->ID = curID++;
     newAttitude->speedX = speedX;
     newAttitude->speedY = speedY;
     newAttitude->speedZ = speedZ;
     newAttitude->yaw = yaw;
     newAttitude->nextAttitude = NULL;
     now->nextAttitude = newAttitude;
}

void calculatePath::showAllAttitude(Attitude *h)
{
     //if (h->exeTime != 0){
         printf("ID = %d  speed = ( %f, %f, %f); yaw = %f;\n", h->ID, h->speedX, h->speedY, h->speedZ, h->yaw);
     //}
     if (h->nextAttitude != NULL){
         showAllAttitude(h->nextAttitude);
     }
}

void calculatePath::showReverseAttitude(Attitude *n)
{
     if (n->preAttitude != NULL){
         cout << "ID = " <<n->ID << endl;
         showReverseAttitude(n->preAttitude);
     }
}

/*
void calculatePath::addTime(Attitude *now, int exTime)
{
     now->nextAttitude->exeTime += exTime; 
}
*/

void calculatePath::freeAttitude(Attitude *h)
{
     if(h->nextAttitude != NULL) 
     {
         //h->exeTime = 0;
         freeAttitude(h->nextAttitude);
         free(h);
     }
     else 
     {
         free(h);
         //h->exeTime = 0;
     }
}

Attitude* calculatePath::getNowAttitude()
{
     return now;
}

void calculatePath::exeAttitude(Attitude *h, DJI::onboardSDK::CoreAPI api)
{
     printf("exeAttitude\n");
     Attitude *n;
     n = h;

     FlightData *flightdata;
     flightdata = (FlightData*)malloc(sizeof(FlightData));
     flightdata->flag = 0x40;

     float q_q0, q_q1, q_q2, q_q3;
     float yaw, roll, pitch;

     while(n->nextAttitude != NULL)
     {
         //send()
         printf("ID = %d\n", n->nextAttitude->ID);
	 if(n->nextAttitude->ID == 0){
	      break;
	 }
	 //const Flight *flight = new Flight(&api);

	 if( flightdata->flag == 0x40){
	      //flightdata->flag = 0x40;
              //printf("flag = %d\n", flightdata->flag);

	      flightdata->x = n->nextAttitude->speedX;
              flightdata->y = n->nextAttitude->speedY;
              flightdata->z = n->nextAttitude->speedZ;
              flightdata->yaw = n->nextAttitude->yaw;
              api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
	 }
	 else if (flightdata->flag == 0x02){  // roll pitch
	      q_q0 = n->nextAttitude->q0;
              q_q1 = n->nextAttitude->q1;
              q_q2 = n->nextAttitude->q2;
              q_q3 = n->nextAttitude->q3;

 	      roll  = atan2(2.0 * (q_q3 * q_q2 + q_q0 * q_q1) , 1.0 - 2.0 * (q_q1 * q_q1 + q_q2 * q_q2));
	      pitch = asin(2.0 * (q_q2 * q_q0 - q_q3 * q_q1));
	      //yaw   = atan2(2.0 * (q_q3 * q_q0 + q_q1 * q_q2) , - 1.0 + 2.0 * (q_q0 * q_q0 + q_q1 * q_q1));

	      flightdata->x = roll;
              flightdata->y = pitch;
              flightdata->z = n->nextAttitude->speedZ;
              flightdata->yaw = n->nextAttitude->yaw;

	      api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
	 }
	
         n = n->nextAttitude;
	 sleep(1);
     }
}

void calculatePath::exeAttitudeR(Attitude *tail, DJI::onboardSDK::CoreAPI api)
{
     Attitude *n;
     n = tail;

     FlightData *flightdata;
     flightdata = (FlightData*)malloc(sizeof(FlightData));
     flightdata->flag = 0x40;

     float q_q0, q_q1, q_q2, q_q3;
     float yaw, roll, pitch;

     for(int i = 0; i < 20; i++){
	 flightdata->x = 0;
         flightdata->y = 0;
         flightdata->z = 0;
         flightdata->yaw = 0;
	 api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
     }

     while(n->nextAttitude->preAttitude != NULL)
     {
         printf("ID = %d\n", n->nextAttitude->ID);
	 if(n->nextAttitude->ID == 0){
	      TaskData *taskData;
	      taskData = (TaskData*)malloc(sizeof(TaskData));
	      taskData->cmdSequence++;
	      taskData->cmdData = 6;
	      api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskData), sizeof(TaskData), Flight::taskCallback, 100, 3); //landing
	      sleep(3);
              break;
         }

         if( flightdata->flag == 0x40){
              q_q0 = n->nextAttitude->q0;
              q_q1 = n->nextAttitude->q1;
              q_q2 = n->nextAttitude->q2;
              q_q3 = n->nextAttitude->q3;

	      /*
              yaw = atan2(2.0 * (q_q3 * q_q0 + q_q1 * q_q2) , - 1.0 + 2.0 * (q_q0 * q_q0 + q_q1 * q_q1));
	      if(yaw >= 0){
		   yaw -= 180;
	      }
	      else {
		   yaw += 180;
	      }*/
              //cout << "yaw = " << yaw << endl;

              flightdata->x = -(n->nextAttitude->speedX);
              flightdata->y = -(n->nextAttitude->speedY);
              flightdata->z = -(n->nextAttitude->speedZ);
	      flightdata->yaw = 0;
	      /*
	      if(n->nextAttitude->yaw >= 0){
		   flightdata->yaw = (n->nextAttitude->yaw) - 180;
	      }
	      else{
	           flightdata->yaw = (n->nextAttitude->yaw) + 180;
	      }*/
	      //flightdata->yaw = n->nextAttitude->yaw;
              
              api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
         }
         else if (flightdata->flag == 0x02){  // roll pitch
              q_q0 = n->nextAttitude->q0;
              q_q1 = n->nextAttitude->q1;
              q_q2 = n->nextAttitude->q2;
              q_q3 = n->nextAttitude->q3;

              roll  = atan2(2.0 * (q_q3 * q_q2 + q_q0 * q_q1) , 1.0 - 2.0 * (q_q1 * q_q1 + q_q2 * q_q2));
              pitch = asin(2.0 * (q_q2 * q_q0 - q_q3 * q_q1));
              yaw   = atan2(2.0 * (q_q3 * q_q0 + q_q1 * q_q2) , - 1.0 + 2.0 * (q_q0 * q_q0 + q_q1 * q_q1));
	      cout << "yaw = " << yaw << endl;

              flightdata->x = -(roll);
              flightdata->y = -(pitch);
              flightdata->z = n->nextAttitude->speedZ;
	      /*
              if(n->nextAttitude->yaw >= 0){
                   flightdata->yaw = (n->nextAttitude->yaw) - 180;
              }
              else{
                   flightdata->yaw = (n->nextAttitude->yaw) + 180;
              }*/
	      flightdata->yaw = yaw;

              api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
         }

         n->nextAttitude = n->nextAttitude->preAttitude;
         
	 //sleep(1);
	 struct timespec tim, tim2;
	 tim.tv_sec = 0;
	 tim.tv_nsec = 700000000L;
	 nanosleep(&tim , &tim2);
	 
     }


}

void calculatePath::SelfPredict(Attitude *tail, DJI::onboardSDK::CoreAPI api)
{
     Attitude *n;
     n = tail;

     FlightData *flightdata;
     flightdata = (FlightData*)malloc(sizeof(FlightData));
     flightdata->flag = 0x40;

     Flight *flight = new Flight(&api);

     float q_q0, q_q1, q_q2, q_q3;
     float yaw, roll, pitch;

     for(int i = 0; i < 20; i++){
         flightdata->x = 0;
         flightdata->y = 0;
         flightdata->z = 0;
         flightdata->yaw = 0;
         api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
     }

     while(n->nextAttitude->preAttitude != NULL)
     {
         printf("ID = %d\n", n->nextAttitude->ID);
         if(n->nextAttitude->ID == 0){
              TaskData *taskData;
              taskData = (TaskData*)malloc(sizeof(TaskData));
              taskData->cmdSequence++;
              taskData->cmdData = 6;
              api.send(2, 0, SET_CONTROL, CODE_TASK, (unsigned char *)&(*taskData), sizeof(TaskData), Flight::taskCallback, 100, 3); //landing
              sleep(3);
              break;
         }

         if( flightdata->flag == 0x40){
              q_q0 = n->nextAttitude->q0;
              q_q1 = n->nextAttitude->q1;
              q_q2 = n->nextAttitude->q2;
              q_q3 = n->nextAttitude->q3;

              /*
              yaw = atan2(2.0 * (q_q3 * q_q0 + q_q1 * q_q2) , - 1.0 + 2.0 * (q_q0 * q_q0 + q_q1 * q_q1));
              if(yaw >= 0){
                   yaw -= 180;
              }
              else {
                   yaw += 180;
              }*/
              //cout << "yaw = " << yaw << endl;

              flightdata->x = -(n->nextAttitude->speedX);
              flightdata->y = -(n->nextAttitude->speedY);
              flightdata->z = -(n->nextAttitude->speedZ);
              flightdata->yaw = 0;
              /*
              if(n->nextAttitude->yaw >= 0){
                   flightdata->yaw = (n->nextAttitude->yaw) - 180;
              }
              else{
                   flightdata->yaw = (n->nextAttitude->yaw) + 180;
              }*/
              //flightdata->yaw = n->nextAttitude->yaw;

	      api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
         }
         else if (flightdata->flag == 0x02){  // roll pitch
              q_q0 = n->nextAttitude->q0;
              q_q1 = n->nextAttitude->q1;
              q_q2 = n->nextAttitude->q2;
              q_q3 = n->nextAttitude->q3;

              roll  = atan2(2.0 * (q_q3 * q_q2 + q_q0 * q_q1) , 1.0 - 2.0 * (q_q1 * q_q1 + q_q2 * q_q2));
              pitch = asin(2.0 * (q_q2 * q_q0 - q_q3 * q_q1));
              yaw   = atan2(2.0 * (q_q3 * q_q0 + q_q1 * q_q2) , - 1.0 + 2.0 * (q_q0 * q_q0 + q_q1 * q_q1));
              cout << "yaw = " << yaw << endl;

              flightdata->x = -(roll);
              flightdata->y = -(pitch);
              flightdata->z = n->nextAttitude->speedZ;
              /*
              if(n->nextAttitude->yaw >= 0){
                   flightdata->yaw = (n->nextAttitude->yaw) - 180;
              }
              else{
                   flightdata->yaw = (n->nextAttitude->yaw) + 180;
              }*/
              flightdata->yaw = yaw;

              api.send(0, 0, SET_CONTROL, CODE_CONTROL, (unsigned char *)&(*flightdata), sizeof(FlightData));
         }
	 struct timespec tim, tim2;
         tim.tv_sec = 0;
         tim.tv_nsec = 700000000L;
         nanosleep(&tim , &tim2);

	 //Speed Error = RecordSpeed - CurrentSpeed
	 float speedXerr = (-(n->nextAttitude->speedX)) - flight->getVelocity().x;
	 float speedYerr = (-(n->nextAttitude->speedY)) - flight->getVelocity().y;
   	 float speedZerr = (-(n->nextAttitude->speedZ)) - flight->getVelocity().z;

	 ResetSpeed(n->nextAttitude->preAttitude, speedXerr, speedYerr, speedZerr);

         n->nextAttitude = n->nextAttitude->preAttitude;

         //sleep(1);
         tim.tv_sec = 0;
         tim.tv_nsec = 300000000L;
         //nanosleep(&tim , &tim2);

     }

}

void calculatePath::ResetSpeed(Attitude *pre, float speedXerr, float speedYerr, float speedZerr)
{
     if(speedXerr > 0.5){
	 pre->speedX += speedXerr;
     }
     if(speedYerr > 0.5){
	 pre->speedY += speedYerr;
     }
     //pre->speedZ += speedZerr;
}


