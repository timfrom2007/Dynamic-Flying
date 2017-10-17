#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#define PI 3.141592654
#include "quaternion.h"

using namespace std;

quaternion::quaternion()
{
}

void quaternion::matrixMutiply(double** x, double* y)
{
   double y0 = x[0][0]*y[0]+x[0][1]*y[1]+x[0][2]*y[2];
   double y1 = x[1][0]*y[0]+x[1][1]*y[1]+x[1][2]*y[2];
   double y2 = x[2][0]*y[0]+x[2][1]*y[1]+x[2][2]*y[2];
   
   y[0] = y0;
   y[1] = y1;
   y[2] = y2;
}

void quaternion::Qrotate(double* q0, double rotateAxis[3], double angle)
{
   double u0 = rotateAxis[0];
   double u1 = rotateAxis[1];
   double u2 = rotateAxis[2];
   
   //printf("u0 = %f, u1 = %f, u2 = %f\nangle = %f\n\n", u0, u1, u2, angle);
   
   double *p = new double[3*3];
   double **q = new double*[3];
   for (int i = 0; i < 3; i++){
       q[i] = &p[i*3];
   }

   q[0][0] = u0*u0*(1-cos(angle))+cos(angle);   q[0][1] = u0*u1*(1-cos(angle))-u2*sin(angle);q[0][2] = u0*u2*(1-cos(angle))+u1*sin(angle);
   q[1][0] = u0*u1*(1-cos(angle))+u2*sin(angle);q[1][1] = u1*u1*(1-cos(angle))+cos(angle);   q[1][2] = u1*u2*(1-cos(angle))-u0*sin(angle);
   q[2][0] = u0*u2*(1-cos(angle))-u1*sin(angle);q[2][1] = u1*u2*(1-cos(angle))+u0*sin(angle);q[2][2] = u2*u2*(1-cos(angle))+cos(angle);

   /*
   for(int i = 0; i < 3; i++){
      for(int j = 0; j < 3; j++){
         printf("q[%d][%d] = %f  ", i, j, q[i][j]);
      }
      printf("\n");
   }
   printf("\n");
   */
   
   matrixMutiply( q, q0);
   free(p);
   free(q);
}
