#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"

using namespace std;
/*
double* matrixMutiply(double x[3][3], double y[3][1])
{
   double *z[3][1] = {{ x[0][0]*y[0][0]+x[0][1]*y[1][0]+x[0][2]*y[2][0]},
                      { x[1][0]*y[0][0]+x[1][1]*y[1][0]+x[1][2]*y[2][0]},
                      { x[2][0]*y[0][0]+x[2][1]*y[1][0]+x[2][2]*y[2][0]}};
   return z;
}

double* Qrotate(double q0[3][1], double rotateAxis[3], double angle)
{
   double u0 = rotateAxis[0];
   double u1 = rotateAxis[1];
   double u2 = rotateAxis[2];

   double q[3][3] = {{ u0*u0*(1-cos(angle))+cos(angle), u0*u1*(1-cos(angle))-u2*sin(angle), u0*u2*(1-cos(angle))+u1*sin(angle)},
		   { u0*u1*(1-cos(angle))+u2*sin(angle), u1*u1*(1-cos(angle))+cos(angle), u1*u2*(1-cos(angle))-u0*sin(angle)},
		   { u0*u2*(1-cos(angle))-u1*sin(angle), u1*u2*(1-cos(angle))+u0*sin(angle), u2*u2*(1-cos(angle))+cos(angle)}};

   double *q1[3][1] = matrixMutiply(q,q0);
   return q1;
}
*/
double* test(double *x)
{
   x[0]++;
   printf("x[0] = %f, X[1] = %f, x[2] = %f\n\n", x[0], x[1], x[2]);
   return x;
}

int main(void)
{
   double a[3] = { 1.0, 1.0, 1.0};
   float a1[3][1] = {{ 1.0},{ 0.0},{ 0.0}}; 
   //cout << "a1 = " << a1[0][0] << a1[1][0] << a1[2][0] << endl;  
   //double *Axis[3] = { 0.0, 0.0, 1.0};
   double *Axis[3];
   Axis[0] = 0.0;
   Axis[1] = 0.0;
   Axis[2] = 1.0;

   double test[3] = test(&Axis);
   printf("Axis[0] = %f, Axis[1] = %f, Axis[2] = %f", Axis[0], Axis[1], Axis[2]);

   double angle = 45;
   //double* b1[3][1] = Qrotate( a1, Axis, 45);
   //cout << "b1 = " << b1[0][0] << b1[1][0] << b1[2][0] << endl;

   return 0;
}
