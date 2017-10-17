#ifndef QUATERNION_H
#define QUATERNION_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#define PI 3.141592654

using namespace std;

class quaternion
{
  public :
         quaternion();
         void Qrotate(double* q0, double rotateAxis[3], double angle);
  private :
         void matrixMutiply(double** x, double* y);
};

#endif

