#ifndef VINCENTY_H
#define VINCENTY_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "slamac.h"

using namespace std;

class Vincenty
{
  public :
         Vincenty();
         void destVincenty(double lat1, double lon1, double bearing, double dist, double *lat2out, double *lon2out);
	 double distVincenty(double lat1, double lon1, double lat2, double lon2);
};

#endif
