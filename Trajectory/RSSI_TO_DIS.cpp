#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>

using namespace std;

int main (){

    /*
    double n=2, A=-10; //n:path-loss exponent, A:RSSI per unit
    double dist=10, exp= (A-(-50.813627)) / (10*n);
    dist = pow(dist, exp);
    dist = sqrt(pow(dist,n) + pow(0.700219 , n));
    cout<< dist/1000<<endl;
    */
    

  double theta, dist;
  double lon1 = 1.988959; //1.988964
  double lon2 = 1.988934;
  double lat1 = 0.393446; //0.393454, 0.393438 - 0.393470
  double lat2 = 0.393448;

  theta = lon1 - lon2;
  dist = sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(theta);
  dist = acos(dist);
  dist = dist * 180 / 3.14159265358979323846;
  dist = dist * 60 * 1.1515;
  dist = dist * 1.609344;
  printf("%lf", dist*1000);
  //double a = asin(254828/255099);
  //printf("%lf", a);


   
system("pause");
  return 0;

}





