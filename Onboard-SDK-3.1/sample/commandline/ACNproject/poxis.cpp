#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <iwlib.h>


using namespace std;

double kalman_filter(double* rssi)
{
    double X=0.0, P=3.0, Q=0.0001, K=0.0, R=1.0, I=1.0;
    
    for(int i=0; i<sizeof(*rssi); i++)
    {
        X=X;
        P = P+Q;
        K = P / (P + R);
        X = X + K * (rssi[i] - X);
        P = (I - K) * P;
    }
    
    return X;
    
}

int main (){

    iwrange range;
    int sock;
    wireless_info info;
    sock = iw_sockets_open();
    double fCollect[50];
    
    
    if(iw_get_range_info(sock,"wlan0",&range)<0){
        printf("Error\n");
        exit(2);
    }
    
    int i=0;
    int count=0;
    while(i<5000){
        iw_get_stats(sock,"wlan0",&(info.stats),&range, 1);
        int r = (int8_t)info.stats.qual.level;
        if(i%10==0){
            fCollect[count] = ((double)r/2) - 100;
            //printf("%d\n",(int8_t)info.stats.qual.level);
            printf("%lf\n",fCollect[count]);
            count++;
        }
        usleep(100*100);
        i++;
        cout << i << endl;
    }
    double result = kalman_filter(fCollect);
    cout << "Kalman Result:" << result <<endl;
    
    return 0;
}


