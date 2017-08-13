/*
#include "rssi.h"
float RSSIDetectThr()
{
    struct filtering_result{
	float median;
    } filter;

    iwrange range;
    int sock;
    wireless_info info;
    sock = iw_sockets_open();
    int collect[20];

    if(iw_get_range_info(sock,"wlan0",&range)<0){
        printf("Error\n");
        exit(2);
    }

    int i=0;
    int count=0;
  while(i<1000){
    iw_get_stats(sock,"wlan0",&(info.stats),&range, 1);
    int r = (int8_t)info.stats.qual.level;
    if(i%200==0){
	collect[count] = (int8_t)info.stats.qual.level;
        printf("%d\n",(int8_t)info.stats.qual.level);
	count++;
    }
    usleep(500);
        i++;
  }

    sort(collect, collect+20);
    filter.median = median_filter(collect);
    

    return filter.median;
}


float median_filter(int* rssi)
{
    float result;
    int mid = sizeof(rssi)/2;
    if(sizeof(rssi)%2==0){
        result = (rssi[mid]+rssi[mid+1])/2;
    }
    else{
        result = rssi[mid+1];
    }
    return result;
}
*/
