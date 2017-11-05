#include <stdio.h>
#include <time.h>
#include <iwlib.h>

int main(){
    iwrange range;
    int sock;
    wireless_info info;
    sock = iw_sockets_open();
    fstream fp;
    char filename[] = "result.txt";
    fp.open(filename, ios::out|ios::app);
     
    if(iw_get_range_info(sock,"wlan0",&range)<0){
        printf("Error\n");
        exit(2);
    }
    
    int i=1;
  while(i){
    iw_get_stats(sock,"wlan0",&(info.stats),&range,1);
    fp << ((int8_t)info.stats.qual.level) <<endl;
    printf("%d\n",(int8_t)info.stats.qual.level);
    usleep(10000);
  }
	fp.close();
  exit(0);



}
