#include <stdio.h>
#include <time.h>
#include <iwlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>
using namespace std;

int main(){

    iwrange range;
    int sock;
    wireless_info info;
    sock = iw_sockets_open();
    int collect[50];
    
    if(iw_get_range_info(sock,"wlan1",&range)<0){
        printf("Error\n");
        exit(2);
    }

    char filename[]="HappyDay.txt";
    fstream fp;
    fp.open(filename, ios::out);//開啟檔案
    if(!fp){//如果開啟檔案失敗，fp為0；成功，fp為非0
        cout<<"Fail to open file: "<<filename<<endl;
    }
    int i=0;
    int count=0;
    while(i<50000){
	iw_get_stats(sock,"wlan1",&(info.stats),&range, 1);
        int r = (int8_t)info.stats.qual.level;
        if(i%500==0){
	    cout << r << endl;
	    fp<< r <<" ";//寫入字串
            collect[count] = r;
            count++;
        }
        i++;
	usleep(100);
    }
     fp.close();//關閉檔案
    double X=0.0, P=3.0, Q=0.0001, K=0.0, R=1.0, I=1.0;
cout << sizeof(collect)/sizeof(*collect) << endl;;
    for(int i=0; i<50; i++){
        X=X;
        P = P+Q;
        K = P / (P + R);
        X = X + K * ((double)collect[i] - X);
        P = (I - K) * P;
    }

    cout << "X= " << X << endl;
}

