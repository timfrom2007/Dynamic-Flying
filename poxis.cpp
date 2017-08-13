#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <cmath>

using namespace std;
float avg_filter(int*);
float mid_filter(int*);
float gauss_filter(int*,float);

int main(void)
{
    struct filtering_result{
	float average;
	float mid;
	float gauss;
    } filter;

    //string cmd0 = "airmon-ng start wlan0"; 
    string cmd = "watch -n0.1 \"iwconfig wlan0 | grep 'Signal level'\"";
    cout << cmd << endl;
    string data;
    string mid_fil;
    FILE * stream;
    const int max_buffer = 256 ;
    char buffer[max_buffer];
    char *delim = " ";
    char * pch;
    int collect[10];
    int ten;
    int count=0;
    char filename_avg[] = "result.txt";
    //char filename_mid[] = "result_mid5.txt";
    //char filename_gauss[] = "result_gauss5.txt";
    fstream fp1,fp2,fp3;
    fp1.open(filename_avg, ios::out|ios::app);
    //fp2.open(filename_mid, ios::out|ios::app);
    //fp3.open(filename_gauss, ios::out|ios::app);

    if(!fp1){ //Open failed, fp=0, if success, fp!=0
	cout << "Fail to Open file";
    }

    //popen(cmd0.c_str(), "r");
    stream = popen(cmd.c_str(), "r");
    if(stream){
        while(!feof(stream))
	    if(fgets(buffer, max_buffer, stream) != NULL){
		data.append(buffer);
		pch = strtok(buffer,delim);
		pch = strtok(NULL,delim);
		cout << data << endl;
		/*
		ten = (pch[1] - '0')*10;
                ten += (pch[2]-'0');
		collect[count] = -ten;
		if(count%10==9){
		    sort(collect, collect+10);
		    filter.average = avg_filter(collect);
		    filter.mid = mid_filter(collect);
		    filter.gauss = gauss_filter(collect, filter.average);
		    count=0;
		}
		else{ count++; }
		cout << filter.average << " " << filter.mid <<" "<< filter.gauss <<endl;
		*/
		fp1 << filter.average << endl;
                //fp2 << filter.mid << endl;
                //fp3 << filter.gauss << endl;

	    }
	pclose(stream);
    }

    //cout << data << endl;
    fp1 << "-------------" << endl;
    fp1.close();
    fp2 << "-------------" << endl;
    fp2.close();
    fp3 << "-------------" << endl;
    fp3.close();

}

float avg_filter(int* rssi)
{
    float result;
    for(int i=0;i<sizeof(rssi);i++){
        result += rssi[i];
    }
    result = result/sizeof(rssi);
    return result;
}

float que_filter(int rssi)
{
    return 0;
}

float mid_filter(int* rssi)
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

float gauss_filter(int* rssi, float average)
{
    float std=0, result=0;
    int count=0;
    for(int i=0; i<sizeof(rssi); i++){
	std = std + pow((rssi[i]-average), 2);
    }
    std = sqrt(std/(sizeof(rssi)-1));
    for(int i=0; i<sizeof(rssi); i++){
	if(rssi[i]>=average-std && rssi[i]<=average+std){
	    result += rssi[i];
	    count ++;
	}
    }
    result = result / count;
    return result;
}
