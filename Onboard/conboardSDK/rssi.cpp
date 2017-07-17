#include "rssi.h"
void *RSSIDetectThr(void *ptr)
{
    //string cmd0 = "airmon-ng start wlan0"; 
    string cmd = "sudo airodump-ng mon0 -c 1,1 2>&1 | grep 'ntnu'"; //ping SSID HsTS
    string data;
    FILE * stream;
    const int max_buffer = 128 ;
    char buffer[max_buffer];
    char *delim = " ";
    char * pch;
    int ten;
    int filter=0;
    int count=0;
    
    char filename[] = "result.txt";
    fstream fp;
    fp.open(filename, ios::out|ios::app);
    if(!fp){ //Open failed, fp=0, if success, fp!=0
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
		ten = (pch[1] - '0')*10;
		ten += (pch[2]-'0');
		filter += ten;
		if(count%15==14){
		    filter /= 10;
		    cout << -filter <<endl;
		    fp << filter <<endl;
		    filter =0;
		}
		count++;
		//cout << pch << endl;
		
	    }
	pclose(stream);
    }

    //cout << data << endl;
    fp << "-------------" << endl;
    fp.close();
}
