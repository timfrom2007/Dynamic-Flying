#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

using namespace std;

struct RSSIDetectParam{
    double *rssi = 0;
    bool detecting = true;
};


void *RSSIDetectThr(void *ptr);
