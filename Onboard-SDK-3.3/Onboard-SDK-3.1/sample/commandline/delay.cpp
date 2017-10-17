#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <time.h>

using namespace std;

int main(void)
{
    struct timespec tim, tim2;
    tim.tv_sec = 15;
    tim.tv_nsec = 0000000000L;
    cout << "wait for 10 sec" << endl;
    nanosleep(&tim , &tim2);
    cout << "XD" << endl;
    return 0;
}
