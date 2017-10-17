#include <iostream>

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "DJI_API.h"

using namespace std;
using namespace DJI::onboardSDK;

int main(int argc, char *argv[])
{
    HardDriverManifold driver("/dev/ttyAMA0", 230400);
    driver.init();
    
    CoreAPI api(&driver);
    
    ActivateData *data;
    data = (ActivateData *)malloc(sizeof(ActivateData));
        
    data->version = SDK_VERSION;
    data->ID = 1026279;
    char *key = "bba4b69aaef4bba3ebb38806cce9575ba2b43676d93cfc4fe7bf8a8e11b3e850";
    data->encKey = key;
    data->reserved = 2;

    printf("data : %d\n", data);
    printf("*data : %d\n", *data);
    printf("ID : %d\n", data->ID);
    printf("key : %s\n", data-->encKey);
    printf("version : %d\n", data->version);
    printf("reserved : %d\n", data->reserved);

    //activate
    api.activate(data);
    
    API_LOG(&driver, STATUS_LOG, "Send Function...\n");
    api.send(2, 0, SET_ACTIVATION, CODE_ACTIVATE, (unsigned char *)&(*data), sizeof(*data) - sizeof(char *), CoreAPI::activateCallback, 1000, 3);
        
    return 0;
}
