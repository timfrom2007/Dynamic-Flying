#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>

void *thread(void *ptr)
{
    //int i;
    //j = 0;
    for(int i = 0; i < 100; i++){
        //sleep(1);
        //printf("This is a pthread.\n");
	//(*ptr)++;
	printf("ptr = %d\n", ptr);
    }
}

int main(void)
{
    pthread_t id;
    int i,ret;
    ret = pthread_create( &id, NULL, thread, (void*)10);
    if(ret != 0){
        printf ("Create pthread error!\n");
        exit (1);
    }
    /*
    for(i = 0; i < 3; i++){
        printf("This is the main process.\n");
        sleep(1);
    }
    */
    pthread_join( id, NULL);
    return (0);
}
