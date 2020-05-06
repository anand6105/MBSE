/*
 ************************************************************************************
 *   Copyright (c) 2020 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts -
 *        initial API and implementation
 ************************************************************************************
 * main.c
 *
 *  Created on: Apr 11, 2020
 *      Author: Anand Prakash
 */

#include "mbse.h"
#include "mbseCuda.h"


typedef void *(*threadPool_t)(void *);


static threadPool_t threadPool[MBSE_NUMBER_OF_THREADS] = {objDetectGetObject, objDetectStructureFromMotion, vPlanner, vCanBusPolling};


static void startRealTimeThreads(void)
{
    pthread_t thread[MBSE_NUMBER_OF_THREADS];
    pthread_attr_t attr[MBSE_NUMBER_OF_THREADS];
    uint8_t threadIndex = 0;

    for(threadIndex = 0; threadIndex < MBSE_NUMBER_OF_THREADS; threadIndex++)
    {
        /* init to default values */
        if (pthread_attr_init(&attr[threadIndex]))
        {
            error(1);
        }

        /* And finally start the actual thread */
        if (pthread_create(&thread[threadIndex],
                &attr[threadIndex],
                threadPool[threadIndex], NULL))
        {
            error(2);
        }
    }
    for(threadIndex = 0; threadIndex < MBSE_NUMBER_OF_THREADS; threadIndex++)
    {
        pthread_join(thread[threadIndex], NULL);
    }
}



int main(int argc, char *argv[])
{
    uint8_t userPrivilege;
    userPrivilege = getuid();
    if(userPrivilege != 0)
    {
        fprintf(stdout, "The current user is not root\n");
        fflush(stdout);
        exit(1);
    }

    getCudaDeviceProperties();

    startRealTimeThreads();
    return 0;
}

