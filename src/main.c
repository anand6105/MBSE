/*
 * main.c
 *
 *  Created on: Apr 11, 2020
 *      Author: anand
 */


/*******************************************************************************
 *   Copyright (c) 2019 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts - initial API and implementation
 *******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <limits.h>
#include <stdint.h>
#include "mbse.h"


typedef void *(*threadPool_t)(void *);

static void *vDetection(void *args);
static void *vPlanner(void *args);

static threadPool_t threadPool[MBSE_NUMBER_OF_THREADS] = {vDetection, vPlanner};

/* Function to set the priority of the thread */
static void setThreadPriority(int prio, int sched)
{
    struct sched_param param;
    /* Set real-time priority for this thread */
    param.sched_priority = prio;
    if (sched_setscheduler(0, sched, &param) < 0)
    {
        perror("sched_setscheduler");
    }
}

static void addDelay(uint32_t delay)
{
    struct timespec res;
    res.tv_sec = delay/1000;
    res.tv_nsec = (delay*1000000) % 1000000000;
    clock_nanosleep(CLOCK_MONOTONIC, 0, &res, NULL);
}



/* The thread to start */
static void *vPlanner(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    threadPriority = sched_get_priority_max(SCHED_FIFO);

    setThreadPriority(threadPriority, SCHED_FIFO);

    pthread_getschedparam(pthread_self(), &threadPolicy, &param);

    printf("I am %s with priority %i \n", __func__, param.sched_priority);

    while(1)
    {
        uint32_t sum = 0;
        uint32_t prod = 0;
        /* Execute some instructions. Do some RT-things here */
        for(executionCount = 0; executionCount < 10000; executionCount++)
        {
            // @TODO Do some random calculation. GPU calls to be made here.
            sum += 1;
            prod += (sum * 2);
            if (executionCount % 1000 == 0)
            {
                printf("%s executing loop %d\n", __func__, executionCount);
            }
        }
        /* Call CUDA kernel */
        addTwoVectors(__func__);
        /* wait 100 milliseconds before next thread cycle begins */
        addDelay(100);
    }

    return NULL;
}


/* The thread to start */
static void *vDetection(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;

    threadPriority = sched_get_priority_max(SCHED_FIFO);

    setThreadPriority(threadPriority, SCHED_FIFO);

    pthread_getschedparam(pthread_self(), &threadPolicy, &param);

    printf("I am %s with priority %i \n", __func__, param.sched_priority);

    while(1)
    {
        uint32_t sum = 0;
        uint32_t prod = 0;

        /* Execute some instructions. Do some RT-things here */
        for(executionCount = 0; executionCount < 10000; executionCount++)
        {
            // @TODO Do some random calculation. GPU calls to be made here.
            sum += 1;
            prod += (sum * 2);
            if (executionCount % 2000 == 0)
            {
                printf("%s executing loop %d\n", __func__, executionCount);
            }
        }
        /* Call CUDA kernel */
        addTwoVectors(__func__);
        /* wait 20 milliseconds before next thread cycle begins */
        addDelay(20);
    }

    return NULL;
}

static void error(int at)
{
    /* Just exit on error */
    fprintf(stderr, "Some error occured at %d", at);
    fflush(stderr);
    exit(1);
}

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

    startRealTimeThreads();
    return 0;
}

