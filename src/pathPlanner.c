/*
 ************************************************************************************
 *   Copyright (c) 2020 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts -
 *        initial API and implementation
 ************************************************************************************
 * pathPlanner.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Anand Prakash
 */


#include "mbse.h"
#include "mbseCuda.h"


/* The thread to start */
void *vPlanner(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(5, &mask);

    pthread_t threadId = pthread_self();

    if (pthread_setaffinity_np(threadId, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_setaffinity");
        exit(2);
    }

    utilSetThreadPriority(threadId, 5);

    pthread_getschedparam(threadId, &threadPolicy, &param);

    printf("I am %s with priority %i on CPU %d\n", __func__, param.sched_priority, sched_getcpu());

    utilInitializeTimer(PLANNER_TIMER);
    while(1)
    {
        uint32_t sum = 0;
        uint32_t prod = 0;
        /* Execute some instructions. Do some RT-things here */
        for(executionCount = 0; executionCount < 20000; executionCount++)
        {
            // @TODO Do some random calculation. GPU calls to be made here.
            sum += 1;
            prod += (sum * 2);
            if (executionCount % 1000 == 0)
            {
                //printf("%s executing loop %d\n", __func__, executionCount);
            }
        }
        /* Call CUDA kernel */
        //addTwoVectors(__func__);
        fprintf(stdout, "Planner task complete. Wait for 600 ms\n");
        /* wait 600 milliseconds before next thread cycle begins */
        utilAddDelay(600, PLANNER_TIMER);
    }

    return NULL;
}


/* The thread to start */
void *vCanBusPolling(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(5, &mask);

    pthread_t threadId = pthread_self();

    if (pthread_setaffinity_np(threadId, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_setaffinity");
        exit(3);
    }

    utilSetThreadPriority(threadId, 0);

    pthread_getschedparam(threadId, &threadPolicy, &param);

    printf("I am %s with priority %i on CPU %d\n", __func__, param.sched_priority, sched_getcpu());

    utilInitializeTimer(CAN_BUS_POLLING_TIMER);
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
                //printf("%s executing loop %d\n", __func__, executionCount);
            }
        }
        /* Call CUDA kernel */
        //addTwoVectors(__func__);
        fprintf(stdout, "Can Bus polling complete. Wait for 300 ms\n");
        /* wait 300 milliseconds before next thread cycle begins */
        utilAddDelay(300, CAN_BUS_POLLING_TIMER);
    }

    return NULL;
}
