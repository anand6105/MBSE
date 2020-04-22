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

    setThreadPriority(threadId, 5);

    pthread_getschedparam(threadId, &threadPolicy, &param);

    printf("I am %s with priority %i on CPU %d\n", __func__, param.sched_priority, sched_getcpu());

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
        addTwoVectors(__func__);
        /* wait 100 milliseconds before next thread cycle begins */
        addDelay(210);
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

    setThreadPriority(threadId, 0);

    pthread_getschedparam(threadId, &threadPolicy, &param);

    printf("I am %s with priority %i on CPU %d\n", __func__, param.sched_priority, sched_getcpu());

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
        addTwoVectors(__func__);
        /* wait 20 milliseconds before next thread cycle begins */
        addDelay(90);
    }

    return NULL;
}
