/*
 ************************************************************************************
 *   Copyright (c) 2020 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts -
 *        initial API and implementation
 ************************************************************************************
 * objectDetection.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Anand Prakash
 */


#include "mbse.h"
#include "mbseCuda.h"

/*
 *  This task is responsible of detecting and classifying the objects in the road.
 *  All the objects detected are visualized and the information produced is sent
 *  to the Planner task.
 */
void *vDetection(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(0, &mask);

    pthread_t threadId = pthread_self();

    if (pthread_setaffinity_np(threadId, sizeof(cpu_set_t), &mask) != 0) {
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
        /* Call CUDA kernel */
        addTwoVectors(__func__);
        /* wait 100 milliseconds before next thread cycle begins */
        addDelay(100);
    }

    return NULL;
}


/*
 *  Structure-From-Motion is a method for estimating 3-D structures (depth) from
 *  vehicle motion and sequences of 2-D images. This task returns a matrix of points
 *  representing the distance with respect the objects of the image.
 **/
void *vSFM(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(0, &mask);

    pthread_t threadId = pthread_self();

    if (pthread_setaffinity_np(threadId, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_setaffinity");
        exit(3);
    }

    setThreadPriority(threadId, 5);

    pthread_getschedparam(threadId, &threadPolicy, &param);

    printf("I am %s with priority %i on CPU %d\n", __func__, param.sched_priority, sched_getcpu());

    while(1)
    {
        uint32_t sum = 0;
        uint32_t prod = 0;

        /* Execute some instructions. Do some RT-things here */
        /* Call CUDA kernel */
        addTwoVectors(__func__);
        /* wait 200 milliseconds before next thread cycle begins */
        addDelay(200);
    }

    return NULL;
}
