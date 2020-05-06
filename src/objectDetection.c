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


static detectObject objectInfo;


static void cInGetObjectDetectedInfo()
{
    objectInfo.bboxHostDetection    = shmemReadSFMDetectionDataInLabel(0);
    objectInfo.imageHostDetection   = shmemReadSFMDetectionDataInLabel(1);
    objectInfo.bboxDeviceDetection  = shmemReadSFMDetectionDataInLabel(2);
    objectInfo.imageDeviceDetection = shmemReadSFMDetectionDataInLabel(3);
}


static void cOutSetObjectDetectedInfo()
{
    /* Divide the length by 4 as it is an integer pointer */
    int dataLength = sizeof(objectInfo);
    shmemWriteDetectionDataOutLabel(0, dataLength, &objectInfo);
}
/*
 *  This task is responsible of detecting and classifying the objects in the road.
 *  All the objects detected are visualized and the information produced is sent
 *  to the Planner task.
 */
void *objDetectGetObject(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    /* Assign the task to core 0 */
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
        /* Copy the input values from shared memory to the local memory. Analogous to the CIn operation. */
        cInGetObjectDetectedInfo();
        /* Call CUDA kernel. All runnables are executed in this CUDA kernel call */
        cuDetectObject(__func__, &objectInfo);
        /* Copy the output value from local memory to shared memory. Analogous to the Cout operation.  */
        cOutSetObjectDetectedInfo();
        fprintf(stdout, "Detection of object is complete.Wait for 300 ms.\n");
        /* wait 300 milliseconds before next thread cycle begins */
        addDelay(300);
    }

    return NULL;
}


/*
 *  Structure-From-Motion is a method for estimating 3-D structures (depth) from
 *  vehicle motion and sequences of 2-D images. This task returns a matrix of points
 *  representing the distance with respect the objects of the image.
 **/
void *objDetectStructureFromMotion(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    /* Assign the task to core 0 */
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
        /* Execute some instructions. Do some RT-things here */
        /* Call CUDA kernel */
        //addTwoVectors(__func__);
        fprintf(stdout,"Structure from motion task is complete. Wait for 400ms.\n ");
        /* wait 400 milliseconds before next thread cycle begins */
        addDelay(400);
    }

    return NULL;
}
