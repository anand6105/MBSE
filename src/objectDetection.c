/************************************************************************************
 * Copyright (C) 2020 Dortmund University of Applied Sciences and Arts and Others.  *
 *                                                                                  *
 * This file is part of the implementation of 2019 Waters Challenge                 *
 *                                                                                  *
 * The processing power offered by GPUs and their capability to execute parallel    *
 * workloads is exploited to execute and accelerate applications related to         *
 * advanced driver assistance systems.                                              *
 *                                                                                  *
 * The challenge consists in analytically master the complex HW/SW system (that     *
 * will be available as Amalthea model) to answer the following questions:          *
 *                                                                                  *
 * Response Time Computation:                                                       *
 *  a) Given an application consisting of a set of dependent tasks and a given      *
 *     mapping, calculate its end-to-end response time.                             *
 *  b) The response time should account for the time for the copy engine to         *
 *     transfer data between the CPU and the GPU.                                   *
 *  c) The response time should account for the time for the data transfers between *
 *     the CPU and the shared main memory considering a read/execute/write semantic.*
 *  d) Optionally the off-loading mechanism (synchronous/asynchronous) can be       *
 *     changed to further optimize the end-to-end latencies.                        *
 ************************************************************************************/


/**
 * @file objectDetection.c
 * @author Anand Prakash
 * @date 22 April 2020
 * @brief This file contains the basic implementation of the Detection and Structure
 *        from motion tasks.
 *
 * The detection task is responsible of detecting and classifying the objects in the road.
 * It uses a machine learning approach. All the objects detected are visualized and the
 * information produced is sent to the Planner task.
 * Structure-From-Motion task is a method for estimating 3-D structures (depth) from vehicle
 * motion and sequences of 2-D images. This task returns a matrix of points representing
 * the distance with respect the objects of the image.
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
 */

#include "mbse.h"
#include "mbseCuda.h"

/* Structure variable to store the object detection input/output data */
static detectObject objectInfo;
/* Structure variable to store the structure from motion input/output data*/
static sfmData sfmVal;

/* Function to read the input data from the global memory to local memory */
static void cInGetObjectDetectedInfo()
{
    objectInfo.bboxHostDetection    = shmemReadSFMDetectionDataInLabel(0); // Read from offset 0
    objectInfo.imageHostDetection   = shmemReadSFMDetectionDataInLabel(1); // Read from offset 1
    objectInfo.bboxDeviceDetection  = shmemReadSFMDetectionDataInLabel(2); // Read from offset 2
    objectInfo.imageDeviceDetection = shmemReadSFMDetectionDataInLabel(3); // Read from offset 3
}

/* Function to write the processed to the planner task buffer */
static void cOutSetObjectDetectedInfo()
{
    int dataLength = sizeof(objectInfo);
    /* Write to offset 0 */
    shmemWriteDetectionDataOutLabel(0, dataLength, &objectInfo);
}

/*
/*
 * @brief This task is used perform the detection functionality
 *
 * This task is responsible of detecting and classifying the objects in the road.
 * All the objects detected are visualized and the information produced is sent
 * to the Planner task. It is executed on core number 0 on Jetson TX2 ARM A57 core
 * with the thread priority of 94.
 *
 * @param args Optional argument. Currently not in use.
 *
 * @return void
 **/
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

    /*Set the thread priority to 98. Max priority 99 minus 1.*/
    utilSetThreadPriority(threadId, 1);

    pthread_getschedparam(threadId, &threadPolicy, &param);

    printf("I am %s with priority %i on CPU %d\n", __func__, param.sched_priority, sched_getcpu());

    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    while(1)
    {
        /* wait 500 milliseconds before next thread cycle begins */
        utilAddDelay(1000, &deadline);
        /* Copy the input values from shared memory to the local memory. Analogous to the CIn operation. */
        cInGetObjectDetectedInfo();
        /* Call CUDA kernel. All runnables are executed in this CUDA kernel call */
        cuDetectObject(__func__, &objectInfo);
        /* Copy the output value from local memory to shared memory. Analogous to the Cout operation.  */
        cOutSetObjectDetectedInfo();

    }

    return NULL;
}


/* Function to read the input data from the global memory to local memory */
static void cInGetSFMInfo()
{
    sfmVal.matrixSFMHost  = shmemReadSFMDetectionDataInLabel(4); // Read from offset 4
    sfmVal.imageSFMHost   = shmemReadSFMDetectionDataInLabel(5); // Read from offset 5
}

/* Function to write the processed to the planner task buffer */
static void cOutSetSFMInfo()
{
    int dataLength = sizeof(sfmVal);
    /* Write to offset 0. To fed as an input to planner task. */
    shmemWriteSFMDataOutLabel(0, dataLength, &sfmVal);
}


/*
 * @brief This task is used perform the Structure-From-Motion functionality.
 *
 * Structure-From-Motion is a method for estimating 3-D structures (depth) from
 * vehicle motion and sequences of 2-D images. This task returns a matrix of points
 * representing the distance with respect the objects of the image. It is executed on
 * core number 0 on Jetson TX2 ARM A57 core with the thread priority of 99.
 *
 * @param args Optional argument. Currently not in use.
 *
 * @return void
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

    /* Set the thread priority to 99. */
    utilSetThreadPriority(threadId, 0);

    pthread_getschedparam(threadId, &threadPolicy, &param);

    printf("I am %s with priority %i on CPU %d\n", __func__, param.sched_priority, sched_getcpu());

    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    while(1)
    {
        /* wait 300 milliseconds before next thread cycle begins */
        utilAddDelay(1000, &deadline);
        /* Copy the input values from shared memory to the local memory. Analogous to the CIn operation. */
        cInGetSFMInfo();
        /* Call CUDA kernel. All runnables are executed in this CUDA kernel call */
        cuObjDetectSFM(__func__, &sfmVal);
        /* Copy the output value from local memory to shared memory. Analogous to the Cout operation.  */
        cOutSetSFMInfo();
    }

    return NULL;
}
