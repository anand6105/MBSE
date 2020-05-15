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
 * @file computeSpeedAndSteer.c
 * @author Anand Prakash
 * @date 12 May 2020
 * @brief This file contains the basic implementation of the DASM and OS overhead tasks.
 *
 * This task computes and establishes the speed and steer that must be effectively
 * employed from the information that is provided by the Path Planner task. It also
 * implements a task which introduces the OS overhead that may occur during the
 * execution.
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
 */

#include "mbse.h"
#include "mbseCuda.h"

/* Variable to store the DASM steer output */
static int steerObjectiveDASM;
/* Variable to store the DASM speed output */
static int speedObjectiveDASM;


/* Function to read the planner output buffer. These values are used as input to DASM task. */
static void cInGetDASMData(void)
{
    steerObjectiveDASM = shmemReadPlannerBufferLabel(0); // Read from offset 0
    speedObjectiveDASM = shmemReadPlannerBufferLabel(2); // Read from offset 2
}


/* Function to write back the processed data to output buffer. */
static void cOutSetDASMData()
{
    int dataLength = sizeof(int);
    /* Write to offset 0 of output buffer */
    shmemWritePlannerDataOutLabel(0, dataLength, steerObjectiveDASM);
    /* Write to offset 1 of output buffer */
    shmemWritePlannerDataOutLabel(2, dataLength, speedObjectiveDASM);
}

/*
 * @brief This function implements the DASM task.
 *
 * This task computes and establishes the speed and steer that must be effectively
 * employed from the information that is provided by the Path Planner task. It is executed
 * on core number 3 on Jetson TX2 ARM A57 core with the thread priority of 97.
 *
 * @param args Optional argument. Currently not in use.
 *
 * @return void
 **/
void *computeSpeedAndSteer(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    /* Execute of core ID 3 */
    CPU_SET(3, &mask);

    pthread_t threadId = pthread_self();

    if (pthread_setaffinity_np(threadId, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_setaffinity");
        exit(3);
    }

    /* Set the priority to 99 */
    utilSetThreadPriority(threadId, 0);

    pthread_getschedparam(threadId, &threadPolicy, &param);

    fprintf(stdout, "I am %s with priority %i on CPU %d\n", __func__,
                            param.sched_priority, sched_getcpu());

    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    while(1)
    {
        /* wait 400 milliseconds before next thread cycle begins */
        utilAddDelay(1000, &deadline);
        /* Copy the input values from shared memory to the local memory. Analogous to the CIn operation. */
        cInGetDASMData();
        /* Call CUDA kernel */
        cuProcessDASM(__func__, &steerObjectiveDASM, &speedObjectiveDASM);
        /* Copy the output value from local memory to shared memory. Analogous to the Cout operation.  */
        cOutSetDASMData();
    }

    return NULL;
}



/* This function is left intentionally blank as there is no Cin operation */
static void cInOSOverheadData()
{
    // Left blank intentionally
}


/* Function to write data to all input buffers */
static void cOutOSOverheadData()
{
    int localVal = shmemReadSFMDetectionDataInLabel(0);
    localVal++;
    /* Write to detection and SFM data buffer. Provided as input */
    shmemWriteSFMDetectionDataInLabel(0, sizeof(int), localVal);
    shmemWriteSFMDetectionDataInLabel(1, sizeof(int), localVal);
    shmemWriteSFMDetectionDataInLabel(2, sizeof(int), localVal);
    shmemWriteSFMDetectionDataInLabel(3, sizeof(int), localVal);
    shmemWriteSFMDetectionDataInLabel(4, sizeof(int), localVal);
    shmemWriteSFMDetectionDataInLabel(5, sizeof(int), localVal);
    /* Write to data grid buffer */
    shmemWriteGridDataBufferLabel(0, sizeof(int), localVal);
    /* Write to lane boundary buffer */
    shmemWriteLaneBoundaryBufferLabel(0, sizeof(int), localVal);
}

/*
/*
 * @brief This task is used to add the OS overhead to the overall tasks based on the
 *        Amalthea task model.
 *
 * This task adds an additional overhead to the overall application. It is used to
 * simulate the overhead that occur in real scenario. It is executed
 * on core number 3 on Jetson TX2 ARM A57 core with the thread priority of 89.
 *
 * @param args Optional argument. Currently not in use.
 *
 * @return void
 **/
void *computeOSOverhead(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    /* Execute the thread on core id 3 */
    CPU_SET(3, &mask);

    pthread_t threadId = pthread_self();

    if (pthread_setaffinity_np(threadId, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_setaffinity");
        exit(3);
    }

    /* Set the priority to 98. Max priority is 99 minus 1. */
    utilSetThreadPriority(threadId, 0);

    pthread_getschedparam(threadId, &threadPolicy, &param);

    printf("I am %s with priority %i on CPU %d\n", __func__, param.sched_priority, sched_getcpu());

    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    while(1)
    {
        utilAddDelay(1000, &deadline);
        /* wait 450 milliseconds before next thread cycle begins */
        /* Copy the input values from shared memory to the local memory. Analogous to the CIn operation. */
        cInOSOverheadData();
        /* Call CUDA kernel */
        addOSOverhead(__func__);
        /* Copy the output value from local memory to shared memory. Analogous to the Cout operation.  */
        cOutOSOverheadData();
    }

    return NULL;
}
