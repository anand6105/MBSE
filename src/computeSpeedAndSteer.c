/*
 ************************************************************************************
 *   Copyright (c) 2020 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts -
 *        initial API and implementation
 ************************************************************************************
 * computeSpeedAndSteer.c
 *
 *  Created on: May 12, 2020
 *      Author: Anand Prakash
 */

#include "mbse.h"
#include "mbseCuda.h"

static int steerObjectiveDASM;
static int speedObjectiveDASM;


static void cInGetDASMData()
{
    // Left blank intentionally
    steerObjectiveDASM = shmemReadPlannerBufferLabel(0);
    speedObjectiveDASM = shmemReadPlannerBufferLabel(2);
}


static void cOutSetDASMData()
{
    /* Divide the length by 4 as it is an integer pointer */
    int dataLength = sizeof(int);
    shmemWritePlannerDataOutLabel(0, dataLength, steerObjectiveDASM);
    shmemWritePlannerDataOutLabel(0, dataLength, speedObjectiveDASM);
}

/* This task computes and establishes the speed and steer that must be effectively
 * employed from the information that is provided by the Path Planner task*/
void *computeSpeedAndSteer(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(3, &mask);

    pthread_t threadId = pthread_self();

    if (pthread_setaffinity_np(threadId, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_setaffinity");
        exit(3);
    }

    utilSetThreadPriority(threadId, 0);

    pthread_getschedparam(threadId, &threadPolicy, &param);

    printf("I am %s with priority %i on CPU %d\n", __func__, param.sched_priority, sched_getcpu());

    utilInitializeTimer(DASM_TIMER);
    while(1)
    {
        fprintf(stdout, "DASM task  started\n");
        /* Copy the input values from shared memory to the local memory. Analogous to the CIn operation. */
        cInGetDASMData();
        /* Call CUDA kernel */
        cuProcessDASM(__func__, &steerObjectiveDASM, &speedObjectiveDASM);
        fprintf(stdout, "DASM task complete. Wait for 150 ms\n");
        /* Copy the output value from local memory to shared memory. Analogous to the Cout operation.  */
        cOutSetDASMData();
        /* wait 150 milliseconds before next thread cycle begins */
        utilAddDelay(150, DASM_TIMER);
    }

    return NULL;
}




static void cInOSOverheadData()
{
    // Left blank intentionally
}


static void cOutOSOverheadData()
{
    // Left blank intentionally
}

/* This task is used to add the OS overhead to the overall tasks */
void *computeOSOverhead(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(3, &mask);

    pthread_t threadId = pthread_self();

    if (pthread_setaffinity_np(threadId, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_setaffinity");
        exit(3);
    }

    utilSetThreadPriority(threadId, 10);

    pthread_getschedparam(threadId, &threadPolicy, &param);

    printf("I am %s with priority %i on CPU %d\n", __func__, param.sched_priority, sched_getcpu());

    utilInitializeTimer(OS_OVERHEAD_TIMER);
    while(1)
    {
        fprintf(stdout, "Add OS overhead\n");
        /* Copy the input values from shared memory to the local memory. Analogous to the CIn operation. */
        cInOSOverheadData();
        /* Call CUDA kernel */
        addOSOverhead(__func__);
        fprintf(stdout, "OS Overhead complete. Wait for 500 ms\n");
        /* Copy the output value from local memory to shared memory. Analogous to the Cout operation.  */
        cOutOSOverheadData();
        /* wait 500 milliseconds before next thread cycle begins */
        utilAddDelay(500, OS_OVERHEAD_TIMER);
    }

    return NULL;
}
