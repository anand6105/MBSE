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


/* Variable to store the CAN bus polling data */
static int hostCanBusPolling;

/* Structure to store the planner input data*/
static plannerData plannerDataInOut;


static void cInPlannerData()
{
    // Left blank intentionally
    plannerDataInOut.canData = shmemReadPlannerBufferLabel(1);
    plannerDataInOut.yawRate = shmemReadPlannerBufferLabel(3);
    plannerDataInOut.xCar = shmemReadPlannerBufferLabel(4);
    plannerDataInOut.velocity = shmemReadPlannerBufferLabel(5);
    plannerDataInOut.yawCar = shmemReadPlannerBufferLabel(6);
    plannerDataInOut.yawRate = shmemReadPlannerBufferLabel(7);
    plannerDataInOut.matrixSFM = shmemReadSFMDataOutLabel(0);
    plannerDataInOut.bBoxHost = shmemReadDetectionDataOutLabel(0);
    plannerDataInOut.occupancyGrid = shmemReadGridDataBufferLabel(0);
    plannerDataInOut.laneBoundary = shmemLaneBoundaryBufferLabel(0);
}


static void cOutPlannerData()
{
    /* Divide the length by 4 as it is an integer pointer */
    int dataLength = sizeof(int);
    shmemWritePlannerDataOutLabel(0, dataLength, plannerDataInOut.steerObjective);
    shmemWritePlannerDataOutLabel(1, dataLength, plannerDataInOut.speedObjective);
}

/* The main purpose of this component is to define and follow a given trajectory.
 * This trajectory is defined as a spline, that is, a line built through polynomial
 * interpolation at times on the map that represents at each point the position
 * and orientation that the car will have to follow. The planner sends the goal state
 * of the vehicle (i.e., target steer and speed) to the DASM task that is in charge
 * of writing the commands in the CAN line the effective steer and speed to apply.*/
void *pathPlannerCalculation(void *args)
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
        fprintf(stdout, "Planner task started\n");
        /* Copy the input values from shared memory to the local memory. Analogous to the CIn operation. */
        cInPlannerData();
        /* Call CUDA kernel */
        cuPlannerInterpolatePath(__func__, &plannerDataInOut);
        /* Copy the output value from local memory to shared memory. Analogous to the Cout operation.  */
        cOutPlannerData();
        fprintf(stdout, "Planner task is complete. Wait for 500 ms\n");
        /* wait 500 milliseconds before next thread cycle begins */
        utilAddDelay(500, PLANNER_TIMER);
    }

    return NULL;
}

static void cInGetCanBusPollingData()
{
    // Left blank intentionally
}


static void cOutSetCanBusPollingData()
{
    /* Divide the length by 4 as it is an integer pointer */
    int dataLength = sizeof(int);
    shmemWritePlannerDataOutLabel(1, dataLength, hostCanBusPolling);
}

/* This task snoops the key vehicle information (steer/wheel/break/acceleration status...)
 * from the on-board CAN bus and sends it to the Localization, Planner and EKF tasks.*/
void *pathPlannerCanBusPolling(void *args)
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
        fprintf(stdout, "Can bus polling started\n");
        /* Copy the input values from shared memory to the local memory. Analogous to the CIn operation. */
        cInGetCanBusPollingData();
        /* Call CUDA kernel */
        //cuPlannerFetchCanBusData(__func__, &hostCanBusPolling);
        fprintf(stdout, "Can Bus polling complete. Wait for 150 ms\n");
        /* Copy the output value from local memory to shared memory. Analogous to the Cout operation.  */
        cOutSetCanBusPollingData();
        /* wait 150 milliseconds before next thread cycle begins */
        utilAddDelay(150, CAN_BUS_POLLING_TIMER);
    }

    return NULL;
}
