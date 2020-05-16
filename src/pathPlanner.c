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
 * @file pathPlanner.c
 * @author Anand Prakash
 * @date 22 April 2020
 * @brief This file contains the basic implementation of the Planner task and CAN
 *        Bus polling task.
 *
 * The main purpose of Planner component is to define and follow a given trajectory.
 * This trajectory is defined as a spline, that is, a line built through polynomial
 * interpolation at times on the map that represents at each point the position and
 * orientation that the car will have to follow. The spline can be enriched with additional
 * information such as speed to hold, stop, priorities, etc. The planner sends the goal state
 * of the vehicle (i.e., target steer and speed) to the DASM task that is in charge of writing
 * the commands in the CAN line the effective steer and speed to apply.
 *
 * The CAN Bus polling task snoops the key vehicle information (steer/wheel/break/acceleration
 * status...) from the on-board CAN bus and sends it to the Planner task.
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
 */

#include "mbse.h"
#include "mbseCuda.h"


/* Variable to store the CAN bus polling data */
static int hostCanBusPolling;

/* Structure to store the planner input data*/
static plannerData plannerDataInOut;

/* Function to read the planner input data from the global memory to local memory */
static void cInPlannerData()
{
    plannerDataInOut.canData = shmemReadPlannerBufferLabel(1);          // Read from offset 1
    plannerDataInOut.yawRate = shmemReadPlannerBufferLabel(3);          // Read from offset 3
    plannerDataInOut.xCar = shmemReadPlannerBufferLabel(4);             // Read from offset 4
    plannerDataInOut.velocity = shmemReadPlannerBufferLabel(5);         // Read from offset 5
    plannerDataInOut.yawCar = shmemReadPlannerBufferLabel(6);           // Read from offset 6
    plannerDataInOut.yawRate = shmemReadPlannerBufferLabel(7);          // Read from offset 7
    plannerDataInOut.matrixSFM = shmemReadSFMDataOutLabel(0);           // Read from offset 0
    plannerDataInOut.bBoxHost = shmemReadDetectionDataOutLabel(0);      // Read from offset 0
    plannerDataInOut.occupancyGrid = shmemReadGridDataBufferLabel(0);   // Read from offset 0
    plannerDataInOut.laneBoundary = shmemReadLaneBoundaryBufferLabel(0);    // Read from offset 0
}


/* Function to write the processed data by Planner task. To be fed as input data to DASM task. */
static void cOutPlannerData()
{
    int dataLength = sizeof(int);
    /* Write to offset 0. Provided as input to DASM task. */
    shmemWritePlannerDataOutLabel(0, dataLength, plannerDataInOut.steerObjective);
    /* Write to offset 2. Provided as input to DASM task. */
    shmemWritePlannerDataOutLabel(2, dataLength, plannerDataInOut.speedObjective);
}


/**
 * @brief This task is used perform the Planner task functionality.
 *
 * The main purpose of this component is to define and follow a given trajectory.
 * This trajectory is defined as a spline, that is, a line built through polynomial
 * interpolation at times on the map that represents at each point the position
 * and orientation that the car will have to follow. The planner sends the goal state
 * of the vehicle (i.e., target steer and speed) to the DASM task that is in charge
 * of writing the commands in the CAN line the effective steer and speed to apply.
 * It is executed on core number 5 on Jetson TX2 ARM A57 core with the thread priority of 98.
 *
 * @param[in] args Optional argument. Currently not in use.
 *
 * @return void
 */
void *pathPlannerCalculation(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    /* Set the core id to 5 */
    CPU_SET(5, &mask);

    pthread_t threadId = pthread_self();

    if (pthread_setaffinity_np(threadId, sizeof(cpu_set_t), &mask) == -1) {
        perror("sched_setaffinity");
        exit(2);
    }

    /* Set the thread priority to 98. Max priority 99 minus 1. */
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
        cInPlannerData();
        /* Call CUDA kernel */
        cuPlannerInterpolatePath(__func__, &plannerDataInOut);
        /* Copy the output value from local memory to shared memory. Analogous to the Cout operation.  */
        cOutPlannerData();
    }

    return NULL;
}

/* Function is left blank intentionally */
static void cInGetCanBusPollingData()
{
    // Left blank intentionally
}

/* Write the CAN Bus processed data to the planner data buffer as input to planner task */
static void cOutSetCanBusPollingData()
{
    int dataLength = sizeof(int);
    /* Write the CAN data to the planner buffer at offset 1 */
    shmemWritePlannerDataOutLabel(1, dataLength, hostCanBusPolling);
}


/**
 * @brief This task is used perform the CAN Bus Polling task functionality.
 *
 * This task snoops the key vehicle information (steer/wheel/break/acceleration status...)
 * from the on-board CAN bus and sends it to the Localization, Planner and EKF tasks.
 * It is executed on core number 5 on Jetson TX2 ARM A57 core with the thread priority of 99.
 *
 * @param[in] args Optional argument. Currently not in use.
 *
 * @return void
 */
void *pathPlannerCanBusPolling(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct sched_param param;
    uint16_t threadPriority = 0;
    cpu_set_t mask;
    CPU_ZERO(&mask);
    /* Set the thread on core 0. */
    CPU_SET(5, &mask);

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
        /* wait 500 milliseconds before next thread cycle begins */
        utilAddDelay(1000, &deadline);
        /* Copy the input values from shared memory to the local memory. Analogous to the CIn operation. */
        cInGetCanBusPollingData();
        /* Call CUDA kernel */
        cuPlannerFetchCanBusData(__func__, &hostCanBusPolling);
        /* Copy the output value from local memory to shared memory. Analogous to the Cout operation.  */
        cOutSetCanBusPollingData();
    }

    return NULL;
}
