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
 * @file main.c
 * @author Anand Prakash
 * @date 11 April 2020
 * @brief This file contains the entry point of the applications and consists of the
 *        initialization of memory, threads of the application
 *
 * This is the entry point of the application. The application can be executed only with
 * root/superuser privileges and get the CUDA device properties along with initializing
 * the threads and start their execution.
 * The tasks are executed on three ARM A57 cores and uses RMS scheduling approach.
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
 */

#include "mbse.h"
#include "mbseCuda.h"

/* Function pointer declaration for the the task specific threads */
typedef void *(*threadPool_t)(void *);

/* @brief Threadpool declaring the function pointer for all the tasks*/
static threadPool_t threadPool[MBSE_NUMBER_OF_THREADS] =
                            {objDetectGetObject, objDetectStructureFromMotion,
                                    pathPlannerCanBusPolling, computeOSOverhead,
                                    computeSpeedAndSteer, pathPlannerCalculation};

/**
 * @brief Function to start all the task threads.
 *
 * The functions initializes and creates all the task threads mentioned in the thread pool.
 * In case of any error it prints the error message with an error code.
 * @return void
 */
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

        /* And finally start the actual threads */
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


/* Entry point of the application. The application does not need any
 * mandatory arguments. */
int main(int argc, char *argv[])
{
    uint8_t userPrivilege;
    /* Check whether the application is started as root. Thread scheduling,
     * assigning cores and priorities require root privileges. */
    userPrivilege = getuid();
    if(userPrivilege != 0)
    {
        fprintf(stdout, "The current user is not root\n");
        fflush(stdout);
        exit(1);
    }

    /* Get the CUDA properties */
    getCudaDeviceProperties();
    /* Start spawning threads */
    startRealTimeThreads();
    return 0;
}

