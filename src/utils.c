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
 * @file utils.c
 * @author Anand Prakash
 * @date 22 April 2020
 * @brief This file declares and implement the utility functions used in the application.
 *
 * It consists of functions setting the thread priority, adding delay to the tasks and
 * printing errors. All the C utility functions must be added to this file.
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
 */


#include "mbse.h"


/**
 * @brief Function to set the thread priority.
 *
 * This task sets the thread priority based on the threadId and the customPrio provided
 * The priority of the thread is set to max priority minus the custom priority
 *
 * @param[in] threadId      Thread ID whose priority needs to be set.
 * @param[in] customPrio    Priority offset of the thread from the maximum priority
 *
 * @return void
 */
void utilSetThreadPriority(pthread_t threadId, int customPrio)
{
    struct sched_param param;

    int threadPriority = sched_get_priority_max(SCHED_FIFO) - customPrio;
    /* Set real-time priority for this thread */
    param.sched_priority = threadPriority;
    if (pthread_setschedparam(threadId, SCHED_FIFO, &param) < 0)
    {
        perror("sched_setscheduler");
    }
}


/**
 * @brief Function to add delay to the task.
 *
 * This function adds delay to the task by sleeping for the time provided in the argument
 * in terms of millisecond.
 *
 * @param[in]    ms          Time in millisecond for which the thread needs to sleep.
 * @param[inout] deadline    Pointer to the timespec for the next deadline
 *
 * @return void
 */
void utilAddDelay(uint32_t ms, struct timespec *deadline)
{
    /* Calculate next deadline */
    deadline->tv_nsec += (ms * MILLI_SECONDS);
    deadline->tv_sec += deadline->tv_nsec / NSEC_PER_SEC;
    deadline->tv_nsec %= NSEC_PER_SEC;
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, deadline, NULL) != 0)
    {
        if (errno != EINTR)
        {
            /* error handling here */
            return;
        }
    }
}



/**
 * @brief Function to print error message thrown from a particular point in application code.
 *
 * This function prints the error code from which the error was thrown.
 *
 * @param[in] at  Error code.
 *
 * @return void
 */
void error(int at)
{
    /* Just exit on error */
    fprintf(stderr, "Some error occured at %d", at);
    fflush(stderr);
    exit(1);
}
