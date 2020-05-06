/*
 ************************************************************************************
 *   Copyright (c) 2020 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts -
 *        initial API and implementation
 ************************************************************************************
 * utils.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Anand Prakash
 */

#include "mbse.h"


/* Function to set the priority of the thread */
void setThreadPriority(pthread_t threadId, int customPrio)
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


/* Add delay in milliseconds */
void addDelay(uint32_t delay)
{
    struct timespec res;
    /* Divide by 1000 to convert milliseconds to seconds.*/
    res.tv_sec = (delay / 1000);
    /* Convert milliseconds to nanoseconds*/
    res.tv_nsec = (delay * 1000 * 1000) % (1000 * 1000 * 1000);
    clock_nanosleep(CLOCK_MONOTONIC, 0, &res, NULL);
}

/* Function to print error messages to identify error points. */
void error(int at)
{
    /* Just exit on error */
    fprintf(stderr, "Some error occured at %d", at);
    fflush(stderr);
    exit(1);
}
