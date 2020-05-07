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

static struct timespec ts[MBSE_NUMBER_OF_THREADS];


/* Function to set the priority of the thread */
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



void utilInitializeTimer(timerTask timer)
{
    clock_getres(CLOCK_MONOTONIC, &ts[timer]);
    fprintf(stdout, "Timer thread started. System resolution: %ld ns\n", ts[timer].tv_nsec);
    fflush(stdout);

}

/* Add delay in milliseconds */
void utilAddDelay(uint32_t ms, timerTask timer)
{
    struct timespec deadline;
    /* Divide by 1000 to convert milliseconds to seconds.*/
    deadline.tv_sec = (ms / 1000);
    /* Convert milliseconds to nanoseconds*/
    deadline.tv_nsec = (ms * 1000 * 1000) % (1000 * 1000 * 1000);
    clock_nanosleep(CLOCK_MONOTONIC, 0, &deadline, NULL);
}

/* Function to print error messages to identify error points. */
void error(int at)
{
    /* Just exit on error */
    fprintf(stderr, "Some error occured at %d", at);
    fflush(stderr);
    exit(1);
}
