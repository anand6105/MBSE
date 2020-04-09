/*******************************************************************************
 *   Copyright (c) 2019 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts - initial API and implementation
 *******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <limits.h>
#include <stdint.h>
#include "mbse.h"


typedef void *(*threadPool_t)(void *);

static void *my_rt_thread(void *args);
static void *rt_thread(void *args);

static threadPool_t threadPool[MBSE_NUMBER_OF_THREADS] = {my_rt_thread, rt_thread};

static void setThreadPriority(int prio, int sched)
{
    struct sched_param param;
    // Set realtime priority for this thread
    param.sched_priority = prio;
    if (sched_setscheduler(0, sched, &param) < 0)
    {
        perror("sched_setscheduler");
    }
}



/* The thread to start */
static void *rt_thread(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct timespec deadline;
    struct sched_param param;
    uint16_t threadPriority = 0;
    /* Re-start the timer after 10 seconds */
    uint64_t threadDeadline = (uint64_t)(30 * MILLI_SECONDS);
    threadPriority = sched_get_priority_max(SCHED_FIFO);

    setThreadPriority(threadPriority, SCHED_FIFO);

    pthread_getschedparam(pthread_self(), &threadPolicy, &param);

    printf("I am a RT-thread with priority %i \n", param.sched_priority);

    clock_gettime(CLOCK_MONOTONIC, &(deadline));

    while(1)
    {
        uint32_t sum = 0;
        uint32_t prod = 0;
        /* Sleep for 5 milli second */
        deadline.tv_nsec += 5 * MILLI_SECONDS;
        if(deadline.tv_nsec >= threadDeadline) {
            deadline.tv_nsec -= threadDeadline;
            deadline.tv_sec++;
        }

        /* Execute some instructions. Do some RT-things here */
        for(executionCount = 0; executionCount < 10000; executionCount++)
        {
            // @TODO Do some random calculation. GPU calls to be made here.
            sum += 1;
            prod += (sum * 2);
        }
        /* wait 2 milliseconds before next thread cycle begins */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
    }

    return NULL;
}


/* The thread to start */
static void *my_rt_thread(void *args)
{
    uint32_t threadPolicy = 0;
    uint32_t executionCount = 0;
    struct timespec deadline;
    struct sched_param param;
    uint16_t threadPriority = 0;
    /* Re-start the timer after 10 seconds */
    uint64_t threadDeadline = (uint64_t)(10 * MILLI_SECONDS);
    threadPriority = sched_get_priority_max(SCHED_FIFO);

    setThreadPriority(threadPriority, SCHED_FIFO);

    pthread_getschedparam(pthread_self(), &threadPolicy, &param);

    printf("I am a RT-thread with priority %i \n", param.sched_priority);

    clock_gettime(CLOCK_MONOTONIC, &(deadline));

    while(1)
    {
        uint32_t sum = 0;
        uint32_t prod = 0;
        /* Sleep for 2 milli second */
        deadline.tv_nsec += 2 * MILLI_SECONDS;
        if(deadline.tv_nsec >= threadDeadline) {
            deadline.tv_nsec -= threadDeadline;
            deadline.tv_sec++;
        }

        /* Execute some instructions. Do some RT-things here */
        for(executionCount = 0; executionCount < 20000; executionCount++)
        {
            // @TODO Do some random calculation. GPU calls to be made here.
            sum += 1;
            prod += (sum * 2);
        }
        /* wait 2 milliseconds before next thread cycle begins */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
    }

    return NULL;
}

static void error(int at)
{
    /* Just exit on error */
    fprintf(stderr, "Some error occured at %d", at);
    fflush(stderr);
    exit(1);
}

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

        /* And finally start the actual thread */
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


int main(int argc, char *argv[])
{
    uint8_t userPrivilege;
    userPrivilege = getuid();
    if(userPrivilege != 0)
    {
        fprintf(stdout, "The current user is not root\n");
        fflush(stdout);
    }

    startRealTimeThreads();
    return 0;
}

