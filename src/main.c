/*******************************************************************************
 *   Copyright (c) 2019 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts - initial API and implementation
 *******************************************************************************/
#define _GNU_SOURCE
#include <sched.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>

void Thread1()
{
    int j;
    int policy;
    int a = 0;
    int b = 0;

    struct sched_param param;
    pthread_getschedparam(pthread_self(),&policy,&param);

    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &(deadline));

    while(1)
    {

    deadline.tv_nsec += 4 * 1000 * 1000;
    if(deadline.tv_nsec >= 1000000000) {
        deadline.tv_nsec -= 1000000000;
        deadline.tv_sec++;
    }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
      printf("Thread 1 executing\n");
         for(j=1;j<60000;j++)
        {
             a += 1;
             b += a * 2;
             if ((j % 2000) == 0){
                 printf("Thread 1 executing count %d\n", j);
             }
        }
        printf("Thread 1 execution done\n");
        a = 0;
        b = 0;
    }

}

void Thread2()
{
    //sleep(1);
    int j;
    int policy;
    int a = 0;
    int b = 0;

    struct sched_param param;
    pthread_getschedparam(pthread_self(),&policy,&param);

    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &(deadline));

    while(1)
    {

    deadline.tv_nsec += 2 * 1000 * 1000;
    if(deadline.tv_nsec >= 1000000000) {
        deadline.tv_nsec -= 1000000000;
        deadline.tv_sec++;
    }

      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
      printf("Thread 2 executing\n");
         for(j=1;j<20000;j++)
        {
        a += 1;
        b += a * 2;
        if (j % 1000 == 0)
        {
            printf("Thread 2 executing count%d\n", j);
        }
        }
         printf("Thread 2 execution done\n");
    a = 0;
    b = 0;
    }

}

int main()
{
    int i;
    i = getuid();
    if(i==0)
        printf("The current user is root\n");
    else
        printf("The current user is not root\n");

    pthread_t ppid1, ppid2;
    struct sched_param param;

    pthread_attr_t attr1, attr2;
    i = pthread_attr_init(&attr1);
    if (i != 0)
	    printf("ATTR1 init failed\n");
    i = pthread_attr_init(&attr2);
    if (i != 0)
	    printf("ATTR2 init failed\n");

    cpu_set_t cpu;
    CPU_ZERO(&cpu);
    CPU_SET(1,&cpu);
    printf("CPU is set\n");
    pthread_attr_setaffinity_np(&attr1, sizeof(cpu_set_t),&cpu);
    pthread_attr_setaffinity_np(&attr2, sizeof(cpu_set_t),&cpu);

    int minprio = sched_get_priority_min(SCHED_FIFO);
    int maxprio = sched_get_priority_max(SCHED_FIFO);

    printf("Min prio=%d Max prio=%d\n", minprio, maxprio);

    param.sched_priority = 10;
    pthread_attr_setschedpolicy(&attr1,SCHED_FIFO);
    pthread_attr_setschedparam(&attr1,&param);
    pthread_attr_setinheritsched(&attr1,PTHREAD_EXPLICIT_SCHED);

    param.sched_priority = 15;
    pthread_attr_setschedpolicy(&attr2,SCHED_FIFO);
    pthread_attr_setschedparam(&attr2,&param);
    pthread_attr_setinheritsched(&attr2,PTHREAD_EXPLICIT_SCHED);
    printf("Creating threads\n");
    pthread_create(&ppid1,&attr1,(void *)Thread1,NULL);
    pthread_create(&ppid2,&attr2,(void *)Thread2,NULL);

    pthread_join(ppid1,NULL);
    pthread_join(ppid2,NULL);

    pthread_attr_destroy(&attr1);
    pthread_attr_destroy(&attr2);
    return 0;
}

