/*
 * mbse.h
 *
 *  Created on: Apr 11, 2020
 *      Author: anand
 */

#ifndef MBSE_H_
#define MBSE_H_

#define MBSE_NUMBER_OF_THREADS       2
#define MBSE_THREAD_STACK_SIZE       (100 * 1024)      /* 100 kB is enough for now. */

#define MICRO_SECONDS                1000
#define MILLI_SECONDS                (MICRO_SECONDS * 1000)
#define SECONDS                      (MILLI_SECONDS * 1000)


void addTwoVectors(const char *function);

#endif /* MBSE_H_ */
