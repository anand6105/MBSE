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
 * @file cuDASM.cu
 * @author Anand Prakash
 * @date 12 May 2020
 * @brief This file contains the CUDA kernel implementation of the DASM
 *        and OS overhead tasks.
 *
 * This file implements the runnables and used by the DASM and OS Overhead tasks.
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
 */


extern "C"{
    #include "mbseCuda.h"
}


// Kernel that executes on the CUDA device to compute the speed and steer for DASM task.
__global__ void computeDASM( int *devSpeed, int *devSteer, int step, int size )
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    int speed, steer = 0;
    while (tid < size) {
        steer = devSteer[tid];
        speed = devSpeed[tid];
        devSpeed[tid] += (steer * step);
        devSteer[tid] += (speed * step);
        tid += blockDim.x * gridDim.x;
    }
}

/**
 * @brief Function to process the DASM task.
 *
 * The functions process the data received from the planner task and generate the steer and speed
 * output. The task consists of one runnable.
 *
 * @param func       Function name
 * @param steer      Pointer to input steer from planner. Data is modified after processing.
 * @param speed      Pointer to input speed from planner. Data is modified after processing.
 *
 * @return void
 */
extern "C"
void cuProcessDASM(const char *func, int *steer, int *speed)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;
    cudaEvent_t start, stop;
    int *devSpeed, *devSteer = NULL;
    /* Select random length and thread block */
    const int length = 768 * 1024;
    const int size = length * sizeof(int);
    const int threadPerBlock = 8;
    int blockPerGrid = ((length + threadPerBlock - 1) / threadPerBlock);
    int index = 0;
    // Set random step size. Set to 3
    int step = 3;

    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    cudaMalloc((void **) &devSpeed, size);  // Allocate array on device
    cudaMalloc((void **) &devSteer, size);  // Allocate array on device
    // Initialize array in device to 0
    for(index = 0; index < length; index++){
        cudaMemset(&devSpeed[index], *steer, sizeof(int));
        cudaMemset(&devSteer[index], *speed, sizeof(int));
    }

    cudaEventRecord(start, 0);

    computeDASM <<<blockPerGrid, threadPerBlock>>> (devSpeed, devSteer, step, length); // call CUDA kernel
    cudaDeviceSynchronize();

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float elapsedTime = 0.0f;
    cudaEventElapsedTime(&elapsedTime, start, stop);
    fprintf(stdout,"DASM task is %4.5f ms \n", elapsedTime);

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(steer, devSteer, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to write can data to host memory (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    err = cudaMemcpy(speed, devSpeed, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to write can data to host memory (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    cudaFree(devSpeed);
    cudaFree(devSteer);
}


// Kernel that executes on the CUDA device the OS overhead task.
__global__ void osOverhead(int * A, int * B, int * C, int size) {

    int ROW = blockIdx.y*blockDim.y+threadIdx.y;
    int COL = blockIdx.x*blockDim.x+threadIdx.x;

    int tmpSum = 0;

    if (ROW < size && COL < size) {
        // each thread computes one element of the block sub-matrix
        for (int i = 0; i < size; i++) {
            tmpSum += A[ROW * size + i] * B[i * size + COL];
        }
    }
    C[ROW * size + COL] = tmpSum;
}


/**
 * @brief Function to process the OS overhead task.
 *
 * The functions adds some OS overhead to simulate the real scenario.
 *
 * @param func       Function name
 *
 * @return void
 */
void addOSOverhead(const char * func)
{
    int *devRow, *devCol, *devResult = NULL;
    // Select some random row and col value.
    int row = 1024;
    int col = row;
    int numOfElements = row * col;
    cudaEvent_t start, stop;
    int size = numOfElements * sizeof(int);

    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaMalloc((void**) &devRow, size);
    cudaMalloc((void**) &devCol, size);
    // copy host memory to device
    /* Use some random value to do the calculation */
    int matrixA = rand() % 83;
    int matrixB = rand() % 23;
    cudaMemset(devRow, matrixA, size);
    cudaMemset(devCol, matrixB, size);

    cudaMalloc((void**) &devResult, size);
    // use 4 threads per block
    int threadsPerBlock = 4;
    int blocksPerGrid =(numOfElements + threadsPerBlock - 1) / threadsPerBlock;
    cudaEventRecord(start, 0);
    osOverhead<<<blocksPerGrid,threadsPerBlock>>>(devRow, devCol, devResult, row);
    cudaDeviceSynchronize();

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float elapsedTime = 0.0f;
    cudaEventElapsedTime(&elapsedTime, start, stop);
    fprintf(stdout,"OS overhead task is %4.5f ms \n", elapsedTime);
    cudaFree(devRow);
    cudaFree(devCol);
    cudaFree(devResult);

}
