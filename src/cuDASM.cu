/*
 ************************************************************************************
 *   Copyright (c) 2020 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts -
 *        initial API and implementation
 ************************************************************************************
 * cuDASM.cu
 *
 *  Created on: May 12, 2020
 *      Author: Anand Prakash
 */
extern "C"{
    #include "mbseCuda.h"
}


// Kernel that executes on the CUDA device
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

/* Function to get the CAN Bus polling data */
extern "C"
void cuProcessDASM(const char *func, int *steer, int *speed)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;
    int *devSpeed, *devSteer = NULL;
    const int length = 50 * 1024;
    const int threadPerBlock = 256;
    int blockPerGrid = minVal(32, ((length + threadPerBlock - 1) / threadPerBlock));
    int index = 0;
    int step = 3;

    cudaMalloc((void **) &devSpeed, length);  // Allocate array on device
    cudaMalloc((void **) &devSteer, length);  // Allocate array on device
    // Initialize array in device to 0
    for(index = 0; index < length; index++){
        cudaMemset(devSpeed, *steer, sizeof(int));
        cudaMemset(devSteer, *speed, sizeof(int));
    }


    computeDASM <<<blockPerGrid, threadPerBlock>>> (devSpeed, devSteer, step, length); // call CUDA kernel
    cudaDeviceSynchronize();

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


void addOSOverhead(const char * func)
{
    int *devRow, *devCol, *devResult = NULL;
    int row = 512;
    int col = 512;
    int size = row * col;
    cudaMalloc((void**) &devRow, size);
    cudaMalloc((void**) &devCol, size);
    // copy host memory to device
    /* Use some random value to do the calculation */
    int matrixA = rand() % 83;
    int matrixB = rand() % 23;
    cudaMemset(devRow, matrixA, size);
    cudaMemset(devCol, matrixB, size);

    cudaMalloc((void**) &devResult, size);
    // declare the number of blocks per grid and the number of threads per block
    // use 1 to 512 threads per block
    dim3 threadsPerBlock(128, 1);
    dim3 blocksPerGrid(128, 1);


    osOverhead<<<blocksPerGrid,threadsPerBlock>>>(devRow, devCol, devResult, 128);
    cudaDeviceSynchronize();
    cudaFree(devRow);
    cudaFree(devCol);
    cudaFree(devResult);

}
