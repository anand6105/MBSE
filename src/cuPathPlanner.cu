/*
 ************************************************************************************
 *   Copyright (c) 2020 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts -
 *        initial API and implementation
 ************************************************************************************
 * cuPathPlanner.cu
 *
 *  Created on: May 12, 2020
 *      Author: Anand Prakash
 */

extern "C"{
    #include "mbseCuda.h"
}


#define NUM_BLOCK    8 // Number of thread blocks
#define NUM_THREAD  64  // Number of threads per block

// Kernel that executes on the CUDA device
__global__ void getCanBusData(int *canData, int *val, int nthreads, int nblocks) {
    int i;
    int idx = blockIdx.x*blockDim.x+threadIdx.x;  // Sequential thread index across the blocks
    for (i=idx; i< 100000; i+=nthreads*nblocks) {
        canData[idx] += val[idx];
    }
}

/* Function to get the CAN Bus polling data */
extern "C"
void cuPlannerFetchCanBusData(const char *func, int *hostCanPollingData)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;
    int *canHost, *fetchVal = NULL;
    dim3 dimGrid(NUM_BLOCK,1,1);  // Grid dimensions
    dim3 dimBlock(NUM_THREAD,1,1);  // Block dimensions
    size_t size = NUM_BLOCK*NUM_THREAD*sizeof(int);  //Array memory size

    cudaMalloc((void **) &canHost, size);  // Allocate array on device
    cudaMalloc((void **) &fetchVal, size);  // Allocate array on device
    // Initialize array in device to 0
    cudaMemset(canHost, *hostCanPollingData, size);
    cudaMemset(fetchVal, 1, size);

    getCanBusData <<<dimGrid, dimBlock>>> (canHost, fetchVal, NUM_THREAD, NUM_BLOCK); // call CUDA kernel
    cudaDeviceSynchronize();

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(hostCanPollingData, canHost, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to write can data to host memory (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    cudaFree(canHost);
    cudaFree(fetchVal);
}



// Kernel that executes on the CUDA device
__global__ void pathPlan( int *devSpeed, int *devSteer, int size )
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    while (tid < size) {
        devSpeed[tid] += 1;
        devSteer[tid] += 1;
        tid += blockDim.x * gridDim.x;
    }
}

/* Function to get the CAN Bus polling data */
extern "C"
void cuPlannerInterpolatePath(const char *func, plannerData *data)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;
    int *devSpeed = NULL, *devSteer = NULL;
    const int length = 33 * 1024;
    const int threadPerBlock = 256;
    int blockPerGrid = minVal(32, ((length + threadPerBlock - 1) / threadPerBlock));
    plannerData *planner = data;
    int index = 0;

    cudaMalloc((void **) &devSpeed, length);  // Allocate array on device
    cudaMalloc((void **) &devSteer, length);  // Allocate array on device
    // Initialize array in device to 0
    for(index = 0; index < length; index++){
        cudaMemset(devSpeed, planner->speedObjective + planner->matrixSFM, sizeof(int));
        cudaMemset(devSteer, planner->steerObjective + planner->canData, sizeof(int));
    }


    pathPlan <<<blockPerGrid, threadPerBlock>>> (devSpeed, devSteer, length); // call CUDA kernel
    cudaDeviceSynchronize();
    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(&planner->speedObjective, devSpeed, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to write Speed data to host memory (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    err = cudaMemcpy(&planner->steerObjective, devSteer, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to write Steer data to host memory (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    cudaFree(devSpeed);
    cudaFree(devSteer);
}
