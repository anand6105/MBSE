# MBSE
This repository is used for the development and analysis of WATERS CHALLENGE 2019.

# Copyright
Copyright (C) 2020 Dortmund University of Applied Sciences and Arts and Others.

# Description
This project is part of the implementation of 2019 Waters Challenge. The processing power offered by GPUs and their capability to execute parallel workloads is exploited to execute and accelerate applications related to advanced driver assistance systems.                                              
                                                                                 
The challenge consists in analytically master the complex HW/SW system (that will be available as Amalthea model) to answer the following questions:          
                                                                                 
Response Time Computation:                                                       
 * Given an application consisting of a set of dependent tasks and a given mapping, calculate its end-to-end response time.                       
 * The response time should account for the time for the copy engine to transfer data between the CPU and the GPU.                   
 * The response time should account for the time for the data transfers between the CPU and the shared main memory considering a read/execute/write semantic.
 * Optionally the off-loading mechanism (synchronous/asynchronous) can be changed to further optimize the end-to-end latencies. 
