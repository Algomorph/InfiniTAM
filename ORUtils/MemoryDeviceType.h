// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

//TODO: instead of having a separate "MemoryDeviceType" in the Configuration and a "MemoryDeviceType" here, just have a single,
// unified enum for MemoryDeviceType here.

/**
 * \brief The values of this enumeration denote the different types of memory device on which code may be running.
 */
enum MemoryDeviceType {
	MEMORYDEVICE_CPU,
	MEMORYDEVICE_CUDA,
	MEMORYDEVICE_METAL,
	MEMORYDEVICE_NONE
};

enum MemoryCopyDirection {
	CPU_TO_CPU, CPU_TO_CUDA, CUDA_TO_CPU, CUDA_TO_CUDA
};

inline
MemoryCopyDirection DetermineMemoryCopyDirection(MemoryDeviceType targetMemoryType, MemoryDeviceType sourceMemoryType){
	if(targetMemoryType == sourceMemoryType){
		switch(targetMemoryType){
			default:
			case MEMORYDEVICE_CPU:
				return CPU_TO_CPU;
			case MEMORYDEVICE_CUDA:
				return CUDA_TO_CUDA;
		}
	}else{
		switch(targetMemoryType){
			default:
			case MEMORYDEVICE_CPU:
				return CUDA_TO_CPU;
			case MEMORYDEVICE_CUDA:
				return CPU_TO_CUDA;
		}
	}
}

