//  ================================================================
//  Created by Gregory Kramida on 12/13/19.
//  Copyright (c) 2019 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================

//stdlib
#include <iostream>
#include <functional>
#include <vector>
#include <algorithm>

//local
#include "CUDAAtomicTesting.h"
#include "../ORUtils/CUDADefines.h"
#include "../ORUtils/MemoryBlock.h"
#include "../ITMLib/Utils/CUDAUtils.h"
#include "TemporaryCUDA_Atomics.h"


namespace {

template<typename T>
__global__ void testAtomicCAS(T* data, T* output, int data_count, T expected, T new_value) {
	int index = threadIdx.x;
	T old_val = atomicCAS(data + index, expected, new_value);
	output[index + blockIdx.x * data_count] = old_val;
}

template<typename T>
__global__ void testAtomicAdd(T* data, T* output, int data_count) {
	int index = threadIdx.x;
	T old_val = atomicAdd(data + index, (T) 1);
	output[index + blockIdx.x * data_count] = old_val;
}

} // end anonymous namespace

template<typename T>
bool AtomicTest(std::function<void(dim3, dim3, T*, T*, int)> CUDA_function_call,
		std::function<bool(T*, T*, int, int)> resultVerificationProcedure, T initial_value = 0){
	const int data_count = 35;
	const int count_of_threads_to_modify_each_item = 99;
	dim3 block_size(data_count);
	dim3 grid_size(count_of_threads_to_modify_each_item);
	ORUtils::MemoryBlock<T> data(data_count, true, true);
	ORUtils::MemoryBlock<T> output(data_count * count_of_threads_to_modify_each_item, true, true);
	T* data_CPU = data.GetData(MEMORYDEVICE_CPU);
	for(int i = 0; i < data_count; i++){
		data_CPU[i] = initial_value;
	}

	std::cout << "Data before modification: " << std::endl;
	for (int i = 0; i < data_count; i++) {
		std::cout << (int) data_CPU[i] << ", ";
	}
	std::cout << std::endl;

	data.UpdateDeviceFromHost();
	CUDA_function_call(grid_size, block_size, data.GetData(MEMORYDEVICE_CUDA), output.GetData(MEMORYDEVICE_CUDA),
	                   data_count);
	ORcudaKernelCheck;
	data.UpdateHostFromDevice();
	output.UpdateHostFromDevice();

	std::cout << "Data after modification: " << std::endl;
	for (int i = 0; i < data_count; i++) {
		std::cout << (int) data_CPU[i] << ", ";
	}
	std::cout << std::endl;

	T* output_CPU = output.GetData(MEMORYDEVICE_CPU);
	std::cout << "Output (threads modifying the same value appear in the same column): " << std::endl;
	for (int iCompetingThread = 0; iCompetingThread < count_of_threads_to_modify_each_item; iCompetingThread++) {
		for (int iDataItem = 0; iDataItem < data_count; iDataItem++) {
			std::cout << (int) output_CPU[iDataItem + iCompetingThread * data_count] << ", ";
		}
		std::cout << std::endl;
	}
	return resultVerificationProcedure(data_CPU, output_CPU, data_count, count_of_threads_to_modify_each_item);
}

template<typename T>
std::vector<T> arange(T start, T stop, T step = 1) {
	std::vector<T> values;
	for (T value = start; value < stop; value += step)
		values.push_back(value);
	return values;
}

template<typename T>
bool AtomicAddTest(){
	return AtomicTest<T>(
			[](dim3 grid_size, dim3 block_size, T* data, T* output, int data_count) {
				testAtomicAdd << < grid_size, block_size >> > (data, output, data_count);
			},
			[](T* data, T* output, int data_count, int count_of_threads_to_modify_each_item){
				std::vector<T> expected = arange((T)0, (T)count_of_threads_to_modify_each_item, (T)1);
				for (int iDataItem = 0; iDataItem < data_count; iDataItem++) {
					std::vector<T> competing_thread_outputs;
					for (int iCompetingThread = 0; iCompetingThread < count_of_threads_to_modify_each_item; iCompetingThread++) {
						competing_thread_outputs.push_back(output[iDataItem + iCompetingThread * data_count]);
					}
					std::sort(competing_thread_outputs.begin(), competing_thread_outputs.end(), std::less<T>());
					if(!std::equal(competing_thread_outputs.begin(), competing_thread_outputs.end(), expected.begin())) return false;
				}
				return true;
			},
			(T)0
	);
}

template<typename T>
bool AtomicCASTest(){
	double initial_value = 0;
	double final_value = 14;
	return AtomicTest<T>(
			[&](dim3 grid_size, dim3 block_size, T* data, T* output, int data_count) {
				testAtomicCAS << < grid_size, block_size >> > (data, output, data_count, (T)initial_value, (T)final_value);
			},
			[&](T* data, T* output, int data_count, int count_of_threads_to_modify_each_item){
				std::vector<T> expected;
				expected.push_back((T)initial_value);
				for(int i = 0; i < count_of_threads_to_modify_each_item - 1; i++){
					expected.push_back((T)final_value);
				}

				for (int iDataItem = 0; iDataItem < data_count; iDataItem++) {
					std::vector<T> competing_thread_outputs;
					for (int iCompetingThread = 0; iCompetingThread < count_of_threads_to_modify_each_item; iCompetingThread++) {
						competing_thread_outputs.push_back(output[iDataItem + iCompetingThread * data_count]);
					}
					std::sort(competing_thread_outputs.begin(), competing_thread_outputs.end(), std::less<T>());
					if(!std::equal(competing_thread_outputs.begin(), competing_thread_outputs.end(), expected.begin())) return false;
				}
				return true;
			},
			(T)initial_value
	);
}

bool AtomicAddCharTest() {
	return AtomicAddTest<char>();
}

bool AtomicAddShortTest() {
	return AtomicAddTest<short>();
}

bool AtomicCASCharTest(){
	return AtomicCASTest<char>();
}