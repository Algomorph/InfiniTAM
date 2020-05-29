//  ================================================================
//  Created by Gregory Kramida on 2/25/20.
//  Copyright (c) 2020 Gregory Kramida
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
#pragma once
//TODO: make Boost implicit converters (?) / overload constructors

#include <vector>
#include <unordered_set>
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib {
template<typename T>
inline std::vector<T>
ORUtils_MemoryBlock_to_std_vector(const ORUtils::MemoryBlock<T>& block, MemoryDeviceType device_type) {
	std::vector<T> vector(block.size());
	switch (device_type) {
		case MEMORYDEVICE_CPU:
			memcpy(vector.data(), block.GetData(MEMORYDEVICE_CPU), vector.size() * sizeof(T));
			break;
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			ORcudaSafeCall(cudaMemcpy(vector.data(), block.GetData(MEMORYDEVICE_CUDA), vector.size() * sizeof(T),
			                          cudaMemcpyDeviceToHost));
#else
			DIEWITHEXCEPTION_REPORTLOCATION("Not supported without compilation with CUDA.");
#endif
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
	}
	return vector;
}

template<typename T>
inline std::vector<T>
ORUtils_MemoryBlock_to_std_vector(const ORUtils::MemoryBlock<T>& block, MemoryDeviceType device_type, int count) {
	std::vector<T> vector(count);
	switch (device_type) {
		case MEMORYDEVICE_CPU:
			memcpy(vector.data(), block.GetData(MEMORYDEVICE_CPU), count * sizeof(T));
			break;
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			ORcudaSafeCall(cudaMemcpy(vector.data(), block.GetData(MEMORYDEVICE_CUDA), count * sizeof(T),
			                          cudaMemcpyDeviceToHost));
#else
			DIEWITHEXCEPTION_REPORTLOCATION("Not supported without compilation with CUDA.");
#endif
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
	}
	return vector;
}

template<typename T>
inline std::unordered_set<T>
ORUtils_MemoryBlock_to_std_unordered_set(const ORUtils::MemoryBlock<T>& block, MemoryDeviceType device_type) {
	std::vector<T> vector = ORUtils_MemoryBlock_to_std_vector(block, device_type);
	std::unordered_set<T> set(vector.begin(), vector.end());
	return set;
}

template<typename T>
inline std::unordered_set<T>
ORUtils_MemoryBlock_to_std_unordered_set(const ORUtils::MemoryBlock<T>& block, MemoryDeviceType device_type, int count) {
	std::vector<T> vector = ORUtils_MemoryBlock_to_std_vector(block, device_type, count);
	std::unordered_set<T> set(vector.begin(), vector.end());
	return set;
}

template<typename T>
inline std::vector<T>
raw_block_to_std_vector(const T* block, MemoryDeviceType device_type, int element_count) {
	std::vector<T> vector(element_count);
	switch (device_type) {
		case MEMORYDEVICE_CPU:
			memcpy(vector.data(), block, element_count * sizeof(T));
			break;
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			ORcudaSafeCall(cudaMemcpy(vector.data(), block, element_count * sizeof(T), cudaMemcpyDeviceToHost));
#else
			DIEWITHEXCEPTION_REPORTLOCATION("Not supported without compilation with CUDA.");
#endif
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
	}
	return vector;
}

template<typename T>
inline std::unordered_set<T>
raw_block_to_std_unordered_set(const T* block, MemoryDeviceType device_type, int element_count) {
	std::vector<T> vector = raw_block_to_std_vector(block, device_type, element_count);
	std::unordered_set<T> set(vector.begin(), vector.end());
	return set;
}

template<typename T>
static ORUtils::MemoryBlock<T>
std_vector_to_ORUtils_MemoryBlock(std::vector<T> vector, MemoryDeviceType memoryDeviceType) {
	ORUtils::MemoryBlock<T> block(vector.size(), memoryDeviceType == MEMORYDEVICE_CPU,
	                              memoryDeviceType == MEMORYDEVICE_CUDA);
	switch (memoryDeviceType) {
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			ORcudaSafeCall(cudaMemcpy(block.GetData(MEMORYDEVICE_CUDA), vector.data(), vector.size() * sizeof(T),
			                          cudaMemcpyHostToDevice));
#else
			DIEWITHEXCEPTION_REPORTLOCATION("Not supported without compilation with CUDA.");
#endif
			break;
		case MEMORYDEVICE_CPU:
			memcpy(block.GetData(MEMORYDEVICE_CPU), vector.data(), vector.size() * sizeof(T));
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Device type not supported by operation.");
	}
	return block;
}



} // namespace ITMLib