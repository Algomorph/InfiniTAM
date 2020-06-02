//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/31/20.
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

#include "MemoryDeviceType.h"
#include "PlatformIndependentAtomics.h"

namespace ORUtils {

constexpr int parallel_prefix_sum_block_size = 256;

template<MemoryDeviceType TMemoryDeviceType>
struct ParallelSum;

//TODO: refactor method signatures to flip arguments after #225 is resolved
template<>
struct ParallelSum<MEMORYDEVICE_CPU>{
	template<typename TElement>
	static inline TElement Add(TElement addend, ATOMIC_ARGUMENT(TElement) sum){
		return ATOMIC_ADD(sum, addend);
	}
	template<typename TElement>
	static inline TElement Add1D(TElement addend, ATOMIC_ARGUMENT(TElement) sum){
		return ATOMIC_ADD(sum, addend);
	}
	template<typename TElement>
	static inline TElement Add2D(TElement addend, ATOMIC_ARGUMENT(TElement) sum){
		return ATOMIC_ADD(sum, addend);
	}
	template<typename TElement>
	static inline TElement Add3D(TElement addend, ATOMIC_ARGUMENT(TElement) sum){
		return ATOMIC_ADD(sum, addend);
	}
};

////_DEBUG alloc
//#define __CUDACC__
//#define __device__
//#define __shared__
//static int dummy() {}
//#define __syncthreads dummy

#if !defined(COMPILE_WITHOUT_CUDA) && defined(__CUDACC__)

template <typename T>
__device__ inline static int ComputePrefixSum_MBS256(T element, T *sum, int thread_id)
{
	__shared__ unsigned int prefix_buffer[parallel_prefix_sum_block_size];
	__shared__ unsigned int prior_thread_group_sum; // "groupOffset" in legacy InfiniTAM/ORUtils code.

	prefix_buffer[thread_id] = element;
	__syncthreads();

	int s1, s2;

	//for localId == 0x00000001 (1), it. 0: prefix_buffer[1] += prefix_buffer[0] (final result: prefix_buffer[1] + prefix_buffer[0])
	//for localId == 0x00000011 (3), it. 0: prefix_buffer[3] += prefix_buffer[2]
	//                               it. 1: prefix_buffer[3] += prefix_buffer[1] (final result: sum of prefix_buffer from 0 to 3)
	//for localId == 0x00000101 (5), it. 0: prefix_buffer[5] += prefix_buffer[4] (final result: prefix_buffer[4] + prefix_buffer[5])
	//for localId == 0x00000111 (7), it. 0: prefix_buffer[7] += prefix_buffer[6]
	//                               it. 1: prefix_buffer[7] += prefix_buffer[5]
	//                               it. 2: prefix_buffer[7] += prefix_buffer[3] (final result: sum of prefix_buffer from 0 to 8)
	//for localId == 0x00001001 (9), it. 0: prefix_buffer[9] += prefix_buffer[8] (final result: prefix_buffer[8] + prefix_buffer[9])
	for (s1 = 1, s2 = 1; s1 < parallel_prefix_sum_block_size; s1 <<= 1)
	{
		s2 |= s1;
		if ((thread_id & s2) == s2) prefix_buffer[thread_id] += prefix_buffer[thread_id - s1];
		__syncthreads();
	}
	// at this point, we must have s1 == localSize >> 1 [s1 == 0x10000000 for localSize == 256],
	// and last s2 |= s1 means s2 is now a string of 1-bits up to localSize >> 1 [s2 == 0x11111111 for localSize == 256]
	// s1 >> 2 == 0x00100000
	// s2 >> 1 == 0x01111111
	// for next loop, when s1 == 0x00000001 (last iteration), s2 == 0x00000011
	// for localId == 0x00000011 (3), last iteration will do prefix_buffer[4] += prefix_buffer[3], final result for prefix_buffer[4]: sum of orig. prefix_buffer from 0 to 4
	// for localId == 0x00000111 (7), last iteration will do prefix_buffer[8] += prefix_buffer[7], final result for prefix_buffer[8]: sum of orig. prefix_buffer from 0 to 8
	//                                previous-to-last iteration will do prefix_buffer[9] += prefix_buffer[7], final result for prefix_buffer[9]: sum of orig. prefix_buffer from 0 to 9
	// for localId == 0x00001111 (15), last iteration will do prefix_buffer[16] += prefix_buffer[15], ... you get the idea.
	//                                 previous-to-last iteration will do prefix_buffer[17] += prefix_buffer[15],
	//                                 prev-prev-to-last iteration will do prefix_buffer[19] += prefix_buffer[15]
	for (s1 >>= 2, s2 >>= 1; s1 >= 1; s1 >>= 1, s2 >>= 1)
	{
		//for localSize = 256, if localId = 255, then localId + 1 goes outside the block
		// (and s2 of 0x1 would trigger that), hence, skip it
		if (thread_id != parallel_prefix_sum_block_size - 1 && (thread_id & s2) == s2) prefix_buffer[thread_id + s1] += prefix_buffer[thread_id];
		__syncthreads();
	}
	// at this point, prefix_buffer[localSize-1] has to contain the sum of the whole "group",
	// i.e. sum of elements from x localSize (256?) threads
	// then, groupOffset is the sum across the blocks of all elements before this "group", i.e. thread block
	if (thread_id == 0 && prefix_buffer[parallel_prefix_sum_block_size - 1] > 0) prior_thread_group_sum = atomicAdd(sum, prefix_buffer[parallel_prefix_sum_block_size - 1]);
	__syncthreads();

	int current_sum;// = prior_thread_group_sum + prefix_buffer[localId] - 1;
	// the final "offset", i.e. sum up to this element, is the current "group offset"
	// (sum of elements from "prior" blocks/groups) plus the sum of elements up to and including
	// the thread within the current block.
	if (thread_id == 0) {
		if (prefix_buffer[thread_id] == 0) current_sum = -1;
		else current_sum = prior_thread_group_sum;
	} else {
		if (prefix_buffer[thread_id] == prefix_buffer[thread_id - 1]) current_sum = -1;
		else current_sum = prior_thread_group_sum + prefix_buffer[thread_id - 1];
	}

	return current_sum;
}


template<>
struct ParallelSum<MEMORYDEVICE_CUDA>{
	template<typename TElement>
	__device__
	static inline TElement Add(TElement addend, ATOMIC_ARGUMENT(TElement) sum){
		int thread_id = threadIdx.x + threadIdx.y * blockDim.x + threadIdx.z * (blockDim.x * blockDim.y);
		return ComputePrefixSum_MBS256(addend, sum, thread_id);
	}
	template<typename TElement>
	__device__
	static inline TElement Add1D(TElement addend, ATOMIC_ARGUMENT(TElement) sum){
		int thread_id = threadIdx.x;
		return ComputePrefixSum_MBS256(addend, sum, thread_id);
	}
	template<typename TElement>
	__device__
	static inline TElement Add2D(TElement addend, ATOMIC_ARGUMENT(TElement) sum){
		int thread_id = threadIdx.x + threadIdx.y * blockDim.x;
		return ComputePrefixSum_MBS256(addend, sum, thread_id);
	}
	template<typename TElement>
	__device__
	static inline TElement Add3D(TElement addend, ATOMIC_ARGUMENT(TElement) sum){
		int thread_id = threadIdx.x + threadIdx.y * blockDim.x + threadIdx.z * (blockDim.x * blockDim.y);
		return ComputePrefixSum_MBS256(addend, sum, thread_id);
	}
};
#endif // #ifndef COMPILE_WITHOUT_CUDA !defined(COMPILE_WITHOUT_CUDA) && defined(__CUDACC__)

} // namespace ORUtils