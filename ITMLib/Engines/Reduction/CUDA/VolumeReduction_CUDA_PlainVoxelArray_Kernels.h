//  ================================================================
//  Created by Gregory Kramida on 2/26/20.
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

#include "../Shared/ReductionResult.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"

using namespace ITMLib;

namespace { // anonymous namespace (CUDA kernels)

template<typename TOutput>
__global__
void
setTailToIgnored(ReductionResult<TOutput, PlainVoxelArray>* results, const int from_index, const int result_count,
                 const ReductionResult<TOutput, PlainVoxelArray> ignored_value) {
	unsigned int result_idx = threadIdx.x + from_index;
	if (result_idx > result_count) return;
	results[result_idx] = ignored_value;
}


template<typename TReduceStaticFunctor, typename TVoxel, typename TOutput, typename TRetrieveFunction>
__global__
void
computeVoxelHashReduction_BlockLevel(
		ReductionResult<TOutput, PlainVoxelArray>* block_results,
		const TVoxel* voxels,
		TRetrieveFunction retrieve_function
) {
	__shared__
	ReductionResult<TOutput, PlainVoxelArray> shared_data[(VOXEL_BLOCK_SIZE3 / 2)];

	unsigned int thread_id = threadIdx.x;
	unsigned int block_offset = blockIdx.x * VOXEL_BLOCK_SIZE3;
	unsigned int index1 = block_offset + thread_id;
	unsigned int index2 = index1 + blockDim.x;

	shared_data[thread_id] = TReduceStaticFunctor::reduce(
			{retrieve_function(voxels, index1), index1},
			{retrieve_function(voxels, index2), index2}
	);
	__syncthreads();

	for (unsigned int step = blockDim.x / 2u; step > 0u; step >>= 1u) {
		if (thread_id < step) {
			shared_data[thread_id] = TReduceStaticFunctor::reduce(shared_data[thread_id],
			                                                      shared_data[thread_id + step]);
		}
		__syncthreads();
	}

	if (thread_id == 0) block_results[blockIdx.x] = shared_data[0];
}

template<typename TReduceFunctor, typename TOutput>
__global__
void computeVoxelHashReduction_ResultLevel(ReductionResult<TOutput, PlainVoxelArray>* output,
                                           const ReductionResult<TOutput, PlainVoxelArray>* input) {
	__shared__
	ReductionResult<TOutput, PlainVoxelArray> shared_data[VOXEL_BLOCK_SIZE3 / 2];
	unsigned int thread_id = threadIdx.x;
	unsigned int block_id = blockIdx.x;
	unsigned int base_level_index1 = block_id * blockDim.x * 2 + thread_id;
	unsigned int base_level_index2 = base_level_index1 + blockDim.x;
	shared_data[thread_id] = TReduceFunctor::reduce(input[base_level_index1], input[base_level_index2]);
	__syncthreads();
	for (unsigned int step = blockDim.x / 2u; step > 0u; step >>= 1u) {
		if (thread_id < step) {
			shared_data[thread_id] = TReduceFunctor::reduce(shared_data[thread_id], shared_data[thread_id + step]);
		}
		__syncthreads();
	}

	if (thread_id == 0) output[block_id] = shared_data[0];
}

} // end anonymous namespace (CUDA kernels)
