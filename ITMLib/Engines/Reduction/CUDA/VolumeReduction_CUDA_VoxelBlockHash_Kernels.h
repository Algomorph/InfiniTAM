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

//local
#include "../Shared/ValueAndIndex.h"
#include "../../../Utils/CLionCudaSyntax.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"

using namespace ITMLib;

namespace { // (CUDA global kernels)

template<typename TVoxel, typename TRetrieveSingleFunctor, typename TReduceFunctor, typename TOutput>
__global__ void
computeVoxelHashReduction_BlockLevel(ValueAndIndex<TOutput>* block_results, const TVoxel* voxels, const HashEntry* hash_entries,
			                          const int* utilized_hash_codes) {
	__shared__ ValueAndIndex<TOutput> shared_data[VOXEL_BLOCK_SIZE3 / 2];
	unsigned int thread_id = threadIdx.x;
	unsigned int i_utilized_hash_block = blockIdx.x;
	int hash_code = utilized_hash_codes[i_utilized_hash_block];
	const TVoxel* block = voxels + (hash_entries[hash_code].ptr * VOXEL_BLOCK_SIZE3);
	unsigned int index_within_block1 = thread_id;
	unsigned int index_within_block2 = thread_id + blockDim.x;

	shared_data[thread_id] = TReduceFunctor::reduce(
			{TRetrieveSingleFunctor::retrieve(block[index_within_block1]), index_within_block1},
			{TRetrieveSingleFunctor::retrieve(block[index_within_block2]), index_within_block2});
	__syncthreads();

	for (unsigned int step = blockDim.x / 2u; step > 0u; step >>= 1u) {
		if (thread_id < step) {
			shared_data[thread_id] = TReduceFunctor::reduce(shared_data[thread_id],
			                                                shared_data[thread_id + step]);
		}
		__syncthreads();
	}

	if (thread_id == 0) block_results[i_utilized_hash_block] = shared_data[0];
}

} // end anonymous namespace (CUDA global kernels)