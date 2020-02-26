//  ================================================================
//  Created by Gregory Kramida on 2/21/20.
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

#include "../../CLionCudaSyntax.h"
#include "../../Math.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../WarpType.h"
#include "../Statistics.h"

struct StatisticAndLocation {
	float statistic;
	Vector3i location;
};

template<typename T>
struct ValueAndIndex {
	T value;
	unsigned int index_within_block;
};

template<typename TVoxel, ITMLib::WarpType TWarpType>
struct TRetreiveWarpLengthFunctor;

template<typename TVoxel>
struct TRetreiveWarpLengthFunctor<TVoxel, ITMLib::WARP_UPDATE>{
public:
	_CPU_AND_GPU_CODE_
	inline static float retrieve(const TVoxel& voxel){
		return ORUtils::length(voxel.warp_update);
	}
};

template<typename TVoxel, ITMLib::WarpType TWarpType, ITMLib::Statistic TStatistic>
struct TReduceWarpLengthStatisticFunctor;

template<typename TVoxel>
struct TReduceWarpLengthStatisticFunctor<TVoxel, ITMLib::WARP_UPDATE, ITMLib::MAXIMUM>{
public:
	_CPU_AND_GPU_CODE_
	inline static const ValueAndIndex<float>& reduce(const ValueAndIndex<float>& tuple1, const ValueAndIndex<float>& tuple2){
		return (tuple1.value < tuple2.value) ? tuple2 : tuple1;
	}
};

namespace { // (CUDA global kernels)


template<typename TVoxel, typename TRetrieveSingleFunctor, typename TReduceFunctor, typename TOutput>
__global__ void
computeVoxelHashReduction(ValueAndIndex<TOutput>* block_results, const TVoxel* voxels, const HashEntry* hash_entries,
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
