//  ================================================================
//  Created by Gregory Kramida on 2/5/20.
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
#include "../../Common/CheckBlockVisibility.h"
#include "../../../Utils/CUDAUtils.h"

namespace {
//CUDA kernels

__global__ void setVisibleEntriesToVisibleAtPreviousFrameAndUnstreamed(HashBlockVisibility* hash_block_visibility_types,
                                                                       const int* utilized_block_hash_codes,
                                                                       int utilized_entry_count) {
	int entryId = threadIdx.x + blockIdx.x * blockDim.x;
	if (entryId > utilized_entry_count - 1) return;
	hash_block_visibility_types[utilized_block_hash_codes[entryId]] = VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED;
}


template<bool useSwapping>
__global__ void
buildVisibilityList_device(HashEntry* hashTable, ITMLib::ITMHashSwapState* swapStates, int hash_entry_count,
                           int* visibleEntryIDs, int* visible_block_count, HashBlockVisibility* blockVisibilityTypes,
                           Matrix4f M_d, Vector4f projParams_d, Vector2i depthImgSize, float voxelSize) {
	int hash_code = threadIdx.x + blockIdx.x * blockDim.x;
	if (hash_code >= hash_entry_count) return;

	__shared__ bool shouldPrefix;
	shouldPrefix = false;
	__syncthreads();

	HashBlockVisibility hash_block_visibility_type = blockVisibilityTypes[hash_code];
	const HashEntry& hashEntry = hashTable[hash_code];

	if (hash_block_visibility_type == VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED) {
		bool isVisibleEnlarged, isVisible;

		if (useSwapping) {
			checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize,
			                           depthImgSize);
			if (!isVisibleEnlarged) hash_block_visibility_type = INVISIBLE;
		} else {
			checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize,
			                            depthImgSize);
			if (!isVisible) hash_block_visibility_type = INVISIBLE;
		}
		blockVisibilityTypes[hash_code] = hash_block_visibility_type;
	}

	if (hash_block_visibility_type > 0) shouldPrefix = true;

	if (useSwapping) {
		if (hash_block_visibility_type > 0 && swapStates[hash_code].state != 2) swapStates[hash_code].state = 1;
	}

	__syncthreads();

	if (shouldPrefix) {
		int offset = computePrefixSum_device<int>(hash_block_visibility_type > 0, visible_block_count,
		                                          blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) visibleEntryIDs[offset] = hash_code;
	}

}


} // end anonymous namespace (CUDA kernels)