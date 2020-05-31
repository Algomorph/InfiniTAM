//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/26/20.
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
#include "../../Objects/Volume/VoxelBlockHash.h"
#include "../../Utils/Math.h"
#include "../../Utils/Geometry/CheckBlockVisibility.h"
#include "../../Utils/CUDAUtils.h"
#include "../../../ORUtils/PlatformIndependedParallelSum.h"

namespace {
//anonymous namespace (Cuda global kernels)
__global__ void
buildCompleteVisibleList_device(const ITMLib::HashEntry* hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/ int hashBlockCount,
                                int* visibleBlockHashCodes, int* visibleBlockCount, ITMLib::HashBlockVisibility* blockVisibilityTypes, Matrix4f M,
                                Vector4f projParams, Vector2i imgSize, float voxelSize) {
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > hashBlockCount - 1) return;

	__shared__ bool shouldPrefix;

	unsigned char hashVisibleType = 0; //blockVisibilityTypes[targetIdx];
	const ITMLib::HashEntry& hashEntry = hashTable[targetIdx];

	shouldPrefix = false;
	__syncthreads();

	if (hashEntry.ptr >= 0) {
		shouldPrefix = true;

		bool isVisible, isVisibleEnlarged;
		CheckVoxelHashBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M, projParams, voxelSize,
		                                     imgSize);

		hashVisibleType = isVisible;
	}

	if (hashVisibleType > 0) shouldPrefix = true;

	__syncthreads();

	if (shouldPrefix) {
		int offset = ORUtils::ParallelSum<MEMORYDEVICE_CUDA>::Add1D<int>(hashVisibleType > 0, visibleBlockCount);
		if (offset != -1) visibleBlockHashCodes[offset] = targetIdx;
	}
}
} //end anonymous namespace (Cuda global kernels)