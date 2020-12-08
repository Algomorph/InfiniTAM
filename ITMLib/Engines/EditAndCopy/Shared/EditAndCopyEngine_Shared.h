//  ================================================================
//  Created by Gregory Kramida on 9/25/19.
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
#pragma once


#include "../../../Utils/Math.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
//#ifdef __CUDACC__
//#include "../../../Utils/CUDA/CUDAUtils.h"
//#endif


struct CopyAllocationTempData {
	int countOfAllocatedOrderedEntries;
	int countOfAllocatedExcessEntries;
	int countOfBlocksToCopy;
	bool success;
};

template<typename TVoxel>
struct ReadVoxelResult {
	TVoxel voxel;
	int index;
	bool found;
};

struct CopyHashBlockPairInfo {
	int sourceHash;
	int destinationHash;
	bool fullyInBounds;
};






_CPU_AND_GPU_CODE_
inline
void GetVoxelHashLocals(int& vmIndex, int& locId, int& xInBlock, int& yInBlock, int& zInBlock,
                        const CONSTPTR(ITMLib::PlainVoxelArray::IndexData)* indexData,
                        ITMLib::PlainVoxelArray::IndexCache& cache, const CONSTPTR(Vector3i)& point) {
	locId = findVoxel(indexData, point, vmIndex);
	xInBlock = point.x;
	yInBlock = point.y;
	zInBlock = point.z;
}
/**
 * \brief Find the exact local positioning indices (1D and 3D) for a voxel with the given world coordinates
 * within a hash block
 * \tparam TVoxel type of voxel
 * \param vmIndex 0 if not found, 1 if in cache, positive integer representing hash + 1
 * \param locId
 * \param xInBlock
 * \param yInBlock
 * \param zInBlock
 * \param voxels
 * \param hashEntries
 * \param cache
 * \param point
 */
_CPU_AND_GPU_CODE_
inline
void GetVoxelHashLocals(int& vmIndex, int& locId, int& xInBlock, int& yInBlock, int& zInBlock,
                        const CONSTPTR(ITMLib::HashEntry)* hashEntries,
                        ITMLib::VoxelBlockHash::IndexCache& cache, const CONSTPTR(Vector3i)& point) {
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(point, blockPos);
	zInBlock = linearIdx / (VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE);
	yInBlock = (linearIdx % (VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE)) / VOXEL_BLOCK_SIZE;
	xInBlock = linearIdx - zInBlock * (VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE) - yInBlock * VOXEL_BLOCK_SIZE;
	locId = linearIdx;

	if IS_EQUAL3(blockPos, cache.blockPos) {
		vmIndex = true;
	}

	int hashIdx = HashCodeFromBlockPosition(blockPos);

	while (true) {
		ITMLib::HashEntry hashEntry = hashEntries[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0) {
			cache.blockPos = blockPos;
			cache.blockPtr = hashEntry.ptr * VOXEL_BLOCK_SIZE3;
			vmIndex = hashIdx + 1; // add 1 to support legacy true / false operations for isFound
		}

		if (hashEntry.offset < 1) break;
		hashIdx = ORDERED_LIST_SIZE + hashEntry.offset - 1;
	}

	vmIndex = false;
};