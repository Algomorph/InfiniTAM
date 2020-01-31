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


#include "../../../Utils/ITMMath.h"
#include "../../../Objects/Scene/VoxelBlockHash.h"
#include "../../../Objects/Scene/RepresentationAccess.h"
//#ifdef __CUDACC__
//#include "../../../Utils/ITMCUDAUtils.h"
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

struct OffsetCopyHashBlockInfo {
	int destinationHash;
	bool fullyInBounds;
};



//TODO: move to GeometryBooleanOperations in Utils
_CPU_AND_GPU_CODE_
inline
bool
IsHashBlockFullyInRange(const Vector3i& hashBlockPositionVoxels, const Vector6i& bounds) {
	return hashBlockPositionVoxels.x + VOXEL_BLOCK_SIZE - 1 <= bounds.max_x &&
	       hashBlockPositionVoxels.x >= bounds.min_x &&
	       hashBlockPositionVoxels.y + VOXEL_BLOCK_SIZE - 1 <= bounds.max_y &&
	       hashBlockPositionVoxels.y >= bounds.min_y &&
	       hashBlockPositionVoxels.z + VOXEL_BLOCK_SIZE - 1 <= bounds.max_z &&
	       hashBlockPositionVoxels.z >= bounds.min_z;
}

//TODO: move to GeometryBooleanOperations in Utils
_CPU_AND_GPU_CODE_
inline
bool IsHashBlockPartiallyInRange(const Vector3i& hashBlockPositionVoxels, const Vector6i& bounds) {
	//@formatter:off
	return ((hashBlockPositionVoxels.x + VOXEL_BLOCK_SIZE - 1 >= bounds.max_x && hashBlockPositionVoxels.x <= bounds.max_x)
	     || (hashBlockPositionVoxels.x + VOXEL_BLOCK_SIZE - 1 >= bounds.min_x && hashBlockPositionVoxels.x <= bounds.min_x)) &&
	       ((hashBlockPositionVoxels.y + VOXEL_BLOCK_SIZE - 1 >= bounds.max_y && hashBlockPositionVoxels.y <= bounds.max_y)
	     || (hashBlockPositionVoxels.y + VOXEL_BLOCK_SIZE - 1 >= bounds.min_y && hashBlockPositionVoxels.y <= bounds.min_y)) &&
	       ((hashBlockPositionVoxels.z + VOXEL_BLOCK_SIZE - 1 >= bounds.max_z && hashBlockPositionVoxels.z <= bounds.max_z)
	     || (hashBlockPositionVoxels.z + VOXEL_BLOCK_SIZE - 1 >= bounds.min_z && hashBlockPositionVoxels.z <= bounds.min_z));
	//@formatter:on
}

/**
 * \brief find the hash block at the specified spatial coordinates (in blocks, not voxels!) and return its hash
 * \param voxelIndex
 * \param at
 * \return -1 if hash block is not found, hash code of the block otherwise
 */
_CPU_AND_GPU_CODE_
inline int
FindHashCodeAt(const CONSTPTR(ITMLib::VoxelBlockHash::IndexData)* voxelIndex, const THREADPTR(Vector3s)& at) {
	int hash = HashCodeFromBlockPosition(at);
	while (true) {
		ITMHashEntry hashEntry = voxelIndex[hash];

		if (IS_EQUAL3(hashEntry.pos, at) && hashEntry.ptr >= 0) {
			return hash;
		}

		if (hashEntry.offset < 1) break;
		hash = ORDERED_LIST_SIZE + hashEntry.offset - 1;
	}
	return -1;
}

_CPU_AND_GPU_CODE_
inline void
ComputeVoxelBlockOffsetRange(const CONSTPTR(Vector3i)& offset,
                             THREADPTR(Vector6i)& offsetRange) {
#define  ITM_CMP(a, b)    (((a) > (b)) - ((a) < (b)))
#define  ITM_SIGN(a)     ITM_CMP((a),0)

	int xA = offset.x / VOXEL_BLOCK_SIZE;
	int xB = xA + ITM_SIGN(offset.x % VOXEL_BLOCK_SIZE);
	int yA = offset.y / VOXEL_BLOCK_SIZE;
	int yB = yA + ITM_SIGN(offset.y % VOXEL_BLOCK_SIZE);
	int zA = offset.z / VOXEL_BLOCK_SIZE;
	int zB = zA + ITM_SIGN(offset.z % VOXEL_BLOCK_SIZE);
	int tmp;
	if (xA > xB) {
		tmp = xA;
		xA = xB;
		xB = tmp;
	}
	if (yA > yB) {
		tmp = yA;
		yA = yB;
		yB = tmp;
	}
	if (zA > zB) {
		tmp = zA;
		zA = zB;
		zB = tmp;
	}
	offsetRange.min_x = xA;
	offsetRange.min_y = yA;
	offsetRange.min_z = zA;
	offsetRange.max_x = xB + 1;
	offsetRange.max_y = yB + 1;
	offsetRange.max_z = zB + 1;
#undef ITM_CMP
#undef ITM_SIGN
}

/**
 * \brief Determine if hash blocks spanning any part of the volume offset from the given block by a given vector are allocated
 * \param targetIndex Target index where to check the blocks being allocated
 * \param blockCoord coordinate of the given block (before offset)
 * \param offset offset, in voxels (not blocks!)
 * \return true if any of the said blocks are allocated, false otherwise
 */
_CPU_AND_GPU_CODE_
inline bool
HashBlockAllocatedAtOffset(const CONSTPTR(ITMLib::VoxelBlockHash::IndexData)* targetIndex,
                           const THREADPTR(Vector3s)& blockCoord,
                           const CONSTPTR(Vector3i)& offset) {
	Vector6i blockRange;
	ComputeVoxelBlockOffsetRange(offset, blockRange);

	bool allocated = false;
	for (int z = blockCoord.z + blockRange.min_z; z < blockCoord.z + blockRange.max_z; z++) {
		for (int y = blockCoord.y + blockRange.min_y; y < blockCoord.y + blockRange.max_y; y++) {
			for (int x = blockCoord.x + blockRange.min_x; x < blockCoord.x + blockRange.max_x; x++) {
				if (FindHashCodeAt(targetIndex, Vector3s(x, y, z)) != -1) {
					allocated = true;
				}
			}
		}
	}
	return allocated;
}

/**
 * \brief Same as above, except with block ranges spanning the offset volume of the given block precomputed and provided
 * \param targetIndex Target index where to check the blocks being allocated
 * \param blockCoord coordinate of the given block (before offset)
 * \param offsetBlockRange block range to check (in blocks!) - each value of the range is an offset from block coord
 * \return  true if any of the blocks in blockCoord + [values within offset range] are allocated, false otherwise
 */
_CPU_AND_GPU_CODE_
inline bool
HashBlockAllocatedAtOffset(const CONSTPTR(ITMLib::VoxelBlockHash::IndexData)* targetIndex,
                           const THREADPTR(Vector3s)& blockCoord,
                           const CONSTPTR(Vector6i)& offsetBlockRange) {

	bool allocated = false;
	for (int z = blockCoord.z + offsetBlockRange.min_z; z < blockCoord.z + offsetBlockRange.max_z; z++) {
		for (int y = blockCoord.y + offsetBlockRange.min_y; y < blockCoord.y + offsetBlockRange.max_y; y++) {
			for (int x = blockCoord.x + offsetBlockRange.min_x; x < blockCoord.x + offsetBlockRange.max_x; x++) {
				if (FindHashCodeAt(targetIndex, Vector3s(x, y, z)) != -1) {
					allocated = true;
				}
			}
		}
	}
	return allocated;
}


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
                        const CONSTPTR(ITMLib::VoxelBlockHash::IndexData)* hashEntries,
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
		ITMHashEntry hashEntry = hashEntries[hashIdx];

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