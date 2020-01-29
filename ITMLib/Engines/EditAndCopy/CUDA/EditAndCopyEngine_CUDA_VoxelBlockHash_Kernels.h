//  ================================================================
//  Created by Gregory Kramida on 10/2/19.
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

#include "../../../../ORUtils/JetbrainsCUDASyntax.hpp"
#include "../../Indexing/Shared/IndexingEngine_Shared.h"
#include "../Shared/EditAndCopyEngine_Shared.h"
#include "../../../Utils/ITMHashBlockProperties.h"
#include "../../../Utils/Geometry/ITMGeometryBooleanOperations.h"

using namespace ITMLib;

namespace {

template<class TVoxel>
__global__ void noOffsetCopy_device(TVoxel* destinationVoxels, const TVoxel* sourceVoxels,
                                    const ITMHashEntry* destinationHashTable,
                                    const ITMHashEntry* sourceHashTable,
                                    int totalBlocksToCopy,
                                    const CopyHashBlockPairInfo* copyHashIdBuffer,
                                    const Vector6i bounds) {

	if (blockIdx.x > totalBlocksToCopy - 1) return;
	const CopyHashBlockPairInfo& hashBlockPairInfo = copyHashIdBuffer[blockIdx.x];
	int sourceHash = hashBlockPairInfo.sourceHash;
	int destinationHash = hashBlockPairInfo.destinationHash;

	const ITMHashEntry& destinationHashEntry = destinationHashTable[destinationHash];
	if (destinationHashEntry.ptr < 0) return;
	const ITMHashEntry& sourceHashEntry = sourceHashTable[sourceHash];

	TVoxel* destinationVoxelBlock = &(destinationVoxels[destinationHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
	const TVoxel* sourceVoxelBlock = &(sourceVoxels[sourceHashEntry.ptr * VOXEL_BLOCK_SIZE3]);

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	Vector3i globalPos = destinationHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
	globalPos.x += x;
	globalPos.y += y;
	globalPos.z += z;

	if (hashBlockPairInfo.fullyInBounds || isPointInBounds(globalPos, bounds)) {
		destinationVoxelBlock[locId] = sourceVoxelBlock[locId];
	}
}

template<class TVoxel>
__global__ void offsetCopy_device(TVoxel* destinationVoxels, const TVoxel* sourceVoxels,
                                  const ITMHashEntry* destinationHashTable,
                                  const ITMHashEntry* sourceHashTable,
                                  int totalBlocksToCopy,
                                  const OffsetCopyHashBlockInfo* copyHashIdBuffer,
                                  const Vector3i offset,
                                  const Vector6i bounds) {
	// assume grid is one-dimensional, blockIdx corresponds to the destination block to copy
	if (blockIdx.x > totalBlocksToCopy - 1) return;
	const OffsetCopyHashBlockInfo& copyHashBlockInfo = copyHashIdBuffer[blockIdx.x];
	const ITMHashEntry& destinationHashEntry = destinationHashTable[copyHashBlockInfo.destinationHash];
	if (destinationHashEntry.ptr < 0) return;

	TVoxel* destinationVoxelBlock = &(destinationVoxels[destinationHashEntry.ptr * VOXEL_BLOCK_SIZE3]);

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	Vector3i sourcePoint = destinationHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE - offset;
	sourcePoint.x += x;
	sourcePoint.y += y;
	sourcePoint.z += z;

	int vmIndex;
	int sourceVoxelIx = findVoxel(sourceHashTable, sourcePoint, vmIndex);
	if (!vmIndex) return;

	if (copyHashBlockInfo.fullyInBounds || isPointInBounds(sourcePoint, bounds)) {
		destinationVoxelBlock[locId] = sourceVoxels[sourceVoxelIx];
	}
}

template<typename TVoxel>
__global__ void setVoxel_device(TVoxel* voxelArray, ITMHashEntry* hashTable,
                                const Vector3i at, TVoxel value, CopyAllocationTempData* setVoxelTempData,
                                const int* voxelAllocationList, const int* excessAllocationList, int voxelIndexInBlock,
                                const Vector3s blockPos) {

	ITMHashEntry* entry = nullptr;
	int hash;
	if (FindOrAllocateHashEntry(blockPos, hashTable, entry,
	                            setVoxelTempData->countOfAllocatedOrderedEntries,
	                            setVoxelTempData->countOfAllocatedExcessEntries, voxelAllocationList, excessAllocationList,
	                            hash)) {
		TVoxel* localVoxelBlock = &(voxelArray[entry->ptr * (VOXEL_BLOCK_SIZE3)]);
		localVoxelBlock[voxelIndexInBlock] = value;
		setVoxelTempData->success = true;
	} else {
		setVoxelTempData->success = false;
	}
}

template<class TVoxel>
__global__ void readVoxel_device(TVoxel* voxelArray, const ITMHashEntry* hashTable,
                                 const Vector3i at, ReadVoxelResult<TVoxel>* result) {
	int vmIndex = 0;
	int arrayIndex = findVoxel(hashTable, at, vmIndex);
	if (arrayIndex < 0) {
		result->found = false;
	} else {
		result->found = true;
		result->voxel = voxelArray[arrayIndex];
	}
}

template<class TVoxel>
__global__ void readVoxel_device(TVoxel* voxelArray, const ITMHashEntry* hashTable,
                                 Vector3i at, ReadVoxelResult<TVoxel>* result,
                                 ITMLib::VoxelBlockHash::IndexCache* cache) {
	int vmIndex = 0;
	int arrayIndex = findVoxel(hashTable, at, vmIndex);
	if (arrayIndex < 0) {
		result->found = false;
		result->index = -1;
	} else {
		result->found = true;
		result->index = arrayIndex;
		result->voxel = voxelArray[arrayIndex];
	}
}


// Modifies copyHashIdBuffer, entriesAllocType, allocationBlockCoords, and allocData
// Finds which blocks in the destination volume need to be copied, and also which need to be
// allocated before copying can be done. Sets the corresponding entries in entriesAllocType and allocationBlockCoords
// to the bocks that need to be allocated.
// The copyHashIdBuffer is filled with <source,destination> hash pairs for the copying, accompanied by a flag that
// indicates whether the hash block is fully within the bounds specified for copying (alternative being -- partially in bounds).
__global__ void determineDestinationAllocationForNoOffsetCopy_device(
		ITMHashEntry* destinationHashTable,
		const ITMHashEntry* sourceHashTable,
		int noTotalEntries,
		ITMLib::HashEntryAllocationState* entriesAllocType,
		Vector3s* allocationBlockCoords,
		CopyAllocationTempData* allocData,
		CopyHashBlockPairInfo* copyHashIdBuffer,
		const Vector6i bounds) {
	int sourceHash = threadIdx.x + blockIdx.x * blockDim.x;
	if (sourceHash > noTotalEntries - 1) return;

	const ITMHashEntry& currentSourceHashEntry = sourceHashTable[sourceHash];
	if (currentSourceHashEntry.ptr < 0) return;
	Vector3i originalHashBlockPosition = currentSourceHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
	bool isFullyInRange = IsHashBlockFullyInRange(originalHashBlockPosition, bounds);
	bool isPartiallyInRange = false;
	if (!isFullyInRange) {
		isPartiallyInRange = IsHashBlockPartiallyInRange(originalHashBlockPosition, bounds);
		if (!isPartiallyInRange) return;
	}
	int destinationHash = HashCodeFromBlockPosition(currentSourceHashEntry.pos);
	bool collisionDetected = false;
	// see if the block in the destination hash structure needs allocation. If so, mark it as such in entriesAllocType
	// and record its spatial coordinates (in blocks) in allocationBlockCoords.
	MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, destinationHash,
	                                  currentSourceHashEntry.pos, destinationHashTable, collisionDetected);
	//report operation success, as we know some voxels will be copied here
	allocData->success = true;
	//mark which hash to copy from and which to copy to
	int bufferIdx = atomicAdd(&allocData->countOfBlocksToCopy, 1);
	copyHashIdBuffer[bufferIdx].sourceHash = sourceHash;
	copyHashIdBuffer[bufferIdx].destinationHash = destinationHash;
	copyHashIdBuffer[bufferIdx].fullyInBounds = isFullyInRange;

}

// same as above without bounds checks -- intended for whole-scene copying
__global__ void determineDestinationAllocationForNoOffsetCopy_device(
		ITMHashEntry* destinationHashTable,
		const ITMHashEntry* sourceHashTable,
		const int noTotalEntries,
		ITMLib::HashEntryAllocationState* entriesAllocType,
		Vector3s* allocationBlockCoords,
		CopyAllocationTempData* allocData,
		CopyHashBlockPairInfo* copyHashIdBuffer) {
	int sourceHash = threadIdx.x + blockIdx.x * blockDim.x;
	if (sourceHash > noTotalEntries - 1) return;

	const ITMHashEntry& currentSourceHashEntry = sourceHashTable[sourceHash];
	if (currentSourceHashEntry.ptr < 0) return;
	int destinationHash = HashCodeFromBlockPosition(currentSourceHashEntry.pos);
	bool collisionDetected = false;
	// see if the block in the destination hash structure needs allocation. If so, mark it as such in entriesAllocType
	// and record its spatial coordinates (in blocks) in allocationBlockCoords.
	MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, destinationHash,
	                                  currentSourceHashEntry.pos, destinationHashTable, collisionDetected);
	// report operation success, as we know some voxels will be copied here
	allocData->success = true;
	// mark which hash to copy from and which to copy to
	int bufferIdx = atomicAdd(&allocData->countOfBlocksToCopy, 1);
	copyHashIdBuffer[bufferIdx].sourceHash = sourceHash;
	copyHashIdBuffer[bufferIdx].destinationHash = destinationHash;
	copyHashIdBuffer[bufferIdx].fullyInBounds = true;

}

__global__ void determineDestinationAllocationForOffsetCopy_device(
		ITMHashEntry* destinationHashTable,
		const ITMHashEntry* sourceHashTable,
		ITMLib::HashEntryAllocationState* entriesAllocType,
		Vector3s* allocationBlockCoords,
		CopyAllocationTempData* allocData,
		OffsetCopyHashBlockInfo* copyHashIdBuffer,
		const Vector6i destinationBounds,
		const Vector6i inverseOffsetBlockRange,
		const Vector3i destinationBlockRange,
		const Vector3i minDestinationBlockCoord) {

	int blockX = threadIdx.x + blockIdx.x * blockDim.x;
	int blockY = threadIdx.y + blockIdx.y * blockDim.y;
	int blockZ = threadIdx.z + blockIdx.z * blockDim.z;
	if (blockX > destinationBlockRange.x || blockY > destinationBlockRange.y || blockZ > destinationBlockRange.z)
		return;
	blockX += minDestinationBlockCoord.x;
	blockY += minDestinationBlockCoord.y;
	blockZ += minDestinationBlockCoord.z;
	Vector3s destinationBlockPos = Vector3s(blockX, blockY, blockZ);

	if (!HashBlockAllocatedAtOffset(sourceHashTable, destinationBlockPos, inverseOffsetBlockRange)) {
		return;
	}

	int destinationHash = HashCodeFromBlockPosition(destinationBlockPos);
	bool collisionDetected = false;
	bool needs_alloc = MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, destinationHash,
	                                  destinationBlockPos, destinationHashTable, collisionDetected);

	//report operation success, as we know some voxels will be copied here
	allocData->success = true;
	//mark which hash to copy from and which to copy to
	int bufferIdx = atomicAdd(&allocData->countOfBlocksToCopy, 1);
	copyHashIdBuffer[bufferIdx].destinationHash = destinationHash;
	copyHashIdBuffer[bufferIdx].fullyInBounds =
			IsHashBlockFullyInRange(destinationBlockPos.toInt() * VOXEL_BLOCK_SIZE, destinationBounds);

}

} // end anonymous namespace (CUDA kernels)
