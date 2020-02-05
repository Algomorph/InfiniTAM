//  ================================================================
//  Created by Gregory Kramida on 10/8/19.
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

#include "../../../../Utils/CUDAUtils.h"
#include "../../../../Objects/Volume/GlobalCache.h"
#include "../../../EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../Shared/IndexingEngine_Shared.h"
#include "../../../Common/CheckBlockVisibility.h"

namespace {
//CUDA kernels

//TODO: provide a better nomenclature for hash block visibility, i.e. an enum inheriting from unsigned char
__global__ void setVisibleEntriesToVisibleAtPreviousFrameAndUnstreamed(HashBlockVisibility* entriesVisibleType,
                                                                       const int* visibleBlockHashCodes,
                                                                       int visibleEntryCount) {
	int entryId = threadIdx.x + blockIdx.x * blockDim.x;
	if (entryId > visibleEntryCount - 1) return;
	entriesVisibleType[visibleBlockHashCodes[entryId]] = VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED;
}

__global__
void allocateHashedVoxelBlocksUsingLists_SetVisibility_device(
		int* voxelAllocationList, int* excessAllocationList,
		AllocationTempData* allocData,
		HashEntry* hashTable, int noTotalEntries,
		const ITMLib::HashEntryAllocationState* hashEntryStates, Vector3s* blockCoords,
		HashBlockVisibility* blockVisibilityTypes) {
	int hashCode = threadIdx.x + blockIdx.x * blockDim.x;
	if (hashCode >= noTotalEntries) return;
	int vbaIdx, exlIdx;

	switch (hashEntryStates[hashCode]) {
		case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST: //needs allocation, fits in the ordered list
			vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);

			if (vbaIdx >= 0) //there is room in the voxel block array
			{
				HashEntry hashEntry;
				hashEntry.pos = blockCoords[hashCode];
				hashEntry.ptr = voxelAllocationList[vbaIdx];
				hashEntry.offset = 0;
				hashTable[hashCode] = hashEntry;
			} else {
				blockVisibilityTypes[hashCode] = INVISIBLE;
				// Restore the previous value to avoid leaks.
				atomicAdd(&allocData->noAllocatedVoxelEntries, 1);
			}
			break;

		case ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST: //needs allocation in the excess list
			vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
			exlIdx = atomicSub(&allocData->noAllocatedExcessEntries, 1);

			if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
			{
				HashEntry hashEntry;
				hashEntry.pos = blockCoords[hashCode];
				hashEntry.ptr = voxelAllocationList[vbaIdx];
				hashEntry.offset = 0;

				int exlOffset = excessAllocationList[exlIdx];

				hashTable[hashCode].offset = exlOffset + 1; //connect to child

				hashTable[ORDERED_LIST_SIZE + exlOffset] = hashEntry; //add child to the excess list
				blockVisibilityTypes[ORDERED_LIST_SIZE + exlOffset] = IN_MEMORY_AND_VISIBLE;
			} else {
				// Restore the previous values to avoid leaks.
				atomicAdd(&allocData->noAllocatedVoxelEntries, 1);
				atomicAdd(&allocData->noAllocatedExcessEntries, 1);
			}

			break;
	}
}


__global__
void allocateHashedVoxelBlocksUsingLists_device(
		int* voxelAllocationList, int* excessAllocationList,
		AllocationTempData* allocData,
		HashEntry* hashTable, const int hashEntryCount,
		const ITMLib::HashEntryAllocationState* hashEntryStates, Vector3s* blockCoords) {
	int hashCode = threadIdx.x + blockIdx.x * blockDim.x;
	if (hashCode >= hashEntryCount) return;

	int voxelBlockIndex, exlIdx;

	switch (hashEntryStates[hashCode]) {
		case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST: //needs allocation, fits in the ordered list
			voxelBlockIndex = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
			if (voxelBlockIndex >= 0) //there is room in the voxel block array
			{
				HashEntry hashEntry;
				hashEntry.pos = blockCoords[hashCode];
				hashEntry.ptr = voxelAllocationList[voxelBlockIndex];
				hashEntry.offset = 0;
				hashTable[hashCode] = hashEntry;
			} else {
				// Restore the previous value to avoid leaks.
				atomicAdd(&allocData->noAllocatedVoxelEntries, 1);
			}
			break;

		case ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST: //needs allocation in the excess list
			voxelBlockIndex = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
			exlIdx = atomicSub(&allocData->noAllocatedExcessEntries, 1);

			if (voxelBlockIndex >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
			{
				HashEntry hashEntry;
				hashEntry.pos = blockCoords[hashCode];
				hashEntry.ptr = voxelAllocationList[voxelBlockIndex];
				hashEntry.offset = 0;

				int exlOffset = excessAllocationList[exlIdx];

				hashTable[hashCode].offset = exlOffset + 1; //connect to child

				hashTable[ORDERED_LIST_SIZE + exlOffset] = hashEntry; //add child to the excess list
			} else {
				// Restore the previous values to avoid leaks.
				atomicAdd(&allocData->noAllocatedVoxelEntries, 1);
				atomicAdd(&allocData->noAllocatedExcessEntries, 1);
			}

			break;
	}
}

__global__
void buildHashAllocationTypeList_VolumeToVolume(ITMLib::HashEntryAllocationState* hashEntryStates,
                                                Vector3s* blockCoordinates,
                                                HashEntry* targetHashTable, const HashEntry* sourceHashTable,
                                                int hashEntryCount,
                                                bool* collisionDetected) {
	int hashCode = threadIdx.x + blockIdx.x * blockDim.x;
	if (hashCode >= hashEntryCount) return;
	const HashEntry& sourceHashEntry = sourceHashTable[hashCode];
	if (sourceHashEntry.ptr < 0) return;
	Vector3s sourceBlockCoordinates = sourceHashEntry.pos;
	int targetHash = HashCodeFromBlockPosition(sourceBlockCoordinates);

	MarkAsNeedingAllocationIfNotFound(hashEntryStates, blockCoordinates,
	                                  targetHash, sourceBlockCoordinates, targetHashTable,
	                                  *collisionDetected);
}


__global__ void buildHashAllocAndVisibleType_device(ITMLib::HashEntryAllocationState* hashEntryStates,
                                                    HashBlockVisibility* blockVisibilityTypes,
                                                    Vector3s* blockCoords, const float* depth,
                                                    Matrix4f invertedCameraTransform, Vector4f projParams_d,
                                                    float surface_cutoff_distance,
                                                    Vector2i _imgSize, float oneOverHashBlockSizeMeters,
                                                    HashEntry* hashTable, float near_clipping_distance,
                                                    float far_clipping_distance, bool* collisionDetected) {
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > _imgSize.x - 1 || y > _imgSize.y - 1) return;

	findVoxelBlocksForRayNearSurface(hashEntryStates, blockCoords, blockVisibilityTypes,
	                                 hashTable, x, y, depth,
	                                 surface_cutoff_distance,
	                                 invertedCameraTransform, projParams_d, oneOverHashBlockSizeMeters, _imgSize,
	                                 near_clipping_distance, far_clipping_distance,
	                                 *collisionDetected);
}


template<bool useSwapping>
__global__ void
buildVisibilityList_device(HashEntry* hashTable, ITMLib::ITMHashSwapState* swapStates, int hashEntryCount,
                           int* visibleEntryIDs, int* visibleBlockCount, HashBlockVisibility* blockVisibilityTypes,
                           Matrix4f M_d, Vector4f projParams_d, Vector2i depthImgSize, float voxelSize) {
	int hashCode = threadIdx.x + blockIdx.x * blockDim.x;
	if (hashCode >= hashEntryCount) return;

	__shared__ bool shouldPrefix;
	shouldPrefix = false;
	__syncthreads();

	HashBlockVisibility hashVisibleType = blockVisibilityTypes[hashCode];
	const HashEntry& hashEntry = hashTable[hashCode];

	if (hashVisibleType == VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED) {
		bool isVisibleEnlarged, isVisible;

		if (useSwapping) {
			checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize,
			                           depthImgSize);
			if (!isVisibleEnlarged) hashVisibleType = INVISIBLE;
		} else {
			checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize,
			                            depthImgSize);
			if (!isVisible) hashVisibleType = INVISIBLE;
		}
		blockVisibilityTypes[hashCode] = hashVisibleType;
	}

	if (hashVisibleType > 0) shouldPrefix = true;

	if (useSwapping) {
		if (hashVisibleType > 0 && swapStates[hashCode].state != 2) swapStates[hashCode].state = 1;
	}

	__syncthreads();

	if (shouldPrefix) {
		int offset = computePrefixSum_device<int>(hashVisibleType > 0, visibleBlockCount,
		                                          blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) visibleEntryIDs[offset] = hashCode;
	}

}

__global__ void
reAllocateSwappedOutVoxelBlocks_device(int* voxelAllocationList, HashEntry* hashTable, int hashEntryCount,
                                       AllocationTempData* allocData, /*int *countOfAllocatedOrderedEntries,*/
                                       HashBlockVisibility* blockVisibilityTypes) {
	int hashCode = threadIdx.x + blockIdx.x * blockDim.x;
	if (hashCode >= hashEntryCount) return;

	int vbaIdx;
	int hashEntry_ptr = hashTable[hashCode].ptr;

	if (blockVisibilityTypes[hashCode] > 0 &&
	    hashEntry_ptr == -1) //it is visible and has been previously allocated inside the hash, but deallocated from VBA
	{
		vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
		if (vbaIdx >= 0) hashTable[hashCode].ptr = voxelAllocationList[vbaIdx];
		else atomicAdd(&allocData->noAllocatedVoxelEntries, 1);
	}
}

__global__ void findHashEntry_device(HashEntry* entry, const HashEntry* hashTable, Vector3s pos, int* hashCode) {

	*hashCode = FindHashCodeAt(hashTable, pos);
	if (*hashCode != -1) {
		const HashEntry& targetEntry = hashTable[*hashCode];
		entry->offset = targetEntry.offset;
		entry->ptr = targetEntry.ptr;
		entry->pos = targetEntry.pos;
	}
}


__global__ void markForAllocationBasedOnOneRingAroundAnotherAllocation_device(
		ITMLib::HashEntryAllocationState* hashEntryStates, Vector3s* blockCoordinates,
		const HashEntry* sourceHashTable, const HashEntry* targetHashTable, int hashEntryCount,
		bool* collisionDetected) {

	if (*collisionDetected) return;
	int hashCode = blockIdx.x;

	if (hashCode >= hashEntryCount) return;
	const HashEntry& sourceHashEntry = sourceHashTable[hashCode];
	if (sourceHashEntry.ptr < 0) return;

	Vector3s targetBlockCoordinates = sourceHashEntry.pos + Vector3s(threadIdx.x - 1, threadIdx.y - 1, threadIdx.z - 1);

	int targetHash = HashCodeFromBlockPosition(targetBlockCoordinates);
	MarkAsNeedingAllocationIfNotFound(hashEntryStates, blockCoordinates, targetHash, targetBlockCoordinates,
	                                  targetHashTable, *collisionDetected);
}

__global__ void markForAllocationAndSetVisibilityBasedOnOneRingAroundAnotherAllocation_device(
		ITMLib::HashEntryAllocationState* hashEntryStates, Vector3s* blockCoordinates,
		HashBlockVisibility* hashBlockVisibilityTypes,
		const HashEntry* sourceHashTable, const HashEntry* targetHashTable, int hashEntryCount,
		bool* collisionDetected) {

	if (*collisionDetected) return;
	int hashCode = blockIdx.x;

	if (hashCode >= hashEntryCount) return;
	const HashEntry& sourceHashEntry = sourceHashTable[hashCode];
	if (sourceHashEntry.ptr < 0) return;

	Vector3s targetBlockCoordinates = sourceHashEntry.pos + Vector3s(threadIdx.x - 1, threadIdx.y - 1, threadIdx.z - 1);
	MarkForAllocationAndSetVisibilityTypeIfNotFound(hashEntryStates, blockCoordinates, hashBlockVisibilityTypes,
	                                                targetBlockCoordinates, targetHashTable, *collisionDetected);
}

struct SingleHashAllocationData {
	int lastFreeVoxelBlockId;
	int lastFreeExcessListId;
	int hashCode;
	bool success;
};

__global__ void allocateHashEntry_device(SingleHashAllocationData* data,
                                         const Vector3s at, HashEntry* hashTable,
                                         const int* voxelAllocationList, const int* excessAllocationList) {
	HashEntry* entry;
	data->success = FindOrAllocateHashEntry(at, hashTable, entry, data->lastFreeVoxelBlockId,
	                                        data->lastFreeExcessListId, voxelAllocationList, excessAllocationList,
	                                        data->hashCode);
};

} // end anonymous namespace (CUDA kernels)