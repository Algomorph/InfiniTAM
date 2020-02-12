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
			vbaIdx = atomicSub(&allocData->last_free_voxel_block_id, 1);

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
				atomicAdd(&allocData->last_free_voxel_block_id, 1);
			}
			break;

		case ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST: //needs allocation in the excess list
			vbaIdx = atomicSub(&allocData->last_free_voxel_block_id, 1);
			exlIdx = atomicSub(&allocData->last_free_excess_list_id, 1);

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
				atomicAdd(&allocData->last_free_voxel_block_id, 1);
				atomicAdd(&allocData->last_free_excess_list_id, 1);
			}

			break;
	}
}


__global__
void allocateHashedVoxelBlocksUsingLists_device(
		int* block_allocation_list, int* excess_allocation_list,
		AllocationTempData* temporary_allocation_data,
		HashEntry* hash_table, const int hash_entry_count,
		const ITMLib::HashEntryAllocationState* hash_entry_states, Vector3s* block_coordinates,
		int* utilized_block_hash_codes) {
	int hash_code = threadIdx.x + blockIdx.x * blockDim.x;
	if (hash_code >= hash_entry_count) return;

	int voxel_block_index, excess_list_index;

	auto updateUtilizedHashCodes = [&temporary_allocation_data, &utilized_block_hash_codes, &hash_code]() {
		int utilized_index = atomicAdd(&temporary_allocation_data->utilized_block_count, 1);
		utilized_block_hash_codes[utilized_index] = hash_code;
	};

	switch (hash_entry_states[hash_code]) {
		case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST: //needs allocation, fits in the ordered list
			voxel_block_index = atomicSub(&temporary_allocation_data->last_free_voxel_block_id, 1);
			if (voxel_block_index >= 0) //there is room in the voxel block array
			{
				HashEntry hash_entry;
				hash_entry.pos = block_coordinates[hash_code];
				hash_entry.ptr = block_allocation_list[voxel_block_index];
				hash_entry.offset = 0;
				hash_table[hash_code] = hash_entry;
				updateUtilizedHashCodes();
			} else {
				// Restore the previous value to avoid leaks.
				atomicAdd(&temporary_allocation_data->last_free_voxel_block_id, 1);
			}
			break;

		case ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST: //needs allocation in the excess list
			voxel_block_index = atomicSub(&temporary_allocation_data->last_free_voxel_block_id, 1);
			excess_list_index = atomicSub(&temporary_allocation_data->last_free_excess_list_id, 1);

			if (voxel_block_index >= 0 &&
			    excess_list_index >= 0) //there is room in the voxel block array and excess list
			{
				HashEntry hash_entry;
				hash_entry.pos = block_coordinates[hash_code];
				hash_entry.ptr = block_allocation_list[voxel_block_index];
				hash_entry.offset = 0;

				int excess_list_offset = excess_allocation_list[excess_list_index];

				hash_table[hash_code].offset = excess_list_offset + 1; //connect to child

				hash_table[ORDERED_LIST_SIZE + excess_list_offset] = hash_entry; //add child to the excess list
				updateUtilizedHashCodes();
			} else {
				// Restore the previous values to avoid leaks.
				atomicAdd(&temporary_allocation_data->last_free_voxel_block_id, 1);
				atomicAdd(&temporary_allocation_data->last_free_excess_list_id, 1);
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


__global__ void buildHashAllocAndVisibleType_device(ITMLib::HashEntryAllocationState* hash_entry_states,
                                                    Vector3s* block_coords, const float* depth,
                                                    Matrix4f inverted_camera_pose,
                                                    Vector4f inverted_camera_projection_parameters,
                                                    float surface_cutoff_distance, Vector2i depth_image_size,
                                                    float reciprocal_hash_block_size, HashEntry* hash_table,
                                                    float near_clipping_distance, float far_clipping_distance,
                                                    Vector3s* colliding_block_positions, int* colliding_block_count) {
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > depth_image_size.x - 1 || y > depth_image_size.y - 1) return;

	findVoxelBlocksForRayNearSurface(hash_entry_states, block_coords, hash_table, x, y, depth,
	                                 surface_cutoff_distance, inverted_camera_pose,
	                                 inverted_camera_projection_parameters, reciprocal_hash_block_size,
	                                 depth_image_size, near_clipping_distance, far_clipping_distance,
	                                 colliding_block_positions, colliding_block_count);
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
		vbaIdx = atomicSub(&allocData->last_free_voxel_block_id, 1);
		if (vbaIdx >= 0) hashTable[hashCode].ptr = voxelAllocationList[vbaIdx];
		else atomicAdd(&allocData->last_free_voxel_block_id, 1);
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

//ITMLib::HashEntryAllocationState* hash_entry_states,
//		Vector3s* hash_block_coordinates, int& hash_code,
//const CONSTPTR(Vector3s)& desired_hash_block_position,
//const CONSTPTR(HashEntry)* hash_table,
//		THREADPTR(Vector3s)* colliding_block_positions,
//THREADPTR(int*) colliding_block_hashes,
//ATOMIC_ARGUMENT(int) colliding_block_count

__global__ void buildHashAllocationTypeList_BlockList_device(
		const Vector3s* new_block_positions, HashEntryAllocationState* hash_entry_states, Vector3s* block_coordinates,
		HashEntry* hash_table, const int new_block_count, Vector3s* colliding_block_positions,
		int* colliding_block_count) {

	int new_block_index = threadIdx.x + blockIdx.x * blockDim.x;
	if (new_block_index >= new_block_count) return;

	Vector3s desired_block_position = new_block_positions[new_block_index];
	int hash_code = HashCodeFromBlockPosition(desired_block_position);

	MarkAsNeedingAllocationIfNotFound(hash_entry_states, block_coordinates, hash_code,
	                                  desired_block_position, hash_table, colliding_block_positions,
	                                  colliding_block_count);
}

} // end anonymous namespace (CUDA kernels)