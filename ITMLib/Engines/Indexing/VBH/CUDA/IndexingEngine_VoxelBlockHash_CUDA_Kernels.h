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

#include "../../../../../ORUtils/PlatformIndependedParallelSum.h"
#include "../../../../Utils/CUDA/CUDAUtils.h"
#include "../../../../Objects/Volume/GlobalCache.h"
#include "../../../EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../Shared/IndexingEngine_Shared.h"
#include "../../Shared/IndexingEngine_RayMarching.h"
#include "../../../../Utils/Geometry/CheckBlockVisibility.h"

namespace {

//CUDA kernels

__global__ void
setVisibleEntriesToVisibleAtPreviousFrameAndUnstreamed(ITMLib::HashBlockVisibility* hash_block_visibility_types,
                                                       const int* utilized_block_hash_codes,
                                                       int utilized_entry_count) {
	int entryId = threadIdx.x + blockIdx.x * blockDim.x;
	if (entryId > utilized_entry_count - 1) return;
	hash_block_visibility_types[utilized_block_hash_codes[entryId]] = ITMLib::VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED;
}


template<bool useSwapping>
__global__ void
buildVisibilityList_device(ITMLib::HashEntry* hashTable, ITMLib::HashSwapState* swapStates, int hash_entry_count,
                           int* visibleEntryIDs, int* visible_block_count,
                           ITMLib::HashBlockVisibility* blockVisibilityTypes,
                           Matrix4f M_d, Vector4f projParams_d, Vector2i depthImgSize, float voxelSize) {
	int hash_code = threadIdx.x + blockIdx.x * blockDim.x;
	if (hash_code >= hash_entry_count) return;

	__shared__ bool shouldPrefix;
	shouldPrefix = false;
	__syncthreads();

	ITMLib::HashBlockVisibility hash_block_visibility_type = blockVisibilityTypes[hash_code];
	const ITMLib::HashEntry& hashEntry = hashTable[hash_code];

	if (hash_block_visibility_type == ITMLib::VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED) {
		bool isVisibleEnlarged, isVisible;

		if (useSwapping) {
			CheckVoxelHashBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d,
			                                    voxelSize,
			                                    depthImgSize);
			if (!isVisibleEnlarged) hash_block_visibility_type = ITMLib::INVISIBLE;
		} else {
			CheckVoxelHashBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d,
			                                     voxelSize,
			                                     depthImgSize);
			if (!isVisible) hash_block_visibility_type = ITMLib::INVISIBLE;
		}
		blockVisibilityTypes[hash_code] = hash_block_visibility_type;
	}

	if (hash_block_visibility_type > 0) shouldPrefix = true;

	if (useSwapping) {
		if (hash_block_visibility_type > 0 && swapStates[hash_code].state != 2) swapStates[hash_code].state = 1;
	}

	__syncthreads();

	if (shouldPrefix) {
		int offset = ORUtils::ParallelSum<MEMORYDEVICE_CUDA>::Add1D<int>(hash_block_visibility_type > 0, visible_block_count);
		if (offset != -1) visibleEntryIDs[offset] = hash_code;
	}

}

__global__ void
reAllocateSwappedOutVoxelBlocks_device(int* voxelAllocationList, ITMLib::HashEntry* hashTable, int hashEntryCount,
                                       AllocationTempData* allocData, /*int *countOfAllocatedOrderedEntries,*/
                                       ITMLib::HashBlockVisibility* blockVisibilityTypes) {
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

__global__ void
findHashEntry_device(ITMLib::HashEntry* entry, const ITMLib::HashEntry* hashTable, Vector3s pos, int* hashCode) {

	*hashCode = FindHashCodeAt(hashTable, pos);
	if (*hashCode != -1) {
		const ITMLib::HashEntry& targetEntry = hashTable[*hashCode];
		entry->offset = targetEntry.offset;
		entry->ptr = targetEntry.ptr;
		entry->pos = targetEntry.pos;
	}
}


struct SingleHashAllocationData {
	int last_free_allocation_block_id;
	int last_free_excess_entry_id;
	int utilized_hash_code_count;
	int hash_code;
	bool success;
};

__global__ void allocateHashEntry_device(SingleHashAllocationData* data,
                                         const Vector3s at, ITMLib::HashEntry* hash_table,
                                         const int* voxel_allocation_list,
                                         const int* excess_allocation_list,
                                         int* utilized_hash_codes) {
	ITMLib::HashEntry* entry;
	data->success = FindOrAllocateHashEntry(at, hash_table, entry, data->last_free_allocation_block_id,
	                                        data->last_free_excess_entry_id, data->utilized_hash_code_count,
	                                        voxel_allocation_list, excess_allocation_list,
	                                        utilized_hash_codes, data->hash_code);
};


__global__ void determineTargetAllocationForOffsetCopy_device(
		ITMLib::HashEntry* target_hash_table,
		const ITMLib::HashEntry* source_hash_table,
		ITMLib::HashEntryAllocationState* hash_entry_states,
		Vector3s* block_coordinates,
		const Vector6i target_bounds,
		const Vector6i inverse_offset_block_range,
		const Vector3i target_block_range,
		const Vector3i min_target_block_coordinate,
		Vector3s* colliding_block_positions,
		int* colliding_block_count,
		bool* unresolvable_collision_encountered) {

	int block_x = threadIdx.x + blockIdx.x * blockDim.x;
	int block_y = threadIdx.y + blockIdx.y * blockDim.y;
	int block_z = threadIdx.z + blockIdx.z * blockDim.z;
	if (block_x > target_block_range.x || block_y > target_block_range.y || block_z > target_block_range.z)
		return;

	block_x += min_target_block_coordinate.x;
	block_y += min_target_block_coordinate.y;
	block_z += min_target_block_coordinate.z;
	Vector3s target_block_position = Vector3s(block_x, block_y, block_z);

	if (!HashBlockAllocatedAtOffset(source_hash_table, target_block_position, inverse_offset_block_range)) {
		return;
	}

	ITMLib::ThreadAllocationStatus status =
			ITMLib::MarkAsNeedingAllocationIfNotFound<true>(hash_entry_states, block_coordinates,
			                                                target_block_position, target_hash_table,
			                                                colliding_block_positions, colliding_block_count);

	if (status == ITMLib::BEING_MODIFIED_BY_ANOTHER_THREAD) {
		*unresolvable_collision_encountered = true;
	}
}

} // end anonymous namespace (CUDA kernels)