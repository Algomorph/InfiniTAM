//  ================================================================
//  Created by Gregory Kramida on 11/1/19.
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

#include "IndexingEngine_CPU_VoxelBlockHash.h"
#include "../../../../Objects/Volume/RepresentationAccess.h"
#include "../../../EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../Shared/IndexingEngine_Functors.h"
#include "../../../Common/CheckBlockVisibility.h"
#include "../../../../Utils/Configuration.h"
#include "../../../../Utils/Geometry/FrustumTrigonometry.h"
#include "../../AtomicArrayThreadGuard/AtomicArrayThreadGuard_CPU.h"


using namespace ITMLib;


template<typename TVoxel>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume) {

	assert(targetVolume->index.hashEntryCount == sourceVolume->index.hashEntryCount);

	const int hashEntryCount = targetVolume->index.hashEntryCount;

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();
	HashEntry* targetHashEntries = targetVolume->index.GetEntries();
	HashEntry* sourceHashEntries = sourceVolume->index.GetEntries();

	bool collisionDetected;

	do {
		collisionDetected = false;
		//reset target allocation states
		targetVolume->index.ClearHashEntryAllocationStates();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(sourceHashEntries, hashEntryStates_device, blockCoordinates_device, \
        targetHashEntries, collisionDetected)
#endif
		for (int sourceHash = 0; sourceHash < hashEntryCount; sourceHash++) {

			const HashEntry& currentSourceHashBlock = sourceHashEntries[sourceHash];
			//skip unfilled live blocks
			if (currentSourceHashBlock.ptr < 0) {
				continue;
			}
			Vector3s sourceHashBlockCoords = currentSourceHashBlock.pos;

			//try to find a corresponding canonical block, and mark it for allocation if not found
			int targetHash = HashCodeFromBlockPosition(sourceHashBlockCoords);

			MarkAsNeedingAllocationIfNotFound(hashEntryStates_device, blockCoordinates_device,
			                                  targetHash, sourceHashBlockCoords, targetHashEntries,
			                                  collisionDetected);
		}

		IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists(targetVolume);
	} while (collisionDetected);

}

static std::vector<Vector3s> neighborOffsets = [] { // NOLINT(cert-err58-cpp)
	std::vector<Vector3s> offsets;
	for (short zOffset = -1; zOffset < 2; zOffset++) {
		for (short yOffset = -1; yOffset < 2; yOffset++) {
			for (short xOffset = -1; xOffset < 2; xOffset++) {
				Vector3s neighborOffset(xOffset, yOffset, zOffset);
				offsets.push_back(neighborOffset);
			}
		}
	}
	return offsets;
}();

template<typename TVoxelTarget, typename TVoxelSource, typename THashBlockMarkProcedure, typename TAllocationProcedure>
void AllocateUsingOtherVolumeExpanded_Generic(VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
                                              VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume,
                                              THashBlockMarkProcedure&& hashBlockMarkProcedure,
                                              TAllocationProcedure&& allocationProcedure) {
	assert(sourceVolume->index.hashEntryCount == targetVolume->index.hashEntryCount);

	int hashEntryCount = targetVolume->index.hashEntryCount;
	HashEntry* targetHashTable = targetVolume->index.GetEntries();
	HashEntry* sourceHashTable = sourceVolume->index.GetEntries();

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();

	bool collision_detected;
	do {
		collision_detected = false;
		targetVolume->index.ClearHashEntryAllocationStates();
#ifdef WITH_OPENMP
#pragma omp parallel for default(shared)
#endif
		for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
			const HashEntry& hashEntry = sourceHashTable[hashCode];
			// skip empty blocks
			if (hashEntry.ptr < 0) continue;
			for (auto& neighborOffset : neighborOffsets) {
				Vector3s neighborBlockCoordinates = hashEntry.pos + neighborOffset;
				// try to find a corresponding canonical block, and mark it for allocation if not found
				int targetHash = HashCodeFromBlockPosition(neighborBlockCoordinates);
				std::forward<THashBlockMarkProcedure>(hashBlockMarkProcedure)(neighborBlockCoordinates, targetHash,
				                                                              collision_detected);
			}
		}
		std::forward<TAllocationProcedure>(allocationProcedure)();
	} while (collision_detected);
}

template<typename TVoxel>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume) {

	HashEntry* targetHashTable = targetVolume->index.GetEntries();

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();

	auto hashBlockMarkProcedure = [&](Vector3s neighborBlockCoordinates, int& targetHash, bool& collision_detected) {
		MarkAsNeedingAllocationIfNotFound(hashEntryStates_device, blockCoordinates_device,
		                                  targetHash, neighborBlockCoordinates, targetHashTable,
		                                  collision_detected);
	};
	auto allocationProcedure = [&]() {
		IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists(targetVolume);
	};
	AllocateUsingOtherVolumeExpanded_Generic(targetVolume, sourceVolume, hashBlockMarkProcedure, allocationProcedure);
}


template<typename TVoxel>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix) {

	HashEntry* targetHashTable = targetVolume->index.GetEntries();
	HashBlockVisibility* hashBlockVisibilityTypes_device = targetVolume->index.GetBlockVisibilityTypes();

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();

	IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(
			targetVolume);

	auto hashBlockMarkProcedure = [&](Vector3s neighborBlockCoordinates, int& targetHash, bool& collision_detected) {
		MarkForAllocationAndSetVisibilityTypeIfNotFound(
				hashEntryStates_device, blockCoordinates_device, hashBlockVisibilityTypes_device,
				neighborBlockCoordinates, targetHashTable, collision_detected);
	};
	auto allocationProcedure = [&]() {
		IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists_SetVisibility(targetVolume);
	};
	AllocateUsingOtherVolumeExpanded_Generic(targetVolume, sourceVolume, hashBlockMarkProcedure, allocationProcedure);

	IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().BuildUtilizedBlockListBasedOnVisibility(
			targetVolume,
			view,
			depth_camera_matrix);
}

// #define EXCEPT_ON_OUT_OF_SPACE
template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
AllocateHashEntriesUsingLists(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	const HashEntryAllocationState* hashEntryStates_device = volume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = volume->index.GetAllocationBlockCoordinates();

	const int hashEntryCount = volume->index.hashEntryCount;
	int lastFreeVoxelBlockId = volume->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = volume->index.GetLastFreeExcessListId();
	int* voxelAllocationList = volume->localVBA.GetAllocationList();
	int* excessAllocationList = volume->index.GetExcessAllocationList();
	HashEntry* hashTable = volume->index.GetEntries();

	for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
		const HashEntryAllocationState& hashEntryState = hashEntryStates_device[hashCode];
		switch (hashEntryState) {
			case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST:

				if (lastFreeVoxelBlockId >= 0) //there is room in the voxel block array
				{
					HashEntry hashEntry;
					hashEntry.pos = blockCoordinates_device[hashCode];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					hashTable[hashCode] = hashEntry;
					lastFreeVoxelBlockId--;
				}
#ifdef EXCEPT_ON_OUT_OF_SPACE
			else {
				DIEWITHEXCEPTION_REPORTLOCATION("Not enough space in ordered list.");
			}
#endif

				break;
			case NEEDS_ALLOCATION_IN_EXCESS_LIST:

				if (lastFreeVoxelBlockId >= 0 &&
				    lastFreeExcessListId >= 0) //there is room in the voxel block array and excess list
				{
					HashEntry hashEntry;
					hashEntry.pos = blockCoordinates_device[hashCode];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					int exlOffset = excessAllocationList[lastFreeExcessListId];
					hashTable[hashCode].offset = exlOffset + 1; //connect to child
					hashTable[ORDERED_LIST_SIZE + exlOffset] = hashEntry; //add child to the excess list
					lastFreeVoxelBlockId--;
					lastFreeExcessListId--;
				}
#ifdef EXCEPT_ON_OUT_OF_SPACE
			else {
				DIEWITHEXCEPTION_REPORTLOCATION("Not enough space in excess list.");
			}
#endif
				break;
			default:
				break;
		}
	}
	volume->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	volume->index.SetLastFreeExcessListId(lastFreeExcessListId);
}

template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
AllocateHashEntriesUsingLists_SetVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	const HashEntryAllocationState* hash_entry_allocation_states_device = volume->index.GetHashEntryAllocationStates();
	Vector3s* allocation_block_coordinates_device = volume->index.GetAllocationBlockCoordinates();
	HashBlockVisibility* hash_block_visibility_types_device = volume->index.GetBlockVisibilityTypes();

	int entry_count = volume->index.hashEntryCount;
	int last_free_voxel_block_id = volume->localVBA.lastFreeBlockId;
	int last_free_excess_list_id = volume->index.GetLastFreeExcessListId();
	int* voxel_allocation_list = volume->localVBA.GetAllocationList();
	int* excess_allocation_list = volume->index.GetExcessAllocationList();
	HashEntry* hash_table = volume->index.GetEntries();

	for (int hash = 0; hash < entry_count; hash++) {
		const HashEntryAllocationState& hash_entry_state = hash_entry_allocation_states_device[hash];
		switch (hash_entry_state) {
			case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST:

				if (last_free_voxel_block_id >= 0) //there is room in the voxel block array
				{
					HashEntry hashEntry;
					hashEntry.pos = allocation_block_coordinates_device[hash];
					hashEntry.ptr = voxel_allocation_list[last_free_voxel_block_id];
					hashEntry.offset = 0;
					hash_table[hash] = hashEntry;
					last_free_voxel_block_id--;
				} else {
					hash_block_visibility_types_device[hash] = INVISIBLE;
				}

				break;
			case NEEDS_ALLOCATION_IN_EXCESS_LIST:

				if (last_free_voxel_block_id >= 0 &&
				    last_free_excess_list_id >= 0) //there is room in the voxel block array and excess list
				{
					HashEntry hashEntry;
					hashEntry.pos = allocation_block_coordinates_device[hash];
					hashEntry.ptr = voxel_allocation_list[last_free_voxel_block_id];
					hashEntry.offset = 0;
					int exlOffset = excess_allocation_list[last_free_excess_list_id];
					hash_table[hash].offset = exlOffset + 1; //connect to child
					hash_table[ORDERED_LIST_SIZE + exlOffset] = hashEntry; //add child to the excess list
					hash_block_visibility_types_device[ORDERED_LIST_SIZE +
					                                   exlOffset] = IN_MEMORY_AND_VISIBLE; //make child visible and in memory
					last_free_voxel_block_id--;
					last_free_excess_list_id--;
				}
				break;
			default:
				break;
		}
	}
	volume->localVBA.lastFreeBlockId = last_free_voxel_block_id;
	volume->index.SetLastFreeExcessListId(last_free_excess_list_id);
}


template<MemoryDeviceType TMemoryDeviceType>
_DEVICE_WHEN_AVAILABLE_
inline int AllocateBlock_DEBUG(const CONSTPTR(Vector3s)& desired_block_position,
                               HashEntry* hash_table,
                               AtomicArrayThreadGuard<TMemoryDeviceType>& guard,
                               ATOMIC_ARGUMENT(int) last_free_voxel_block_id,
                               ATOMIC_ARGUMENT(int) last_free_excess_list_id,
                               int* block_allocation_list, int* excess_allocation_list) {

	int hash_code = HashCodeFromBlockPosition(desired_block_position);
	guard.lock(hash_code);
	HashEntry hash_entry = hash_table[hash_code];
	//check if hash table contains entry
	if (!(IS_EQUAL3(hash_entry.pos, desired_block_position) && hash_entry.ptr >= -1)) {
		if (hash_entry.ptr >= -1) {
			//search excess list only if there is no room in ordered part
			while (hash_entry.offset >= 1) {
				int new_hash_code = ORDERED_LIST_SIZE + hash_entry.offset - 1;
				guard.lock(new_hash_code);
				guard.release(hash_code);
				hash_code = new_hash_code;

				hash_entry = hash_table[hash_code];
				if (IS_EQUAL3(hash_entry.pos, desired_block_position) && hash_entry.ptr >= -1) {
					guard.release(hash_code);
					return hash_code;
				}
			}
			int block_index = ATOMIC_SUB(last_free_voxel_block_id, 1);
			int excess_list_index = ATOMIC_SUB(last_free_excess_list_id, 1);
			if (block_index >= 0 && excess_list_index >= 0) {
				int excess_list_offset = excess_allocation_list[excess_list_index];
				hash_table[hash_code].offset = excess_list_offset + 1;
				HashEntry& new_hash_entry = hash_table[ORDERED_LIST_SIZE + excess_list_offset];
				new_hash_entry.pos = desired_block_position;
				new_hash_entry.ptr = block_allocation_list[block_index];
				new_hash_entry.offset = 0;
			} else {
				if(block_index < 0){
					DIEWITHEXCEPTION_REPORTLOCATION("Out of block space...");
				}
				if(excess_list_index < 0){
					DIEWITHEXCEPTION_REPORTLOCATION("Out of excess list space...");
				}
				ATOMIC_ADD(last_free_voxel_block_id, 1);
				ATOMIC_ADD(last_free_excess_list_id, 1);
				guard.release(hash_code);
				return 0;
			}
			guard.release(hash_code);
			return hash_code;
		}
		int ordered_index = ATOMIC_SUB(last_free_voxel_block_id, 1);

		if (ordered_index >= 0) {
			HashEntry& new_hash_entry = hash_table[hash_code];
			new_hash_entry.pos = desired_block_position;
			new_hash_entry.ptr = block_allocation_list[ordered_index];
			new_hash_entry.offset = 0;
		} else {
			DIEWITHEXCEPTION_REPORTLOCATION("Out of block space...");
			ATOMIC_ADD(last_free_voxel_block_id, 1);
			guard.release(hash_code);
			return 0;
		}
		return hash_code;
	}
	guard.release(hash_code);
	// already have hash block, no allocation needed
	return hash_code;
}

template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateBlockList(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ORUtils::MemoryBlock<Vector3s>& block_coordinates,
		const int new_block_count) {

	int entry_count = volume->index.hashEntryCount;
	AllocationCounters<MEMORYDEVICE_CPU> counters(volume->localVBA.lastFreeBlockId,
	                                              volume->index.GetLastFreeExcessListId());
	int* block_allocation_list = volume->localVBA.GetAllocationList();
	int* excess_allocation_list = volume->index.GetExcessAllocationList();
	AtomicArrayThreadGuard<MEMORYDEVICE_CPU> guard(entry_count);

	HashEntry* hash_table = volume->index.GetEntries();
	const Vector3s* block_coordinates_device = block_coordinates.GetData(MEMORYDEVICE_CPU);

#if WITH_OPENMP
#pragma parallel for default(none) shared(block_coordinates_device, last_free_voxel_block_id, \
                                          last_free_excess_list_id, block_allocation_list, excess_allocation_list)
#endif
	for (int i_new_block = 0; i_new_block < new_block_count; i_new_block++) {
		AllocateBlock_DEBUG(block_coordinates_device[i_new_block], hash_table, guard, counters.last_free_voxel_block_id,
		                    counters.last_free_excess_list_id, block_allocation_list, excess_allocation_list);
	}
	volume->localVBA.lastFreeBlockId = GET_ATOMIC_VALUE(counters.last_free_voxel_block_id);
	volume->index.SetLastFreeExcessListId(GET_ATOMIC_VALUE(counters.last_free_excess_list_id));
}


template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::BuildUtilizedBlockListBasedOnVisibility(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view, const Matrix4f& depth_camera_matrix) {
	// ** scene data **
	const int hash_entry_count = volume->index.hashEntryCount;
	HashBlockVisibility* hash_block_visibility_types = volume->index.GetBlockVisibilityTypes();
	int* visible_hash_entry_codes = volume->index.GetUtilizedBlockHashCodes();
	HashEntry* hash_table = volume->index.GetEntries();
	bool useSwapping = volume->globalCache != nullptr;
	ITMHashSwapState* swapStates = volume->Swapping() ? volume->globalCache->GetSwapStates(false) : 0;

	// ** view data **
	Vector4f depthCameraProjectionParameters = view->calib.intrinsics_d.projectionParamsSimple.all;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = volume->sceneParams->voxel_size;

	int visibleEntryCount = 0;
	//build visible list
	for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
		HashBlockVisibility hash_block_visibility_type = hash_block_visibility_types[hash_code];
		const HashEntry& hash_entry = hash_table[hash_code];

		if (hash_block_visibility_type == 3) {
			bool is_visible_enlarged, is_visible;

			if (useSwapping) {
				checkBlockVisibility<true>(is_visible, is_visible_enlarged, hash_entry.pos, depth_camera_matrix,
				                           depthCameraProjectionParameters,
				                           voxelSize, depthImgSize);
				if (!is_visible_enlarged) hash_block_visibility_type = INVISIBLE;
			} else {
				checkBlockVisibility<false>(is_visible, is_visible_enlarged, hash_entry.pos, depth_camera_matrix,
				                            depthCameraProjectionParameters,
				                            voxelSize, depthImgSize);
				if (!is_visible) { hash_block_visibility_type = INVISIBLE; }
			}
			hash_block_visibility_types[hash_code] = hash_block_visibility_type;
		}

		if (useSwapping) {
			if (hash_block_visibility_type > 0 && swapStates[hash_code].state != 2) swapStates[hash_code].state = 1;
		}

		if (hash_block_visibility_type > 0) {
			visible_hash_entry_codes[visibleEntryCount] = hash_code;
			visibleEntryCount++;
		}
	}
	volume->index.SetUtilizedHashBlockCount(visibleEntryCount);
}

template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	HashBlockVisibility* utilized_block_visibility_types = volume->index.GetBlockVisibilityTypes();
	const int* utilized_block_hash_codes = volume->index.GetUtilizedBlockHashCodes();
	const int utilized_block_count = volume->index.GetUtilizedHashBlockCount();
#if WITH_OPENMP
#pragma omp parallel for default(none) shared(utilized_block_hash_codes, utilized_block_visibility_types)
#endif
	for (int i_visible_block = 0; i_visible_block < utilized_block_count; i_visible_block++) {
		utilized_block_visibility_types[utilized_block_hash_codes[i_visible_block]] =
				HashBlockVisibility::VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED;
	}
}

template<typename TVoxel>
HashEntry
IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::FindHashEntry(const VoxelBlockHash& index,
                                                                        const Vector3s& coordinates) {
	const HashEntry* entries = index.GetEntries();
	int hashCode = FindHashCodeAt(entries, coordinates);
	if (hashCode == -1) {
		return {Vector3s(0, 0, 0), 0, -2};
	} else {
		return entries[hashCode];
	}
}

template<typename TVoxel>
HashEntry
IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::FindHashEntry(const VoxelBlockHash& index,
                                                                        const Vector3s& coordinates,
                                                                        int& hashCode) {
	const HashEntry* entries = index.GetEntries();
	hashCode = FindHashCodeAt(entries, coordinates);
	if (hashCode == -1) {
		return {Vector3s(0, 0, 0), 0, -2};
	} else {
		return entries[hashCode];
	}
}

template<typename TVoxel>
bool IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateHashBlockAt(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hashCode) {

	HashEntry* hashTable = volume->index.GetEntries();
	int lastFreeVoxelBlockId = volume->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = volume->index.GetLastFreeExcessListId();
	int* voxelAllocationList = volume->localVBA.GetAllocationList();
	int* excessAllocationList = volume->index.GetExcessAllocationList();
	HashEntry* entry = nullptr;
	hashCode = -1;
	if (!FindOrAllocateHashEntry(at, hashTable, entry, lastFreeVoxelBlockId, lastFreeExcessListId,
	                             voxelAllocationList, excessAllocationList, hashCode)) {
		return false;
	}
	volume->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	volume->index.SetLastFreeExcessListId(lastFreeExcessListId);
	return true;
}