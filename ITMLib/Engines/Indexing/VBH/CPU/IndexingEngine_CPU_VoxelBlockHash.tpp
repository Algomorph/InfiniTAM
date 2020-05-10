//  ================================================================
//  Created by Gregory Kramida on 11/1/19.
//  Copyright (c) 2019-2000 Gregory Kramida
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

//ORUtils
#include "../../../../../ORUtils/PlatformIndependentAtomics.h"

// local
#include "IndexingEngine_CPU_VoxelBlockHash.h"
#include "../../../../Objects/Volume/RepresentationAccess.h"
#include "../../../EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../../Traversal/Interface/HashTableTraversal.h"
#include "../../Shared/IndexingEngine_Functors.h"
#include "../../../../Utils/Geometry/CheckBlockVisibility.h"
#include "../../../../Utils/Configuration/Configuration.h"
#include "../../../../Utils/Geometry/FrustumTrigonometry.h"
#include "../../../../Utils/MemoryBlock_StdContainer_Convertions.h"


using namespace ITMLib;


template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
AllocateHashEntriesUsingAllocationStateList_SetVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	const HashEntryAllocationState* hash_entry_allocation_states_device = volume->index.GetHashEntryAllocationStates();
	Vector3s* allocation_block_coordinates_device = volume->index.GetAllocationBlockCoordinates();
	HashBlockVisibility* hash_block_visibility_types_device = volume->index.GetBlockVisibilityTypes();

	int entry_count = volume->index.hash_entry_count;
	int last_free_voxel_block_id = volume->index.GetLastFreeBlockListId();
	int last_free_excess_list_id = volume->index.GetLastFreeExcessListId();
	int* voxel_allocation_list = volume->index.GetBlockAllocationList();
	int* excess_allocation_list = volume->index.GetExcessEntryList();
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
	volume->index.SetLastFreeBlockListId(last_free_voxel_block_id);
	volume->index.SetLastFreeExcessListId(last_free_excess_list_id);
}

//TODO: refactor to BuildVisibleBlockList..., use VisibleBlockList instead of UtilizedBlockList inside
template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::BuildUtilizedBlockListBasedOnVisibility(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view, const Matrix4f& depth_camera_matrix) {
	// ** scene data **
	const int hash_entry_count = volume->index.hash_entry_count;
	HashBlockVisibility* hash_block_visibility_types = volume->index.GetBlockVisibilityTypes();
	int* visible_hash_entry_codes = volume->index.GetUtilizedBlockHashCodes();
	HashEntry* hash_table = volume->index.GetEntries();
	const bool use_swapping = volume->SwappingEnabled();
	ITMHashSwapState* swapStates = volume->SwappingEnabled() ? volume->global_cache.GetSwapStates(false) : 0;

	// ** view data **
	Vector4f depthCameraProjectionParameters = view->calib.intrinsics_d.projectionParamsSimple.all;
	Vector2i depthImgSize = view->depth->dimensions;
	float voxelSize = volume->GetParameters().voxel_size;

	int visibleEntryCount = 0;
	//build visible list
	for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
		HashBlockVisibility hash_block_visibility_type = hash_block_visibility_types[hash_code];
		const HashEntry& hash_entry = hash_table[hash_code];

		if (hash_block_visibility_type == 3) {
			bool is_visible_enlarged, is_visible;

			if (use_swapping) {
				CheckVoxelHashBlockVisibility<true>(is_visible, is_visible_enlarged, hash_entry.pos,
				                                    depth_camera_matrix,
				                                    depthCameraProjectionParameters,
				                                    voxelSize, depthImgSize);
				if (!is_visible_enlarged) hash_block_visibility_type = INVISIBLE;
			} else {
				CheckVoxelHashBlockVisibility<false>(is_visible, is_visible_enlarged, hash_entry.pos,
				                                     depth_camera_matrix,
				                                     depthCameraProjectionParameters,
				                                     voxelSize, depthImgSize);
				if (!is_visible) { hash_block_visibility_type = INVISIBLE; }
			}
			hash_block_visibility_types[hash_code] = hash_block_visibility_type;
		}

		if (use_swapping) {
			if (hash_block_visibility_type > 0 && swapStates[hash_code].state != 2) swapStates[hash_code].state = 1;
		}

		if (hash_block_visibility_type > 0) {
			visible_hash_entry_codes[visibleEntryCount] = hash_code;
			visibleEntryCount++;
		}
	}
	volume->index.SetUtilizedBlockCount(visibleEntryCount);
}

template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	HashBlockVisibility* utilized_block_visibility_types = volume->index.GetBlockVisibilityTypes();
	const int* utilized_block_hash_codes = volume->index.GetUtilizedBlockHashCodes();
	const int utilized_block_count = volume->index.GetUtilizedBlockCount();
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
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hash_code) {

	HashEntry* hash_table = volume->index.GetEntries();
	int last_free_voxel_block_id = volume->index.GetLastFreeBlockListId();
	int last_free_excess_list_id = volume->index.GetLastFreeExcessListId();
	int utilized_block_count = volume->index.GetUtilizedBlockCount();
	int* block_allocation_list = volume->index.GetBlockAllocationList();
	int* excess_entry_list = volume->index.GetExcessEntryList();
	int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();
	HashEntry* entry = nullptr;
	hash_code = -1;
	if (!FindOrAllocateHashEntry(at, hash_table, entry, last_free_voxel_block_id, last_free_excess_list_id, utilized_block_count,
	                             block_allocation_list, excess_entry_list, utilized_hash_codes, hash_code)) {
		return false;
	}
	volume->index.SetLastFreeBlockListId(last_free_voxel_block_id);
	volume->index.SetLastFreeExcessListId(last_free_excess_list_id);
	volume->index.SetUtilizedBlockCount(utilized_block_count);
	return true;
}

using namespace ITMLib::internal;

template<typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume_OffsetAndBounded_Executor<MEMORYDEVICE_CPU, TVoxelTarget, TVoxelSource>::Execute(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume, const Extent3Di& source_bounds,
		const Vector3i& target_offset) {

	IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>& target_indexer =
			IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();

	Vector6i target_bounds(source_bounds.min_x + target_offset.x,
	                       source_bounds.min_y + target_offset.y,
	                       source_bounds.min_z + target_offset.z,
	                       source_bounds.max_x + target_offset.x,
	                       source_bounds.max_y + target_offset.y,
	                       source_bounds.max_z + target_offset.z);


	Vector3i target_min_point(target_bounds.min_x, target_bounds.min_y, target_bounds.min_z);
	Vector3i target_max_point(target_bounds.max_x, target_bounds.max_y, target_bounds.max_z);
	Vector3i min_point_block, max_point_block;
	pointToVoxelBlockPos(target_min_point, min_point_block);
	pointToVoxelBlockPos(target_max_point, max_point_block);

	HashEntry* target_hash_table = target_volume->index.GetEntries();

	HashEntryAllocationState* hash_entry_allocation_states = target_volume->index.GetHashEntryAllocationStates();
	Vector3s* hash_block_coordinates = target_volume->index.GetAllocationBlockCoordinates();

	ORUtils::MemoryBlock<Vector3s> colliding_block_positions(target_volume->index.hash_entry_count, MEMORYDEVICE_CPU);
	Vector3s* colliding_block_positions_device = colliding_block_positions.GetData(MEMORYDEVICE_CPU);
	std::atomic<int> colliding_block_count;
	bool unresolvable_collision_encountered = false;
	do {
		target_volume->index.ClearHashEntryAllocationStates();
		colliding_block_count.store(0);
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(min_point_block, max_point_block, hash_entry_allocation_states, \
                                              hash_block_coordinates, colliding_block_positions_device, \
                                              colliding_block_count, target_hash_table, unresolvable_collision_encountered)
#endif
		for (int block_z = min_point_block.z; block_z < max_point_block.z; block_z++) {
			for (int block_y = min_point_block.y; block_y < max_point_block.y; block_y++) {
				for (int block_x = min_point_block.x; block_x < max_point_block.x; block_x++) {
					Vector3s new_block_position(block_x, block_y, block_x);
					ThreadAllocationStatus status =
							MarkAsNeedingAllocationIfNotFound<true>(
									hash_entry_allocation_states, hash_block_coordinates, new_block_position,
									target_hash_table, colliding_block_positions_device, colliding_block_count);
					if (status == BEING_MODIFIED_BY_ANOTHER_THREAD) {
						unresolvable_collision_encountered = true;
					}
				}
			}
		}

		target_indexer.AllocateHashEntriesUsingAllocationStateList(target_volume);
		target_indexer.AllocateBlockList(target_volume, colliding_block_positions, colliding_block_count.load());
	} while (unresolvable_collision_encountered);
}