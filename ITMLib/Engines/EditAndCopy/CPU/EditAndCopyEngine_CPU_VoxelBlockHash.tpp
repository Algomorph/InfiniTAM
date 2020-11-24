//  ================================================================
//  Created by Gregory Kramida on 9/26/19.
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


#include "EditAndCopyEngine_CPU.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../DepthFusion/DepthFusionEngineFactory.h"
#include "../../Traversal/Shared/VolumeTraversal_Shared.h"
#include "../Shared/EditAndCopyEngine_Shared.h"
#include "../../Indexing/Interface/IndexingEngine.h"
#include "../../../Utils/Geometry/GeometryBooleanOperations.h"

using namespace ITMLib;

// region ==================================== Voxel Hash Volume EditAndCopy Engine ====================================

template<typename TVoxel>
void EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::ResetVolume(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	const int block_count = volume->index.GetMaximumBlockCount();
	const int block_size = volume->index.GetVoxelBlockSize();

	TVoxel* voxels = volume->GetVoxels();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(voxels) firstprivate(block_count, block_size)
#endif
	for (int i_voxel = 0; i_voxel < block_count * block_size; i_voxel++) voxels[i_voxel] = TVoxel();
	int* block_allocation_list = volume->index.GetBlockAllocationList();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(block_allocation_list) firstprivate(block_count)
#endif
	for (int i_block_index = 0; i_block_index < block_count; i_block_index++)
		block_allocation_list[i_block_index] = i_block_index;

	volume->index.SetLastFreeBlockListId(block_count - 1);

	const HashEntry default_entry = [](){
		HashEntry default_entry;
		memset(&default_entry, 0, sizeof(HashEntry));
		default_entry.ptr = -2;
		return default_entry;
	}();

	HashEntry* hash_table = volume->index.GetEntries();
	const int entry_count = volume->index.hash_entry_count;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(hash_table) firstprivate(entry_count, default_entry)
#endif
	for (int i_entry = 0; i_entry < entry_count; i_entry++) hash_table[i_entry] = default_entry;
	int* excess_entry_list = volume->index.GetExcessEntryList();
	const int excess_list_size = volume->index.GetExcessListSize();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(excess_entry_list) firstprivate(excess_list_size)
#endif
	for (int i_excess_entry = 0; i_excess_entry < excess_list_size; i_excess_entry++)
		excess_entry_list[i_excess_entry] = i_excess_entry;

	volume->index.SetLastFreeExcessListId(volume->index.GetExcessListSize() - 1);
	volume->index.SetUtilizedBlockCount(0);
}

template<typename TVoxel>
bool
EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::SetVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
                                                        Vector3i at, TVoxel voxel) {

	HashEntry* hash_table = volume->index.GetEntries();
	TVoxel* voxels = volume->GetVoxels();
	int hash_code = -1;
	Vector3s block_position;
	int linear_index_in_block = pointToVoxelBlockPos(at, block_position);
	if (IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
			.AllocateHashBlockAt(volume, block_position, hash_code)) {
		HashEntry& entry = hash_table[hash_code];
		TVoxel* local_voxel_block = &(voxels[entry.ptr * (VOXEL_BLOCK_SIZE3)]);
		local_voxel_block[linear_index_in_block] = voxel;
		return true;
	} else {
		return false;
	}
}

template<typename TVoxel>
bool EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::SetVoxelNoAllocation(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume,
		Vector3i at, TVoxel voxel) {
	HashEntry* hash_table = volume->index.GetEntries();
	TVoxel* voxels = volume->GetVoxels();
	Vector3i block_position;
	int index_in_block = pointToVoxelBlockPos(at, block_position);
	int hash_index;
	if (FindHashAtPosition(hash_index, block_position.toShortFloor(), hash_table)) {
		TVoxel* local_voxel_block = &(voxels[hash_index]);
		local_voxel_block[index_in_block] = voxel;
	} else {
		return false;
	}
	return true;
}


template<typename TVoxel>
TVoxel
EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::ReadVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
                                                         Vector3i at) {
	TVoxel* voxels = volume->GetVoxels();
	HashEntry* hash_table = volume->index.GetEntries();
	int vm_index;
	return readVoxel(voxels, hash_table, at, vm_index);
}

template<typename TVoxel>
TVoxel
EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::ReadVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
                                                         Vector3i at,
                                                         VoxelBlockHash::IndexCache& cache) {
	TVoxel* voxels = volume->GetVoxels();
	HashEntry* hash_table = volume->index.GetEntries();
	int vm_index;
	return readVoxel(voxels, hash_table, at, vm_index, cache);
}

template<typename TVoxel>
void
EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::OffsetWarps(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
                                                           Vector3f offset) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented!");
}

template<typename TVoxel>
bool EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::CopyVolumeSlice(
		VoxelVolume<TVoxel, VoxelBlockHash>* target_volume, VoxelVolume<TVoxel, VoxelBlockHash>* source_volume,
		Vector6i bounds, const Vector3i& offset) {

	assert(target_volume->index.hash_entry_count == source_volume->index.hash_entry_count);

	//temporary stuff
	const int hash_entry_count = source_volume->index.hash_entry_count;
	ORUtils::MemoryBlock<HashEntryAllocationState> hashEntryStates(hash_entry_count, MEMORYDEVICE_CPU);
	HashEntryAllocationState* hash_entry_states_device = hashEntryStates.GetData(MEMORYDEVICE_CPU);
	ORUtils::MemoryBlock<Vector3s> block_coordinates(hash_entry_count, MEMORYDEVICE_CPU);
	Vector3s* block_coordinates_device = block_coordinates.GetData(MEMORYDEVICE_CPU);

	TVoxel* source_voxels = source_volume->GetVoxels();
	const HashEntry* source_hash_table = source_volume->index.GetEntries();

	HashEntry* destination_hash_table = target_volume->index.GetEntries();
	TVoxel* destination_voxels = target_volume->GetVoxels();

	bool voxels_were_copied = false;

	if (offset == Vector3i(0)) {
		// *** allocate missing entries in target hash table
		// traverse source hash blocks, see which ones are at least partially inside the specified bounds

		internal::AllocateUsingOtherVolume_Bounded<MEMORYDEVICE_CPU>(target_volume, source_volume, bounds);

		//iterate over source hash blocks & fill in the target hash blocks
		for (int source_hash_code = 0; source_hash_code < hash_entry_count; source_hash_code++) {
			const HashEntry& source_hash_entry = source_hash_table[source_hash_code];

			if (source_hash_entry.ptr < 0) continue;
			int destination_hash;
			FindHashAtPosition(destination_hash, source_hash_entry.pos, destination_hash_table);
			const HashEntry& destinationHashEntry = destination_hash_table[destination_hash];

			//position of the current entry in 3D space (in voxel units)
			Vector3i source_block_position_voxels = source_hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			TVoxel* local_source_voxel_block = &(source_voxels[source_hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel* local_destination_voxel_block = &(destination_voxels[destinationHashEntry.ptr *
			                                                             (VOXEL_BLOCK_SIZE3)]);
			if (IsHashBlockFullyInBounds(source_block_position_voxels, bounds)) {
				//we can safely copy the whole block
				memcpy(local_destination_voxel_block, local_source_voxel_block, sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
				voxels_were_copied = true;
			} else if (IsHashBlockPartiallyInBounds(source_block_position_voxels, bounds)) {
				//we have to copy only parts of the scene that are within bounds
				int z_range_start, z_range_end, y_range_start, y_range_end, x_range_start, x_range_end;
				ComputeCopyRanges(x_range_start, x_range_end, y_range_start, y_range_end, z_range_start, z_range_end,
				                  source_block_position_voxels, bounds);
				for (int z = z_range_start; z < z_range_end; z++) {
					for (int y = y_range_start; y < y_range_end; y++) {
						for (int x = x_range_start; x < x_range_end; x++) {
							int i_voxel_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
							memcpy(&local_destination_voxel_block[i_voxel_in_block], &local_source_voxel_block[i_voxel_in_block],
							       sizeof(TVoxel));
						}
					}
				}
				voxels_were_copied = true;
			}

		}
	} else {
		//non-zero-offset case
		internal::AllocateUsingOtherVolume_OffsetAndBounded<MEMORYDEVICE_CPU>(target_volume, source_volume, bounds, offset);

		VoxelBlockHash::IndexCache source_cache;

		for (int source_z = bounds.min_z; source_z < bounds.max_z; source_z++) {
			for (int source_y = bounds.min_y; source_y < bounds.min_y; source_y++) {
				for (int source_x = bounds.min_x; source_x < bounds.max_x; source_x++) {
					Vector3i source_point(source_x, source_y, source_z);
					Vector3i destination_point = source_point + offset;
					TVoxel source_voxel =
							EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::
							ReadVoxel(source_volume, source_point, source_cache);
					EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::
					SetVoxelNoAllocation(target_volume, destination_point, source_voxel);
					voxels_were_copied = true;
				}
			}
		}
	}

	return voxels_were_copied;
}

template<typename TVoxel>
bool EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::CopyVolume(
		VoxelVolume<TVoxel, VoxelBlockHash>* target_volume, VoxelVolume<TVoxel, VoxelBlockHash>* source_volume,
		const Vector3i& offset) {

	assert(target_volume->index.hash_entry_count == source_volume->index.hash_entry_count);

	//reset destination scene
	EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::ResetVolume(target_volume);

	const int hash_entry_count = source_volume->index.hash_entry_count;

	TVoxel* source_voxels = source_volume->GetVoxels();
	const HashEntry* source_hash_table = source_volume->index.GetEntries();

	HashEntry* destination_hash_table = target_volume->index.GetEntries();
	TVoxel* destination_voxels = target_volume->GetVoxels();

	bool voxels_were_copied = false;

	if (offset == Vector3i(0)) {
		internal::AllocateUsingOtherVolume<MEMORYDEVICE_CPU>(target_volume, source_volume);
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(source_hash_table, destination_hash_table, source_voxels, destination_voxels, voxels_were_copied) \
firstprivate(hash_entry_count)
#endif
		for (int i_source_hash_entry = 0; i_source_hash_entry < hash_entry_count; i_source_hash_entry++) {
			const HashEntry& sourceHashEntry = source_hash_table[i_source_hash_entry];

			if (sourceHashEntry.ptr < 0) continue;
			int i_destination_hash_entry;
			FindHashAtPosition(i_destination_hash_entry, sourceHashEntry.pos, destination_hash_table);
			const HashEntry& destinationHashEntry = destination_hash_table[i_destination_hash_entry];
			//position of the current entry in 3D space (in voxel units)
			TVoxel* localSourceVoxelBlock = &(source_voxels[sourceHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel* localDestinationVoxelBlock = &(destination_voxels[destinationHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			//we can safely copy the whole block
			memcpy(localDestinationVoxelBlock, localSourceVoxelBlock, sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
			voxels_were_copied = true;
		}
	} else {
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(source_hash_table, source_voxels, voxels_were_copied, offset, target_volume) \
firstprivate(hash_entry_count)
#endif
		// traverse source hash blocks
		for (int i_source_hash_entry = 0; i_source_hash_entry < hash_entry_count; i_source_hash_entry++) {
			const HashEntry& currentSourceHashEntry = source_hash_table[i_source_hash_entry];
			if (currentSourceHashEntry.ptr < 0) continue;

			Vector3i source_block_position_voxels;
			source_block_position_voxels.x = currentSourceHashEntry.pos.x;
			source_block_position_voxels.y = currentSourceHashEntry.pos.y;
			source_block_position_voxels.z = currentSourceHashEntry.pos.z;
			source_block_position_voxels *= VOXEL_BLOCK_SIZE;

			TVoxel* voxel_block = &(source_voxels[currentSourceHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);

			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int voxel_index_in_block;
						Vector3i source_point(source_block_position_voxels.x + x, source_block_position_voxels.y + y, source_block_position_voxels.z + z);
						Vector3i destination_point = source_point + offset;
						voxel_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel source_voxel = voxel_block[voxel_index_in_block];
						EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::
						SetVoxel(target_volume, destination_point, source_voxel);
						voxels_were_copied = true;
					}
				}
			}
		}
	}
	return voxels_were_copied;
}

// endregion ===========================================================================================================

