//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/9/20.
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

//ORUtils
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../Utils/HashBlockProperties.h"
#include "../../../Utils/Math.h"
#include "../../../Objects/Volume/RepresentationAccess.h"

#ifdef __CUDACC__
#include "../../../Utils/CUDAUtils.h"
#endif

namespace ITMLib {

// mostly for debugging purposes
enum ThreadAllocationStatus {
	ALREADY_IN_ORDERED_LIST,
	ALREADY_IN_EXCESS_LIST,
	MARK_FOR_ALLOCATION_IN_ORDERED_LIST,
	MARK_FOR_ALLOCATION_IN_EXCESS_LIST,
	TAKEN_CARE_OF_BY_ANOTHER_THREAD,
	BEING_MODIFIED_BY_ANOTHER_THREAD,
	LOGGED_HASH_COLLISION
};
_DEVICE_WHEN_AVAILABLE_
inline const char* ThreadAllocationStatusToString(const ThreadAllocationStatus& status) {
	switch (status) {
		case ALREADY_IN_ORDERED_LIST:
			return "ALREADY_IN_ORDERED_LIST";
		case ALREADY_IN_EXCESS_LIST:
			return "ALREADY_IN_EXCESS_LIST";
		case MARK_FOR_ALLOCATION_IN_ORDERED_LIST:
			return "MARK_FOR_ALLOCATION_IN_ORDERED_LIST";
		case MARK_FOR_ALLOCATION_IN_EXCESS_LIST:
			return "MARK_FOR_ALLOCATION_IN_EXCESS_LIST";
		case TAKEN_CARE_OF_BY_ANOTHER_THREAD:
			return "TAKEN_CARE_OF_BY_ANOTHER_THREAD";
		case BEING_MODIFIED_BY_ANOTHER_THREAD:
			return "BEING_MODIFIED_BY_ANOTHER_THREAD";
		case LOGGED_HASH_COLLISION:
			return "LOGGED_HASH_COLLISION";
		default:
			return "";
	}
}

/**
 * \brief Determines whether the hash block at the specified block position needs it's voxels to be allocated, as well
 * as whether they should be allocated in the excess list or the ordered list of the hash table.
 * If any of these are true, marks the corresponding entry in \param hash_entry_states
 * \param[in,out] hash_entry_states  array where to set the allocation type at final hashIdx index
 * \param[in,out] hash_block_coordinates  array block coordinates for the new hash blocks at final hashIdx index
 * \param[in,out] hash_code  takes in original index assuming coords, i.e. \refitem HashCodeFromBlockPosition(\param desired_hash_block_position),
 * returns final index of the hash block to be allocated (may be updated based on hash closed chaining)
 * \param[in] desired_hash_block_position  position of the hash block to check / allocate
 * \param[in] hash_table  hash table with existing blocks
 * \param[in] collisionDetected set to true if a block with the same hashcode has already been marked for allocation ( a collision occured )
 * \return true if the block needs allocation, false otherwise
 */
template<bool TEnableUnresolvableCollisionDetection>
_DEVICE_WHEN_AVAILABLE_
inline ThreadAllocationStatus MarkAsNeedingAllocationIfNotFound(ITMLib::HashEntryAllocationState* hash_entry_states,
                                                                Vector3s* hash_block_coordinates,
                                                                const CONSTPTR(Vector3s)& desired_hash_block_position,
                                                                const CONSTPTR(HashEntry)* hash_table,
                                                                THREADPTR(Vector3s)* colliding_block_positions,
                                                                ATOMIC_ARGUMENT(int) colliding_block_count) {

	int hash_code = HashCodeFromBlockPosition(desired_hash_block_position);
	HashEntry hash_entry = hash_table[hash_code];
	//check if hash table contains entry
	HashEntryAllocationState new_allocation_state;

	if (IS_EQUAL3(hash_entry.pos, desired_hash_block_position) && hash_entry.ptr >= -1) {
		return ALREADY_IN_ORDERED_LIST;
	} else {
		if (hash_entry.ptr >= -1) {
			//search excess list only if there is no room in ordered part
			while (hash_entry.offset >= 1) {
				hash_code = ORDERED_LIST_SIZE + hash_entry.offset - 1;
				hash_entry = hash_table[hash_code];

				if (IS_EQUAL3(hash_entry.pos, desired_hash_block_position) && hash_entry.ptr >= -1) {
					return ALREADY_IN_EXCESS_LIST;
				}
			}
			new_allocation_state = ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST;
		} else {
			new_allocation_state = ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST;
		}
	}

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
	if (atomicCAS((char*) (hash_entry_states + hash_code),
					  (char) ITMLib::NEEDS_NO_CHANGE,
					  (char) new_allocation_state) != (char) ITMLib::NEEDS_NO_CHANGE) {
			if (TEnableUnresolvableCollisionDetection) {
				if (IS_EQUAL3(hash_block_coordinates[hash_code], desired_hash_block_position)) {
					return BEING_MODIFIED_BY_ANOTHER_THREAD;
				}
			}
			int collision_index = ATOMIC_ADD(colliding_block_count, 1);
			colliding_block_positions[collision_index] = desired_hash_block_position;
			return LOGGED_HASH_COLLISION;
		} else {
			hash_block_coordinates[hash_code] = desired_hash_block_position;
			return (new_allocation_state == NEEDS_ALLOCATION_IN_EXCESS_LIST) ? MARK_FOR_ALLOCATION_IN_EXCESS_LIST
																			 : MARK_FOR_ALLOCATION_IN_ORDERED_LIST;
		}
#else
	ThreadAllocationStatus status;
#ifdef WITH_OPENMP
#pragma omp critical
#endif
	{
		if (hash_entry_states[hash_code] != ITMLib::NEEDS_NO_CHANGE) {
			if (IS_EQUAL3(hash_block_coordinates[hash_code], desired_hash_block_position)) {
				status = TAKEN_CARE_OF_BY_ANOTHER_THREAD;
			} else {
				//hash code already marked for allocation, but at different coordinates, cannot allocate
				int collision_index = ATOMIC_ADD(colliding_block_count, 1);
				colliding_block_positions[collision_index] = desired_hash_block_position;
				status = LOGGED_HASH_COLLISION;
			}
		} else {
			hash_entry_states[hash_code] = new_allocation_state;
			hash_block_coordinates[hash_code] = desired_hash_block_position;
			status = (new_allocation_state == NEEDS_ALLOCATION_IN_EXCESS_LIST) ? MARK_FOR_ALLOCATION_IN_EXCESS_LIST
			                                                                   : MARK_FOR_ALLOCATION_IN_ORDERED_LIST;
		}
	}
	return status;
#endif

};

// region =================== DEALLOCATION & CLEANING OF SINGLE VOXEL HASH BLOCK =======================================

/**
 * \brief Resets all voxels in voxel hash block to (given) default values
 * \tparam TVoxel type of voxel
 * \param last_free_voxel_block_id index of the next available block's offset in the voxel allocation list
 * \param voxel_allocation_list list of block offsets that are available for allocation in the future
 * \param voxels pointer to entire voxel blocks, i.e. where actual voxels for each voxel block are stored
 * \param entry_to_remove hash entry for block to clean
 * \param empty_voxel_block_device default (empty) voxel values for an entire block (copied to block being reset)
 */
template<typename TVoxel>
_DEVICE_WHEN_AVAILABLE_
inline void ResetVoxelBlock(ATOMIC_ARGUMENT(int) last_free_voxel_block_id, int* voxel_allocation_list, TVoxel* voxels,
                            const HashEntry& entry_to_remove, const TVoxel* empty_voxel_block_device) {
	int next_last_free_block_id = ATOMIC_ADD(last_free_voxel_block_id, 1) + 1;
	voxel_allocation_list[next_last_free_block_id] = entry_to_remove.ptr;
	TVoxel* voxel_block = voxels + entry_to_remove.ptr * VOXEL_BLOCK_SIZE3;
	memcpy(voxel_block, empty_voxel_block_device, sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
}

/**
 * \brief Deallocates a given block (resets the block's hash entry in hash table, resets the voxels it was pointing to).
 * \details If the state for the given hash code is not default (NEEDS_NO_CHANGE),
 * function adds the hash_code_to_remove to the colliding code list (incrementing the colliding_block_count)
 * and returns without altering anything.
 * \tparam TVoxel
 * \param hash_code_to_remove index of the entry in the hash table that is to be removed.
 * \param hash_entry_states states of the hash entries (for parallelization, see function description details).
 * \param hash_table pointer to the hash entries for the voxel block hash table
 * \param voxels pointer to the blocks' voxels
 * \param colliding_codes_device list of colliding block codes (for parallelization, see function description details).
 * \param colliding_block_count count of hash codes recorded to colliding_codes_device.
 * \param last_free_voxel_block_id
 * \param last_free_excess_list_id
 * \param voxel_allocation_list
 * \param excess_allocation_list
 * \param empty_voxel_block_device
 */
template<typename TVoxel>
_DEVICE_WHEN_AVAILABLE_
inline void DeallocateBlock(const Vector3s& block_position_to_remove,
                            ITMLib::HashEntryAllocationState* hash_entry_states,
                            THREADPTR(HashEntry)* hash_table,
                            THREADPTR(TVoxel)* voxels,
                            THREADPTR(Vector3s)* colliding_blocks_device,
                            ATOMIC_ARGUMENT(int) colliding_block_count,
                            ATOMIC_ARGUMENT(int) last_free_voxel_block_id,
                            ATOMIC_ARGUMENT(int) last_free_excess_list_id,
                            THREADPTR(int)* voxel_allocation_list,
                            THREADPTR(int)* excess_allocation_list,
                            const CONSTPTR(TVoxel)* empty_voxel_block_device) {

	const HashEntry default_entry =
			[]() {
				HashEntry default_entry;
				memset(&default_entry, 0, sizeof(HashEntry));
				default_entry.ptr = -2;
				return default_entry;
			}();

	int bucket_code = HashCodeFromBlockPosition(block_position_to_remove);

	bool collision = false;

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
	if(atomicCAS((char*) (hash_entry_states + bucket_code),
					  (char) ITMLib::NEEDS_NO_CHANGE,
					  (char) BUCKET_NEEDS_DEALLOCATION) != (char) ITMLib::NEEDS_NO_CHANGE){
			collision = true;
		}
#else
#pragma omp critical
	{
		if (hash_entry_states[bucket_code] == NEEDS_NO_CHANGE) {
			hash_entry_states[bucket_code] = BUCKET_NEEDS_DEALLOCATION;
		} else {
			collision = true;
		}
	};
#endif

/* if there is a bucket collision, we'll want to leave the block alone for now and
 process the block again on next run of the outer while loop, so that we don't get a
 data race on the intra-bucket entry connections */
	if (collision) {
		int collision_index = ATOMIC_ADD(colliding_block_count, 1);
		colliding_blocks_device[collision_index] = block_position_to_remove;
		return;
	}

	int hash_index_to_clear;

	if (!FindHashAtPosition(hash_index_to_clear, block_position_to_remove, hash_table)) {
		return;
	}

	HashEntry entry_to_remove = hash_table[hash_index_to_clear];


	ResetVoxelBlock(last_free_voxel_block_id, voxel_allocation_list, voxels, entry_to_remove,
	                empty_voxel_block_device);

	if (hash_index_to_clear < ORDERED_LIST_SIZE) {
		if (entry_to_remove.offset == 0) {
			// entry has no child in excess list, so it suffices to simply reset the removed entry.
			hash_table[hash_index_to_clear] = default_entry;
		} else {
			// entry has a child in excess list
			// we must move the child from the excess list to its parent's slot in the ordered list

			// the excess list gains a new empty slot from the former child (which will be moved to the ordered list)
			int next_last_free_excess_id = ATOMIC_ADD(last_free_excess_list_id, 1) + 1;
			excess_allocation_list[next_last_free_excess_id] = entry_to_remove.offset - 1;

			// finally, swap parent with child entry in the table and clean out former child's entry slot
			int child_hash_code = ORDERED_LIST_SIZE + entry_to_remove.offset - 1;
			hash_table[hash_index_to_clear] = hash_table[child_hash_code];

			hash_table[child_hash_code] = default_entry;
		}
	} else {
		// we must find direct parent of the excess list entry we're trying to remove
		int current_parent_code = bucket_code;
		int current_parent_offset = hash_table[current_parent_code].offset;
		int original_offset = hash_index_to_clear - ORDERED_LIST_SIZE;
		while (current_parent_offset != original_offset + 1) {
			current_parent_code = ORDERED_LIST_SIZE + current_parent_offset - 1;
			current_parent_offset = hash_table[current_parent_code].offset;
		}
		HashEntry& parent_entry = hash_table[current_parent_code];

		// the excess list gains a new empty slot from the removed entry
		int next_last_free_excess_id = ATOMIC_ADD(last_free_excess_list_id, 1) + 1;
		excess_allocation_list[next_last_free_excess_id] = parent_entry.offset - 1;

		if (entry_to_remove.offset == 0) {
			// entry has no child in excess list
			// the parent should lose its removed child
			parent_entry.offset = 0;
		} else {
			// Removed entry has child in excess list.
			// We must replace the excess offset of parent with that of the removed entry.
			// Parent should point to the removed entry's child in the excess list
			parent_entry.offset = entry_to_remove.offset;
		}
		// reset removed hash entry
		hash_table[hash_index_to_clear] = default_entry;
	}

}


// endregion ===========================================================================================================

} // namespace ITMLib