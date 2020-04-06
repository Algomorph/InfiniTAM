//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../../Objects/Views/View.h"
#include "../../../Objects/Tracking/CameraTrackingState.h"
#include "../../Common/WarpAccessFunctors.h"
#include "../../Common/AllocationTempData.h"
#include "../../../Utils/Math.h"
#include "../../../Utils/PixelUtils.h"
#include "../../../Utils/VoxelFlags.h"
#include "../../../Utils/HashBlockProperties.h"
#include "../../../Utils/Geometry/IntersectionChecks.h"
#include "../../../Utils/CLionCUDAsyntax.h"

#ifdef __CUDACC__

#include "../../../Utils/CUDAUtils.h"

#endif

using namespace ITMLib;

template<MemoryDeviceType TMemoryDeviceType>
struct AllocationCounters {
	AllocationCounters(int _last_free_voxel_block_id, int _last_free_excess_list_id) {
		INITIALIZE_ATOMIC(int, last_free_voxel_block_id, _last_free_voxel_block_id);
		INITIALIZE_ATOMIC(int, last_free_excess_list_id, _last_free_excess_list_id);
	}

	int GetLastFreeVoxelBlockId() {
		return GET_ATOMIC_VALUE_CPU(last_free_voxel_block_id);
	}

	int GetLastFreeExcesListId() {
		return GET_ATOMIC_VALUE_CPU(last_free_excess_list_id);
	}

	~AllocationCounters() {
		CLEAN_UP_ATOMIC(last_free_voxel_block_id);CLEAN_UP_ATOMIC(last_free_excess_list_id);
	}

	DECLARE_ATOMIC(int, last_free_voxel_block_id);
	DECLARE_ATOMIC(int, last_free_excess_list_id);
	DECLARE_ATOMIC(int, utilized_block_count);
};

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
inline void DeallocateBlock(const int hash_code_to_remove,
                            ITMLib::HashEntryAllocationState* hash_entry_states,
                            THREADPTR(HashEntry)* hash_table,
                            THREADPTR(TVoxel)* voxels,
                            THREADPTR(int)* colliding_codes_device,
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

	const HashEntry& entry_to_remove = hash_table[hash_code_to_remove];
	int bucket_code;
	bool entry_in_ordered_list;
	if (hash_code_to_remove < ORDERED_LIST_SIZE) {
		bucket_code = hash_code_to_remove;
		entry_in_ordered_list = true;
	} else {
		bucket_code = HashCodeFromBlockPosition(entry_to_remove.pos);
		entry_in_ordered_list = false;
	}

	bool collision = false;
#pragma omp critical
	{
		if (hash_entry_states[bucket_code] == NEEDS_NO_CHANGE) {
			hash_entry_states[bucket_code] = BUCKET_NEEDS_DEALLOCATION;
		} else {
			collision = true;
		}
	};
	/* if there is a bucket collision, we'll want to leave the block alone for now and
	 process the block again on next run of the outer while loop, so that we don't get a
	 data race on the intra-bucket entry connections */
	if (collision) {
		int collision_index = ATOMIC_ADD(colliding_block_count, 1);
		colliding_codes_device[collision_index] = hash_code_to_remove;
		return;
	}

	ResetVoxelBlock(last_free_voxel_block_id, voxel_allocation_list, voxels, entry_to_remove,
	                empty_voxel_block_device);

	if (entry_in_ordered_list) {
		if (entry_to_remove.offset == 0) {
			// entry has no child in excess list, so it suffices to simply reset the removed entry.
			hash_table[hash_code_to_remove] = default_entry;
		} else {
			// entry has a child in excess list
			// we must move the child from the excess list to its parent's slot in the ordered list

			// the excess list gains a new empty slot from the former child (which will be moved to the ordered list)
			int next_last_free_excess_id = ATOMIC_ADD(last_free_excess_list_id, 1) + 1;
			excess_allocation_list[next_last_free_excess_id] = entry_to_remove.offset - 1;

			// finally, swap parent with child entry in the table and clean out former child's entry slot
			int child_hash_code = ORDERED_LIST_SIZE + entry_to_remove.offset;
			hash_table[hash_code_to_remove] = hash_table[child_hash_code];
			hash_table[child_hash_code] = default_entry;
		}
	} else {
		// we must find direct parent of the excess list entry we're trying to remove
		int current_parent_code = bucket_code;
		int current_parent_offset = hash_table[current_parent_code].offset;
		int original_offset = hash_code_to_remove - ORDERED_LIST_SIZE;
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
		hash_table[hash_code_to_remove] = default_entry;
	}

}

// endregion ===========================================================================================================

_CPU_AND_GPU_CODE_
inline bool
HashBlockAllocatedAtOffset(const CONSTPTR(ITMLib::VoxelBlockHash::IndexData)* target_index,
                           const THREADPTR(Vector3s)& block_position,
                           const CONSTPTR(Vector6i)& offset_block_range) {

	bool allocated = false;
	for (int z = block_position.z + offset_block_range.min_z; z < block_position.z + offset_block_range.max_z; z++) {
		for (int y = block_position.y + offset_block_range.min_y;
		     y < block_position.y + offset_block_range.max_y; y++) {
			for (int x = block_position.x + offset_block_range.min_x;
			     x < block_position.x + offset_block_range.max_x; x++) {
				if (FindHashCodeAt(target_index, Vector3s(x, y, z)) != -1) {
					allocated = true;
				}
			}
		}
	}
	return allocated;
}




/**Get number of differing vector components**/
_CPU_AND_GPU_CODE_ inline int get_differing_component_count(Vector3s coord1, Vector3s coord2) {
	return static_cast<int>(coord1.x != coord2.x) +
	       static_cast<int>(coord1.y != coord2.y) +
	       static_cast<int>(coord1.z != coord2.z);
}

_DEVICE_WHEN_AVAILABLE_ inline void
markVoxelHashBlocksAlongSegment(ITMLib::HashEntryAllocationState* hash_entry_allocation_states,
                                Vector3s* hash_block_coordinates,
                                bool& unresolvable_collision_encountered,
                                const CONSTPTR(HashEntry)* hash_table,
                                const ITMLib::Segment& segment_in_hash_blocks,
                                THREADPTR(Vector3s)* colliding_block_positions,
                                ATOMIC_ARGUMENT(int) colliding_block_count) {

	// number of steps to take along the truncated SDF band
	int step_count = (int) std::ceil(2.0f * segment_in_hash_blocks.length());

	// a single stride along the sdf band segment from one step to the next
	Vector3f strideVector = segment_in_hash_blocks.direction / (float) (step_count - 1);

	Vector3s previous_block_position;
	Vector3f check_position = segment_in_hash_blocks.origin;

	//add neighbouring blocks
	for (int i = 0; i < step_count; i++) {
		//find block position at current step
		Vector3s current_block_position = TO_SHORT_FLOOR3(check_position);
		int directional_increment_count;
		if (i > 0 && (directional_increment_count = get_differing_component_count(current_block_position,
		                                                                          previous_block_position)) > 1) {
			if (directional_increment_count == 2) {
				for (int i_direction = 0; i_direction < 3; i_direction++) {
					if (current_block_position.values[i_direction] != previous_block_position.values[i_direction]) {
						Vector3s potentially_missed_block_position = previous_block_position;
						potentially_missed_block_position.values[i_direction] = current_block_position.values[i_direction];
						if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
						                                       TO_FLOAT3(potentially_missed_block_position),
						                                       1.0f)) {
							if (MarkAsNeedingAllocationIfNotFound<true>(
									hash_entry_allocation_states,
									hash_block_coordinates, potentially_missed_block_position,
									hash_table, colliding_block_positions, colliding_block_count) ==
							    BEING_MODIFIED_BY_ANOTHER_THREAD) {
								unresolvable_collision_encountered = true;
							}
						}
					}
				}
			} else {
				//directional_increment_count == 3
				for (int iDirection = 0; iDirection < 3; iDirection++) {
					Vector3s potentially_missed_block_position = previous_block_position;
					potentially_missed_block_position.values[iDirection] = current_block_position.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
					                                       TO_FLOAT3(potentially_missed_block_position),
					                                       1.0f)) {
						if (MarkAsNeedingAllocationIfNotFound<true>(
								hash_entry_allocation_states,
								hash_block_coordinates, potentially_missed_block_position,
								hash_table, colliding_block_positions, colliding_block_count) ==
						    BEING_MODIFIED_BY_ANOTHER_THREAD) {
							unresolvable_collision_encountered = true;
						}
					}
					potentially_missed_block_position = current_block_position;
					potentially_missed_block_position.values[iDirection] = previous_block_position.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
					                                       TO_FLOAT3(potentially_missed_block_position),
					                                       1.0f)) {
						if (MarkAsNeedingAllocationIfNotFound<true>(
								hash_entry_allocation_states,
								hash_block_coordinates, potentially_missed_block_position,
								hash_table, colliding_block_positions, colliding_block_count) ==
						    BEING_MODIFIED_BY_ANOTHER_THREAD) {
							unresolvable_collision_encountered = true;
						}
					}
				}
			}
		}
		if (MarkAsNeedingAllocationIfNotFound<true>(
				hash_entry_allocation_states,
				hash_block_coordinates, current_block_position,
				hash_table, colliding_block_positions, colliding_block_count) == BEING_MODIFIED_BY_ANOTHER_THREAD) {
			unresolvable_collision_encountered = true;
		}
		check_position += strideVector;
		previous_block_position = current_block_position;
	}
}


_CPU_AND_GPU_CODE_
inline Vector3f cameraVoxelSpaceToWorldHashBlockSpace(const Vector4f& point_camera_space_voxels,
                                                      const Matrix4f& inverted_camera_pose,
                                                      const float one_over_hash_block_size) {
	// account for the fact that voxel coordinates represent the voxel center, and we need the extreme corner position of
	// the hash block, i.e. 0.5 voxel (1/16 block) offset from the position along the ray
	return (TO_VECTOR3(inverted_camera_pose * point_camera_space_voxels)) * one_over_hash_block_size
	       + Vector3f(1.0f / (2.0f * VOXEL_BLOCK_SIZE));
}

_CPU_AND_GPU_CODE_
inline float normOfFirst3Components(const Vector4f& vec) {
	return sqrt(vec.x * vec.x + vec.y + vec.y + vec.z * vec.z);
}
/**
 * \brief compute_allocated the segment formed by tracing the camera-space point a fixed distance forward and backward along
 * the ray; note: the starting & ending points of the returned segment are in voxel block coordinates
 * \param distance_from_point distance to trace backward and forward along the ray
 * \param point_in_camera_space point to trace from
 * \param inverted_camera_pose
 * \param one_over_hash_block_size
 * \return resulting segment in voxel block coordinates
 */
_CPU_AND_GPU_CODE_
inline ITMLib::Segment findHashBlockSegmentAlongCameraRayWithinRangeFromPoint(
		const float distance_from_point,
		const Vector4f& point_in_camera_space,
		const Matrix4f& inverted_camera_pose,
		const float one_over_hash_block_size) {

	// distance to the point along camera ray
	float norm = normOfFirst3Components(point_in_camera_space);

	Vector4f endpoint_in_camera_space;

	endpoint_in_camera_space = point_in_camera_space * (1.0f - distance_from_point / norm);
	endpoint_in_camera_space.w = 1.0f;
	//start position along ray in hash blocks
	Vector3f start_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpace(
			endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);

	endpoint_in_camera_space = point_in_camera_space * (1.0f + distance_from_point / norm);
	endpoint_in_camera_space.w = 1.0f;
	//end position of the segment to march along the ray
	Vector3f end_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpace(
			endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);

	// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
	// end of the (truncated SDF) band (increased by backBandFactor), along the ray cast from the camera through the
	// point, in camera space
	ITMLib::Segment segment(start_point_in_hash_blocks, end_point_in_hash_blocks);
	return segment;
}

_CPU_AND_GPU_CODE_
inline ITMLib::Segment findHashBlockSegmentAlongCameraRayWithinRangeFromAndBetweenTwoPoints(
		const float range,
		const Vector4f& point1_in_camera_space,
		const Vector4f& point2_in_camera_space,
		const Matrix4f& inverted_camera_pose,
		const float one_over_hash_block_size) {

	Vector4f endpoint_in_camera_space;
	float norm;
	if (point2_in_camera_space.z > point1_in_camera_space.z) {
		norm = normOfFirst3Components(point1_in_camera_space);
		endpoint_in_camera_space = point1_in_camera_space * (1.0f - range / norm);
		endpoint_in_camera_space.w = 1.0f;
		Vector3f start_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpace(
				endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);
		norm = normOfFirst3Components(point2_in_camera_space);
		endpoint_in_camera_space = point2_in_camera_space * (1.0f + range / norm);
		Vector3f end_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpace(
				endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);
		return ITMLib::Segment(start_point_in_hash_blocks, end_point_in_hash_blocks);
	} else {
		norm = normOfFirst3Components(point2_in_camera_space);
		endpoint_in_camera_space = point2_in_camera_space * (1.0f - range / norm);
		endpoint_in_camera_space.w = 1.0f;
		Vector3f start_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpace(
				endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);
		norm = normOfFirst3Components(point1_in_camera_space);
		endpoint_in_camera_space = point1_in_camera_space * (1.0f + range / norm);
		Vector3f end_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpace(
				endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);
		return ITMLib::Segment(start_point_in_hash_blocks, end_point_in_hash_blocks);
	}
}

_CPU_AND_GPU_CODE_
inline Vector4f imageSpacePointToCameraSpace(const float depth, const int x, const int y,
                                             const Vector4f& inverted_camera_projection_parameters) {
	return {depth *
	        ((float(x) - inverted_camera_projection_parameters.z) * inverted_camera_projection_parameters.x),
	        depth *
	        ((float(y) - inverted_camera_projection_parameters.w) * inverted_camera_projection_parameters.y),
	        depth, 1.0f};
}

_CPU_AND_GPU_CODE_
inline Vector4f imageSpacePointToCameraSpace(const float depth, const float x, const float y,
                                             const Vector4f& inverted_camera_projection_parameters) {
	return {depth *
	        ((x - inverted_camera_projection_parameters.z) * inverted_camera_projection_parameters.x),
	        depth *
	        ((y - inverted_camera_projection_parameters.w) * inverted_camera_projection_parameters.y),
	        depth, 1.0f};
}

_CPU_AND_GPU_CODE_
inline ITMLib::Segment
findHashBlockSegmentAlongCameraRayWithinRangeFromDepth(const float distance_from_point, const float depth_measure,
                                                       const int x, const int y, const Matrix4f& inverted_camera_pose,
                                                       const Vector4f& inverted_camera_projection_parameters,
                                                       const float one_over_hash_block_size) {

	Vector4f point_in_camera_space = imageSpacePointToCameraSpace(depth_measure, x, y,
	                                                              inverted_camera_projection_parameters);
	return findHashBlockSegmentAlongCameraRayWithinRangeFromPoint(distance_from_point,
	                                                              point_in_camera_space,
	                                                              inverted_camera_pose,
	                                                              one_over_hash_block_size);
}


_DEVICE_WHEN_AVAILABLE_ inline void
findVoxelBlocksForRayNearSurface(ITMLib::HashEntryAllocationState* hash_entry_allocation_states,
                                 Vector3s* hash_block_coordinates,
                                 bool& unresolvable_collision_encountered,
                                 const CONSTPTR(HashEntry)* hash_table,
                                 const int x, const int y, const CONSTPTR(float)* depth, float surface_distance_cutoff,
                                 const Matrix4f inverted_camera_pose,
                                 const Vector4f inverted_projection_parameters,
                                 const float one_over_hash_block_size, const Vector2i depth_image_size,
                                 const float near_clipping_distance, const float far_clipping_distance,
                                 THREADPTR(Vector3s)* colliding_block_positions,
                                 ATOMIC_ARGUMENT(int) colliding_block_count) {

	float depth_measure = depth[x + y * depth_image_size.x];
	if (depth_measure <= 0 || (depth_measure - surface_distance_cutoff) < 0 ||
	    (depth_measure - surface_distance_cutoff) < near_clipping_distance ||
	    (depth_measure + surface_distance_cutoff) > far_clipping_distance)
		return;

	// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
	// end of the (truncated SDF) band (increased by backBandFactor), along the ray cast from the camera through the
	// point, in camera space
	ITMLib::Segment march_segment = findHashBlockSegmentAlongCameraRayWithinRangeFromDepth(
			surface_distance_cutoff, depth_measure, x, y, inverted_camera_pose,
			inverted_projection_parameters, one_over_hash_block_size);


	markVoxelHashBlocksAlongSegment(hash_entry_allocation_states, hash_block_coordinates,
	                                unresolvable_collision_encountered,
	                                hash_table, march_segment, colliding_block_positions, colliding_block_count);
}


_CPU_AND_GPU_CODE_
inline
bool FindOrAllocateHashEntry(const Vector3s& hash_entry_position, HashEntry* hash_table, HashEntry*& result_entry,
                             int& last_free_voxel_block_id, int& last_free_excess_list_id,
                             const int* voxel_allocation_list,
                             const int* excess_allocation_list, int& hash_code) {
	hash_code = HashCodeFromBlockPosition(hash_entry_position);
	HashEntry hash_entry = hash_table[hash_code];
	if (!IS_EQUAL3(hash_entry.pos, hash_entry_position) || hash_entry.ptr < -1) {
		bool isExcess = false;
		//search excess list only if there is no room in ordered part
		if (hash_entry.ptr >= -1) {
			while (hash_entry.offset >= 1) {
				hash_code = ORDERED_LIST_SIZE + hash_entry.offset - 1;
				hash_entry = hash_table[hash_code];
				if (IS_EQUAL3(hash_entry.pos, hash_entry_position) && hash_entry.ptr >= -1) {
					result_entry = &hash_table[hash_code];
					return true;
				}
			}
			isExcess = true;

		}
		//still not found, allocate
		if (isExcess && last_free_voxel_block_id >= 0 && last_free_excess_list_id >= 0) {
			//there is room in the voxel block array and excess list
			HashEntry newHashEntry;
			newHashEntry.pos = hash_entry_position;
			newHashEntry.ptr = voxel_allocation_list[last_free_voxel_block_id];
			newHashEntry.offset = 0;
			int excess_list_offset = excess_allocation_list[last_free_excess_list_id];
			hash_table[hash_code].offset = excess_list_offset + 1; //connect to child
			hash_code = ORDERED_LIST_SIZE + excess_list_offset;
			hash_table[hash_code] = newHashEntry; //add child to the excess list
			result_entry = &hash_table[hash_code];
			last_free_voxel_block_id--;
			last_free_excess_list_id--;
			return true;
		} else if (last_free_voxel_block_id >= 0) {
			//there is room in the voxel block array
			HashEntry new_hash_entry;
			new_hash_entry.pos = hash_entry_position;
			new_hash_entry.ptr = voxel_allocation_list[last_free_voxel_block_id];
			new_hash_entry.offset = 0;
			hash_table[hash_code] = new_hash_entry;
			result_entry = &hash_table[hash_code];
			last_free_voxel_block_id--;
			return true;
		} else {
			return false;
		}
	} else {
		//HashEntry already exists, return the pointer to it
		result_entry = &hash_table[hash_code];
		return true;
	}
}

// region ================================= OFFSET COPYING =============================================================

struct OffsetCopyHashBlockInfo {
	int target_bash;
	bool fully_in_bounds;
};

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

//endregion