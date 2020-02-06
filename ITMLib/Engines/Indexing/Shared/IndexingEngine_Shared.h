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
#include "../../../Objects/Views/ITMView.h"
#include "../../../Objects/Tracking/CameraTrackingState.h"
#include "../../Common/CommonFunctors.h"
#include "../../Common/AllocationTempData.h"
#include "../../../Utils/Math.h"
#include "../../../Utils/PixelUtils.h"
#include "../../../Utils/VoxelFlags.h"
#include "../../../Utils/HashBlockProperties.h"
#include "../../../Utils/Geometry/IntersectionChecks.h"
#include "../AtomicArrayThreadGuard/AtomicArrayThreadGuard.h"


#ifdef __CUDACC__
#include "../../../Utils/CUDAUtils.h"
#endif

using namespace ITMLib;


template<MemoryDeviceType TMemoryDeviceType>
_DEVICE_WHEN_AVAILABLE_
inline int AllocateBlock(const CONSTPTR(Vector3s)& desired_block_position,
                         HashEntry* hash_table,
                         AtomicArrayThreadGuard<TMemoryDeviceType>& guard,
                         ATOMIC_ARGUMENT(int) allocated_blocks,
                         ATOMIC_ARGUMENT(int) allocated_excess_entries,
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
			int block_index = ATOMIC_SUB(allocated_blocks, 1);
			int excess_list_index = ATOMIC_SUB(allocated_excess_entries, 1);
			if (block_index >= 0 && excess_list_index >= 0) {
				int excess_list_offset = excess_allocation_list[excess_list_index];
				hash_table[hash_code].offset = excess_list_offset + 1;
				HashEntry& new_hash_entry = hash_table[ORDERED_LIST_SIZE + excess_list_offset];
				new_hash_entry.pos = desired_block_position;
				new_hash_entry.ptr = block_allocation_list[block_index];
				new_hash_entry.offset = 0;
			} else {
				ATOMIC_ADD(allocated_blocks, 1);
				ATOMIC_ADD(allocated_excess_entries, 1);
				guard.release(hash_code);
				return 0;
			}
			guard.release(hash_code);
			return hash_code;
		}
		int ordered_index = ATOMIC_SUB(allocated_blocks, 1);

		if (ordered_index >= 0) {
			HashEntry& new_hash_entry = hash_table[hash_code];
			new_hash_entry.pos = desired_block_position;
			new_hash_entry.ptr = block_allocation_list[ordered_index];
			new_hash_entry.offset = 0;
		} else {
			ATOMIC_ADD(allocated_blocks, 1);
			guard.release(hash_code);
			return 0;
		}
		return hash_code;
	}
	guard.release(hash_code);
	// already have hash block, no allocation needed
	return hash_code;
}

/**
 * \brief Determines whether the hash block at the specified block position needs it's voxels to be allocated, as well
 * as whether they should be allocated in the excess list or the ordered list of the hash table.
 * If any of these are true, marks the corresponding entry in \param hashEntryStates
 * \param[in,out] hashEntryStates  array where to set the allocation type at final hashIdx index
 * \param[in,out] hash_block_coordinates  array block coordinates for the new hash blocks at final hashIdx index
 * \param[in,out] hash_code  takes in original index assuming coords, i.e. \refitem HashCodeFromBlockPosition(\param desired_hash_block_position),
 * returns final index of the hash block to be allocated (may be updated based on hash closed chaining)
 * \param[in] desired_hash_block_position  position of the hash block to check / allocate
 * \param[in] hash_table  hash table with existing blocks
 * \param[in] collisionDetected set to true if a block with the same hashcode has already been marked for allocation ( a collision occured )
 * \return true if the block needs allocation, false otherwise
 */
_CPU_AND_GPU_CODE_
inline bool MarkAsNeedingAllocationIfNotFound(ITMLib::HashEntryAllocationState* hashEntryStates,
                                              Vector3s* hash_block_coordinates, int& hash_code,
                                              const CONSTPTR(Vector3s)& desired_hash_block_position,
                                              const CONSTPTR(HashEntry)* hash_table, bool& collisionDetected) {

	HashEntry hash_entry = hash_table[hash_code];
	//check if hash table contains entry

	if (!(IS_EQUAL3(hash_entry.pos, desired_hash_block_position) && hash_entry.ptr >= -1)) {

		auto setHashEntryState = [&](HashEntryAllocationState state) {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
			if (atomicCAS((char*) hashEntryStates + hash_code,
					  (char) ITMLib::NEEDS_NO_CHANGE,
					  (char) state) != ITMLib::NEEDS_NO_CHANGE){
			if (IS_EQUAL3(hash_block_coordinates[hash_code], desired_hash_block_position)) return false;
			//hash code already marked for allocation, but at different coordinates, cannot allocate
			collisionDetected = true;
			return false;
		} else {
			hash_block_coordinates[hash_code] = desired_hash_block_position;
			return true;
		}
#else
			//TODO: come up with an atomics-based solution for OpenMP
			bool success = false;
#ifdef WITH_OPENMP
#pragma omp critical
#endif
			{
				//single-threaded version
				if (hashEntryStates[hash_code] != ITMLib::NEEDS_NO_CHANGE) {
					if (!IS_EQUAL3(hash_block_coordinates[hash_code], desired_hash_block_position)) {
						//hash code already marked for allocation, but at different coordinates, cannot allocate
						collisionDetected = true;
					}
					success = false;
				} else {
					hashEntryStates[hash_code] = state;
					hash_block_coordinates[hash_code] = desired_hash_block_position;
					success = true;
				}
			}
			return success;
#endif
		};
		if (hash_entry.ptr >= -1) {
			//search excess list only if there is no room in ordered part
			while (hash_entry.offset >= 1) {
				hash_code = ORDERED_LIST_SIZE + hash_entry.offset - 1;
				hash_entry = hash_table[hash_code];

				if (IS_EQUAL3(hash_entry.pos, desired_hash_block_position) && hash_entry.ptr >= -1) {
					return false;
				}
			}
			return setHashEntryState(ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST);
		}
		return setHashEntryState(ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST);
	}
	// already have hash block, no allocation needed
	return false;

};


_CPU_AND_GPU_CODE_ inline void
MarkForAllocationAndSetVisibilityTypeIfNotFound(ITMLib::HashEntryAllocationState* hashEntryStates,
                                                Vector3s* hashBlockCoordinates,
                                                HashBlockVisibility* blockVisibilityTypes,
                                                Vector3s desiredHashBlockPosition,
                                                const CONSTPTR(HashEntry)* hashTable, bool& collisionDetected) {

	int hashCode = HashCodeFromBlockPosition(desiredHashBlockPosition);

	HashEntry hashEntry = hashTable[hashCode];

	//check if hash table contains entry
	if (IS_EQUAL3(hashEntry.pos, desiredHashBlockPosition) && hashEntry.ptr >= -1) {
		//entry has been streamed out but is visible or in memory and visible
		blockVisibilityTypes[hashCode] = (hashEntry.ptr == -1) ? STREAMED_OUT_AND_VISIBLE
		                                                       : IN_MEMORY_AND_VISIBLE;
		return;
	}

	HashEntryAllocationState allocationState = NEEDS_ALLOCATION_IN_ORDERED_LIST;
	if (hashEntry.ptr >= -1) //search excess list only if there is no room in ordered part
	{
		while (hashEntry.offset >= 1) {
			hashCode = ORDERED_LIST_SIZE + hashEntry.offset - 1;
			hashEntry = hashTable[hashCode];

			if (IS_EQUAL3(hashEntry.pos, desiredHashBlockPosition) && hashEntry.ptr >= -1) {
				//entry has been streamed out but is visible or in memory and visible
				blockVisibilityTypes[hashCode] = (hashEntry.ptr == -1) ? STREAMED_OUT_AND_VISIBLE
				                                                       : IN_MEMORY_AND_VISIBLE;
				return;
			}
		}
		allocationState = NEEDS_ALLOCATION_IN_EXCESS_LIST;
	}

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
	if (atomicCAS((char*) hashEntryStates + hashCode,
				  (char) ITMLib::NEEDS_NO_CHANGE,
				  (char) allocationState) != ITMLib::NEEDS_NO_CHANGE) {
		if (IS_EQUAL3(hashBlockCoordinates[hashCode], desiredHashBlockPosition)) return;
		collisionDetected = true;
	} else {
		//needs allocation
		if (allocationState == NEEDS_ALLOCATION_IN_ORDERED_LIST)
			blockVisibilityTypes[hashCode] = HashBlockVisibility::IN_MEMORY_AND_VISIBLE; //new entry is visible
		hashBlockCoordinates[hashCode] = desiredHashBlockPosition;
	}
#else
#if defined(WITH_OPENMP)
#pragma omp critical
#endif
	{
		if (hashEntryStates[hashCode] != ITMLib::NEEDS_NO_CHANGE) {
			collisionDetected = true;
		} else {
			//needs allocation
			hashEntryStates[hashCode] = allocationState;
			if (allocationState == NEEDS_ALLOCATION_IN_ORDERED_LIST)
				blockVisibilityTypes[hashCode] = HashBlockVisibility::IN_MEMORY_AND_VISIBLE; //new entry is visible
			hashBlockCoordinates[hashCode] = desiredHashBlockPosition;
		}
	}
#endif
}


_CPU_AND_GPU_CODE_ inline int getIncrementCount(Vector3s coord1, Vector3s coord2) {
	return static_cast<int>(coord1.x != coord2.x) +
	       static_cast<int>(coord1.y != coord2.y) +
	       static_cast<int>(coord1.z != coord2.z);
}

_CPU_AND_GPU_CODE_ inline void
findVoxelHashBlocksAlongSegment(ITMLib::HashEntryAllocationState* hash_entry_allocation_states,
                                Vector3s* hash_block_coordinates,
                                HashBlockVisibility* hash_block_visibility_types,
                                const CONSTPTR(HashEntry)* hash_table,
                                const ITMLib::Segment& segment_in_hash_blocks,
                                bool& collision_detected) {

	// number of steps to take along the truncated SDF band
	int step_count = (int) std::ceil(2.0f * segment_in_hash_blocks.length());

	// a single stride along the sdf band segment from one step to the next
	Vector3f strideVector = segment_in_hash_blocks.direction / (float) (step_count - 1);

	Vector3s previousHashBlockPosition;
	Vector3f check_position = segment_in_hash_blocks.origin;

	//add neighbouring blocks
	for (int i = 0; i < step_count; i++) {
		//find block position at current step
		Vector3s currentHashBlockPosition = TO_SHORT_FLOOR3(check_position);
		int incrementCount;
		if (i > 0 && (incrementCount = getIncrementCount(currentHashBlockPosition, previousHashBlockPosition)) > 1) {
			if (incrementCount == 2) {
				for (int iDirection = 0; iDirection < 3; iDirection++) {
					if (currentHashBlockPosition.values[iDirection] != previousHashBlockPosition.values[iDirection]) {
						Vector3s potentiallyMissedBlockPosition = previousHashBlockPosition;
						potentiallyMissedBlockPosition.values[iDirection] = currentHashBlockPosition.values[iDirection];
						if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
						                                       TO_FLOAT3(potentiallyMissedBlockPosition),
						                                       1.0f)) {
							MarkForAllocationAndSetVisibilityTypeIfNotFound(
									hash_entry_allocation_states,
									hash_block_coordinates, hash_block_visibility_types, potentiallyMissedBlockPosition,
									hash_table,
									collision_detected);
						}
					}
				}
			} else {
				//incrementCount == 3
				for (int iDirection = 0; iDirection < 3; iDirection++) {
					Vector3s potentiallyMissedBlockPosition = previousHashBlockPosition;
					potentiallyMissedBlockPosition.values[iDirection] = currentHashBlockPosition.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
					                                       TO_FLOAT3(potentiallyMissedBlockPosition),
					                                       1.0f)) {
						MarkForAllocationAndSetVisibilityTypeIfNotFound(
								hash_entry_allocation_states,
								hash_block_coordinates, hash_block_visibility_types, potentiallyMissedBlockPosition,
								hash_table,
								collision_detected);
					}
					potentiallyMissedBlockPosition = currentHashBlockPosition;
					potentiallyMissedBlockPosition.values[iDirection] = previousHashBlockPosition.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
					                                       TO_FLOAT3(potentiallyMissedBlockPosition),
					                                       1.0f)) {
						MarkForAllocationAndSetVisibilityTypeIfNotFound(
								hash_entry_allocation_states,
								hash_block_coordinates, hash_block_visibility_types, potentiallyMissedBlockPosition,
								hash_table,
								collision_detected);
					}
				}
			}
		}
		MarkForAllocationAndSetVisibilityTypeIfNotFound(hash_entry_allocation_states,
		                                                hash_block_coordinates,
		                                                hash_block_visibility_types, currentHashBlockPosition,
		                                                hash_table,
		                                                collision_detected);

		check_position += strideVector;
		previousHashBlockPosition = currentHashBlockPosition;
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

/**
 * \brief compute the segment formed by tracing the camera-space point a fixed distance forward and backward along
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
	float norm = ORUtils::length(point_in_camera_space);

	Vector4f endpoint_in_camera_space;

	endpoint_in_camera_space = point_in_camera_space * (1.0f - distance_from_point / norm);
	endpoint_in_camera_space.w = 1.0f;
	//start position along ray in hash blocks
	Vector3f start_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpace(
			endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);

	endpoint_in_camera_space = point_in_camera_space * (1.0f + distance_from_point / norm);
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
		norm = ORUtils::length(point1_in_camera_space);
		endpoint_in_camera_space = point1_in_camera_space * (1.0f - range / norm);
		endpoint_in_camera_space.w = 1.0f;
		Vector3f start_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpace(
				endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);
		norm = ORUtils::length(point2_in_camera_space);
		endpoint_in_camera_space = point2_in_camera_space * (1.0f + range / norm);
		Vector3f end_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpace(
				endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);
		return ITMLib::Segment(start_point_in_hash_blocks, end_point_in_hash_blocks);
	} else {
		norm = ORUtils::length(point2_in_camera_space);
		endpoint_in_camera_space = point2_in_camera_space * (1.0f - range / norm);
		endpoint_in_camera_space.w = 1.0f;
		Vector3f start_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpace(
				endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);
		norm = ORUtils::length(point1_in_camera_space);
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
inline ITMLib::Segment
findHashBlockSegmentAlongCameraRayWithinRangeFromDepth(const float distance_from_point, const float depth_measure,
                                                       const int x, const int y, const Matrix4f& inverted_camera_pose,
                                                       const Vector4f& inverted_camera_projection_parameters,
                                                       const float one_over_hash_block_size) {

	Vector4f point_in_camera_space = imageSpacePointToCameraSpace(depth_measure, x, y,
	                                                              inverted_camera_projection_parameters);
	return findHashBlockSegmentAlongCameraRayWithinRangeFromPoint(distance_from_point, point_in_camera_space,
	                                                              inverted_camera_pose, one_over_hash_block_size);
}


_CPU_AND_GPU_CODE_ inline void
findVoxelBlocksForRayNearSurface(ITMLib::HashEntryAllocationState* hash_entry_allocation_states,
                                 Vector3s* hash_block_coordinates,
                                 HashBlockVisibility* hash_block_visibility_types,
                                 const CONSTPTR(HashEntry)* hash_table,
                                 const int x, const int y, const CONSTPTR(float)* depth, float surface_distance_cutoff,
                                 const Matrix4f inverted_camera_pose,
                                 const Vector4f inverted_projection_parameters,
                                 const float one_over_hash_block_size, const Vector2i depth_image_size,
                                 const float near_clipping_distance, const float far_clipping_distance,
                                 bool& collision_detected) {

	float depth_measure = depth[x + y * depth_image_size.x];
	if (depth_measure <= 0 || (depth_measure - surface_distance_cutoff) < 0 ||
	    (depth_measure - surface_distance_cutoff) < near_clipping_distance ||
	    (depth_measure + surface_distance_cutoff) > far_clipping_distance)
		return;

	// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
	// end of the (truncated SDF) band (increased by backBandFactor), along the ray cast from the camera through the
	// point, in camera space
	ITMLib::Segment march_segment = findHashBlockSegmentAlongCameraRayWithinRangeFromDepth(surface_distance_cutoff,
	                                                                                       depth_measure,
	                                                                                       x, y, inverted_camera_pose,
	                                                                                       inverted_projection_parameters,
	                                                                                       one_over_hash_block_size);


	findVoxelHashBlocksAlongSegment(hash_entry_allocation_states, hash_block_coordinates, hash_block_visibility_types,
	                                hash_table, march_segment, collision_detected);
}


_CPU_AND_GPU_CODE_
inline
bool FindOrAllocateHashEntry(const Vector3s& hashEntryPosition, HashEntry* hashTable, HashEntry*& resultEntry,
                             int& lastFreeVoxelBlockId, int& lastFreeExcessListId, const int* voxelAllocationList,
                             const int* excessAllocationList, int& hashCode) {
	hashCode = HashCodeFromBlockPosition(hashEntryPosition);
	HashEntry hashEntry = hashTable[hashCode];
	if (!IS_EQUAL3(hashEntry.pos, hashEntryPosition) || hashEntry.ptr < -1) {
		bool isExcess = false;
		//search excess list only if there is no room in ordered part
		if (hashEntry.ptr >= -1) {
			while (hashEntry.offset >= 1) {
				hashCode = ORDERED_LIST_SIZE + hashEntry.offset - 1;
				hashEntry = hashTable[hashCode];
				if (IS_EQUAL3(hashEntry.pos, hashEntryPosition) && hashEntry.ptr >= -1) {
					resultEntry = &hashTable[hashCode];
					return true;
				}
			}
			isExcess = true;

		}
		//still not found, allocate
		if (isExcess && lastFreeVoxelBlockId >= 0 && lastFreeExcessListId >= 0) {
			//there is room in the voxel block array and excess list
			HashEntry newHashEntry;
			newHashEntry.pos = hashEntryPosition;
			newHashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
			newHashEntry.offset = 0;
			int exlOffset = excessAllocationList[lastFreeExcessListId];
			hashTable[hashCode].offset = exlOffset + 1; //connect to child
			hashCode = ORDERED_LIST_SIZE + exlOffset;
			hashTable[hashCode] = newHashEntry; //add child to the excess list
			resultEntry = &hashTable[hashCode];
			lastFreeVoxelBlockId--;
			lastFreeExcessListId--;
			return true;
		} else if (lastFreeVoxelBlockId >= 0) {
			//there is room in the voxel block array
			HashEntry newHashEntry;
			newHashEntry.pos = hashEntryPosition;
			newHashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
			newHashEntry.offset = 0;
			hashTable[hashCode] = newHashEntry;
			resultEntry = &hashTable[hashCode];
			lastFreeVoxelBlockId--;
			return true;
		} else {
			return false;
		}
	} else {
		//HashEntry already exists, return the pointer to it
		resultEntry = &hashTable[hashCode];
		return true;
	}
}
