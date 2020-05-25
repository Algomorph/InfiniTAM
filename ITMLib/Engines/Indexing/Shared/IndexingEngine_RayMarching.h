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

//_DEBUG allocation
#include <unordered_set>

#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../../ORUtils/CrossPlatformMacros.h"
#include "../../../Utils/Math.h"
#include "../../../Utils/HashBlockProperties.h"
#include "IndexingEngine_ConcurrentBlockManagement.h"
#include "../../../Utils/Geometry/IntersectionChecks.h"


namespace ITMLib {


/**Get number of differing vector components**/
_CPU_AND_GPU_CODE_ inline int GetDifferingComponentCount(Vector3s coord1, Vector3s coord2) {
	return static_cast<int>(coord1.x != coord2.x) +
	       static_cast<int>(coord1.y != coord2.y) +
	       static_cast<int>(coord1.z != coord2.z);
}

_DEVICE_WHEN_AVAILABLE_ inline void
TryToMarkBlockForAllocation(const Vector3s& block_position,
                            ITMLib::HashEntryAllocationState* hash_entry_allocation_states,
                            Vector3s* hash_block_coordinates,
                            bool& unresolvable_collision_encountered,
                            const CONSTPTR(HashEntry)* hash_table,
                            THREADPTR(Vector3s)* colliding_block_positions,
                            ATOMIC_ARGUMENT(int) colliding_block_count) {
	ThreadAllocationStatus resulting_status = MarkAsNeedingAllocationIfNotFound<true>(
			hash_entry_allocation_states,
			hash_block_coordinates, block_position,
			hash_table, colliding_block_positions, colliding_block_count);
//_DEBUG allocation
//#ifndef __CUDACC__
//	std::unordered_set<Vector3s> debug_locations =
//	{
////	Vector3s(0,6,38), Vector3s(1,6,38)
////	Vector3s(3,-1,73), Vector3s(10,-4,31), Vector3s(-3,3,24)
////		Vector3s(10, -4, 31)
//		Vector3s(-4, 3, 21)
//	};
//#endif

	if (resulting_status == BEING_MODIFIED_BY_ANOTHER_THREAD) {
		unresolvable_collision_encountered = true;
	}

//_DEBUG allocation
//#ifndef __CUDACC__
//	else if (debug_locations.find(block_position) != debug_locations.end() &&
//	         (resulting_status == MARK_FOR_ALLOCATION_IN_EXCESS_LIST ||
//	          resulting_status == MARK_FOR_ALLOCATION_IN_ORDERED_LIST)) {
//		int i = 0;
//	}
//#endif
}

_DEVICE_WHEN_AVAILABLE_ inline void
MarkVoxelHashBlocksAlongSegment(ITMLib::HashEntryAllocationState* hash_entry_allocation_states,
                                Vector3s* hash_block_coordinates,
                                bool& unresolvable_collision_encountered,
                                const CONSTPTR(HashEntry)* hash_table,
                                const ITMLib::Segment& segment_in_hash_blocks,
                                THREADPTR(Vector3s)* colliding_block_positions,
                                ATOMIC_ARGUMENT(int) colliding_block_count) {

// number of steps to take along the truncated SDF band
	int step_count = (int) std::ceil(2.0f * segment_in_hash_blocks.length());

// a single stride along the sdf band segment from one step to the next
	Vector3f strideVector = segment_in_hash_blocks.vector_to_destination / (float) (step_count - 1);

	Vector3s previous_block_position;
	Vector3f check_position = segment_in_hash_blocks.origin;

//add neighbouring blocks
	for (int i = 0; i < step_count; i++) {
//find block position at current step
		Vector3s current_block_position = TO_SHORT_FLOOR3(check_position);
		int directional_increment_count;
		if (i > 0 && (directional_increment_count = GetDifferingComponentCount(current_block_position,
		                                                                       previous_block_position)) > 1) {
			if (directional_increment_count == 2) {
				for (int i_direction = 0; i_direction < 3; i_direction++) {
					if (current_block_position.values[i_direction] != previous_block_position.values[i_direction]) {
						Vector3s potentially_missed_block_position = previous_block_position;
						potentially_missed_block_position.values[i_direction] = current_block_position.values[i_direction];
						if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
						                                       TO_FLOAT3(potentially_missed_block_position),
						                                       1.0f)) {
							TryToMarkBlockForAllocation(
									potentially_missed_block_position,
									hash_entry_allocation_states,
									hash_block_coordinates,
									unresolvable_collision_encountered,
									hash_table,
									colliding_block_positions,
									colliding_block_count
							);
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
						TryToMarkBlockForAllocation(
								potentially_missed_block_position,
								hash_entry_allocation_states,
								hash_block_coordinates,
								unresolvable_collision_encountered,
								hash_table,
								colliding_block_positions,
								colliding_block_count
						);
					}
					potentially_missed_block_position = current_block_position;
					potentially_missed_block_position.values[iDirection] = previous_block_position.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
					                                       TO_FLOAT3(potentially_missed_block_position),
					                                       1.0f)) {
						TryToMarkBlockForAllocation(
								potentially_missed_block_position,
								hash_entry_allocation_states,
								hash_block_coordinates,
								unresolvable_collision_encountered,
								hash_table,
								colliding_block_positions,
								colliding_block_count
						);
					}
				}
			}
		}
		TryToMarkBlockForAllocation(
				current_block_position,
				hash_entry_allocation_states,
				hash_block_coordinates,
				unresolvable_collision_encountered,
				hash_table,
				colliding_block_positions,
				colliding_block_count
		);
		check_position += strideVector;
		previous_block_position = current_block_position;
	}
}

_CPU_AND_GPU_CODE_
inline Vector4f ImageSpacePointToCameraSpace(const float depth, const float x, const float y,
                                             const Vector4f& inverted_camera_projection_parameters) {
	return {depth *
	        ((x - inverted_camera_projection_parameters.cx) * inverted_camera_projection_parameters.fx),
	        depth *
	        ((y - inverted_camera_projection_parameters.cy) * inverted_camera_projection_parameters.fy),
	        depth, 1.0f};
}


_CPU_AND_GPU_CODE_
inline Vector4f ImageSpacePointToCameraSpace(const float depth, const int x, const int y,
                                             const Vector4f& inverted_camera_projection_parameters) {
	return {depth *
	        ((float(x) - inverted_camera_projection_parameters.cx) * inverted_camera_projection_parameters.fx),
	        depth *
	        ((float(y) - inverted_camera_projection_parameters.cy) * inverted_camera_projection_parameters.fy),
	        depth, 1.0f};
}

_CPU_AND_GPU_CODE_
inline Vector4f WorldSpacePointToCameraSpace(const Vector4f& point_world_space_metric,
                                             const Matrix4f& camera_pose) {
	return camera_pose * point_world_space_metric;
}

_CPU_AND_GPU_CODE_
inline Vector4f CameraSpacePointToWorldSpace(const Vector4f& point_camera_space_metric,
                                             const Matrix4f& inverted_camera_pose) {
	return inverted_camera_pose * point_camera_space_metric;
}

_CPU_AND_GPU_CODE_
inline Vector4f ImageSpacePointToWorldSpace(const float depth, const int x, const int y,
                                            const Vector4f& inverted_camera_projection_parameters,
                                            const Matrix4f& inverted_camera_pose) {
	return CameraSpacePointToWorldSpace(
			ImageSpacePointToCameraSpace(depth, x, y, inverted_camera_projection_parameters),
			inverted_camera_pose
	);
}

_CPU_AND_GPU_CODE_
inline Vector3f WorldSpaceToWorldHashBlockSpace(const Vector4f& point_world_space_metric,
                                                const float one_over_hash_block_size) {
	// account for the fact that voxel coordinates represent the voxel center, and we need the extreme corner position of
	// the hash block, i.e. 0.5 voxel (1/16 block) offset from the position along the ray
	return TO_VECTOR3(point_world_space_metric) * one_over_hash_block_size + Vector3f(1.0f / (2.0f * VOXEL_BLOCK_SIZE));
}


_CPU_AND_GPU_CODE_
inline Vector3f CameraSpaceToWorldHashBlockSpace(const Vector4f& point_camera_space,
                                                 const Matrix4f& inverted_camera_pose,
                                                 const float one_over_hash_block_size) {
	// account for the fact that voxel coordinates represent the voxel center, and we need the extreme corner position of
	// the hash block, i.e. 0.5 voxel (1/16 block) offset from the position along the ray
	return WorldSpaceToWorldHashBlockSpace(CameraSpacePointToWorldSpace(point_camera_space, inverted_camera_pose),
	                                       one_over_hash_block_size);
}

_CPU_AND_GPU_CODE_
inline float NormOfFirst3Components(const Vector4f& vec) {
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
inline ITMLib::Segment FindHashBlockSegmentAlongCameraRayWithinRangeFromPoint(
		const float distance_from_point,
		const Vector4f& point_in_camera_space,
		const Matrix4f& inverted_camera_pose,
		const float one_over_hash_block_size) {

	// distance to the point along camera ray
	float norm = NormOfFirst3Components(point_in_camera_space);

	Vector4f endpoint_in_camear_space;

	endpoint_in_camear_space = point_in_camera_space * (1.0f - distance_from_point / norm);
	endpoint_in_camear_space.w = 1.0f;
	//start position along ray in hash blocks
	Vector3f start_point_in_hash_blocks = CameraSpaceToWorldHashBlockSpace(
			endpoint_in_camear_space, inverted_camera_pose, one_over_hash_block_size);

	endpoint_in_camear_space = point_in_camera_space * (1.0f + distance_from_point / norm);
	endpoint_in_camear_space.w = 1.0f;
	//end position of the segment to march along the ray
	Vector3f end_point_in_hash_blocks = CameraSpaceToWorldHashBlockSpace(
			endpoint_in_camear_space, inverted_camera_pose, one_over_hash_block_size);

	// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
	// end of the (truncated SDF) band (increased by backBandFactor), along the ray cast from the camera through the
	// point, in camera space
	ITMLib::Segment segment(start_point_in_hash_blocks, end_point_in_hash_blocks);
	return segment;
}

_CPU_AND_GPU_CODE_
inline ITMLib::Segment FindHashBlockSegmentAlongCameraRayWithinRangeFromAndBetweenTwoPoints(
		const float range,
		const Vector4f& point1_in_camera_space,
		const Vector4f& point2_in_camera_space,
		const Matrix4f& inverted_camera_pose,
		const float one_over_hash_block_size) {

	Vector4f endpoint_in_camera_space;
	float norm;
	if (point2_in_camera_space.z > point1_in_camera_space.z) {
		norm = NormOfFirst3Components(point1_in_camera_space);
		endpoint_in_camera_space = point1_in_camera_space * (1.0f - range / norm);
		endpoint_in_camera_space.w = 1.0f;
		Vector3f start_point_in_hash_blocks = CameraSpaceToWorldHashBlockSpace(
				endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);
		norm = NormOfFirst3Components(point2_in_camera_space);
		endpoint_in_camera_space = point2_in_camera_space * (1.0f + range / norm);
		Vector3f end_point_in_hash_blocks = CameraSpaceToWorldHashBlockSpace(
				endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);
		return ITMLib::Segment(start_point_in_hash_blocks, end_point_in_hash_blocks);
	} else {
		norm = NormOfFirst3Components(point2_in_camera_space);
		endpoint_in_camera_space = point2_in_camera_space * (1.0f - range / norm);
		endpoint_in_camera_space.w = 1.0f;
		Vector3f start_point_in_hash_blocks = CameraSpaceToWorldHashBlockSpace(
				endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);
		norm = NormOfFirst3Components(point1_in_camera_space);
		endpoint_in_camera_space = point1_in_camera_space * (1.0f + range / norm);
		Vector3f end_point_in_hash_blocks = CameraSpaceToWorldHashBlockSpace(
				endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);
		return ITMLib::Segment(start_point_in_hash_blocks, end_point_in_hash_blocks);
	}
}

_CPU_AND_GPU_CODE_
inline ITMLib::Segment
FindHashBlockSegmentAlongCameraRayWithinRangeFromDepth(Vector4f& point_in_camera_space_metric,
                                                       const float distance_from_point, const float depth_measure,
                                                       const int x, const int y,
                                                       const Matrix4f& inverse_camera_pose,
                                                       const Vector4f& inverse_camera_projection_parameters,
                                                       const float one_over_hash_block_size) {

	point_in_camera_space_metric = ImageSpacePointToCameraSpace(depth_measure, x, y,
	                                                            inverse_camera_projection_parameters);
	return FindHashBlockSegmentAlongCameraRayWithinRangeFromPoint(distance_from_point,
	                                                              point_in_camera_space_metric,
	                                                              inverse_camera_pose,
	                                                              one_over_hash_block_size);
}

_CPU_AND_GPU_CODE_
inline ITMLib::Segment
FindHashBlockSegmentAlongCameraRayWithinRangeFromDepth(const float distance_from_point, const float depth_measure,
                                                       const int x, const int y,
                                                       const Matrix4f& inverse_camera_pose,
                                                       const Vector4f& inverse_camera_projection_parameters,
                                                       const float hash_block_size_reciprocal) {

	Vector4f point_in_camera_space_metric;
	return FindHashBlockSegmentAlongCameraRayWithinRangeFromDepth(point_in_camera_space_metric, distance_from_point,
	                                                              depth_measure, x, y, inverse_camera_pose,
	                                                              inverse_camera_projection_parameters,
	                                                              hash_block_size_reciprocal);
}


_DEVICE_WHEN_AVAILABLE_ inline void
FindVoxelBlocksForRayNearSurface(ITMLib::HashEntryAllocationState* hash_entry_allocation_states,
                                 Vector3s* hash_block_coordinates,
                                 bool& unresolvable_collision_encountered,
                                 const CONSTPTR(HashEntry)* hash_table,
                                 const int x, const int y, const CONSTPTR(float)* depth,
                                 float surface_distance_cutoff,
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
	ITMLib::Segment march_segment = FindHashBlockSegmentAlongCameraRayWithinRangeFromDepth(
			surface_distance_cutoff, depth_measure, x, y, inverted_camera_pose,
			inverted_projection_parameters, one_over_hash_block_size);


	MarkVoxelHashBlocksAlongSegment(hash_entry_allocation_states, hash_block_coordinates,
	                                unresolvable_collision_encountered,
	                                hash_table, march_segment, colliding_block_positions, colliding_block_count);
}

} // namespace ITMLib
