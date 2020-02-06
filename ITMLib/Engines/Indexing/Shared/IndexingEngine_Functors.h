//  ================================================================
//  Created by Gregory Kramida on 2/5/20.
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

//local
#include "IndexingEngine_Shared.h"
#include "../../Common/CheckBlockVisibility.h"





template<typename TWarp, typename TVoxel, WarpType TWarpType>
struct WarpBasedAllocationMarkerFunctor {
	WarpBasedAllocationMarkerFunctor(
			VoxelVolume<TVoxel, VoxelBlockHash>* sourceVolume,
			VoxelVolume<TVoxel, VoxelBlockHash>* volumeToAllocate,
			Vector3s* allocationBlockCoords,
			HashEntryAllocationState* warpedEntryAllocationStates) :

			collisionDetected(false),

			targetTSDFScene(volumeToAllocate),
			targetTSDFVoxels(volumeToAllocate->localVBA.GetVoxelBlocks()),
			targetTSDFHashEntries(volumeToAllocate->index.GetEntries()),
			targetTSDFCache(),

			sourceTSDFScene(sourceVolume),
			sourceTSDFVoxels(sourceVolume->localVBA.GetVoxelBlocks()),
			sourceTSDFHashEntries(sourceVolume->index.GetEntries()),
			sourceTSDFCache(),

			allocationBlockCoords(allocationBlockCoords),
			warpedEntryAllocationStates(warpedEntryAllocationStates) {}

	_CPU_AND_GPU_CODE_
	inline
	void operator()(TWarp& warpVoxel, Vector3i voxelPosition, Vector3s hashBlockPosition) {

		Vector3f warpVector = ITMLib::WarpVoxelStaticFunctor<TWarp, TWarpType>::GetWarp(warpVoxel);
		Vector3f warpedPosition = warpVector + TO_FLOAT3(voxelPosition);
		Vector3i warpedPositionTruncated = warpedPosition.toInt();

		// perform lookup in source volume
		int vmIndex;
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		const TVoxel& sourceTSDFVoxelAtWarp = readVoxel(sourceTSDFVoxels, sourceTSDFHashEntries,
		                                                warpedPositionTruncated,
		                                                vmIndex, sourceTSDFCache);
#else //don't use cache when multithreading!
		const TVoxel& sourceTSDFVoxelAtWarp = readVoxel(sourceTSDFVoxels, sourceTSDFHashEntries,
														warpedPositionTruncated,
														vmIndex);
#endif

		int targetBlockHash = HashCodeFromBlockPosition(hashBlockPosition);

		MarkAsNeedingAllocationIfNotFound(warpedEntryAllocationStates, allocationBlockCoords, targetBlockHash,
		                                  hashBlockPosition, targetTSDFHashEntries, collisionDetected);
	}

	bool collisionDetected;

private:


	VoxelVolume<TVoxel, VoxelBlockHash>* targetTSDFScene;
	TVoxel* targetTSDFVoxels;
	HashEntry* targetTSDFHashEntries;
	VoxelBlockHash::IndexCache targetTSDFCache;

	VoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDFScene;
	TVoxel* sourceTSDFVoxels;
	HashEntry* sourceTSDFHashEntries;
	VoxelBlockHash::IndexCache sourceTSDFCache;

	Vector3s* allocationBlockCoords;
	HashEntryAllocationState* warpedEntryAllocationStates;
};


template<MemoryDeviceType TMemoryDeviceType>
struct DepthBasedAllocationFunctor {
public:
	DepthBasedAllocationFunctor(VoxelBlockHash& index,
	                            const VoxelVolumeParameters* volume_parameters, const ITMLib::ITMView* view,
	                            Matrix4f depth_camera_pose, float surface_distance_cutoff) :

			near_clipping_distance(volume_parameters->near_clipping_distance),
			far_clipping_distance(volume_parameters->far_clipping_distance),
			inverted_projection_parameters(view->calib.intrinsics_d.projectionParamsSimple.all),

			surface_distance_cutoff(surface_distance_cutoff),

			hash_block_size_reciprocal(1.0f / (volume_parameters->voxel_size * VOXEL_BLOCK_SIZE)),
			hash_entry_allocation_states(index.GetHashEntryAllocationStates()),
			hash_block_coordinates(index.GetAllocationBlockCoordinates()),
			hash_block_visibility_types(index.GetBlockVisibilityTypes()),
			hash_table(index.GetEntries()),

			collision_detected(false) {
		depth_camera_pose.inv(inverted_camera_pose);
		inverted_projection_parameters.fx = 1.0f / inverted_projection_parameters.fx;
		inverted_projection_parameters.fy = 1.0f / inverted_projection_parameters.fy;
	}

	_CPU_AND_GPU_CODE_
	void operator()(const float& depth_measure, int x, int y) {
		if (depth_measure <= 0 || (depth_measure - surface_distance_cutoff) < 0 ||
		    (depth_measure - surface_distance_cutoff) < near_clipping_distance ||
		    (depth_measure + surface_distance_cutoff) > far_clipping_distance)
			return;

		// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
		// end of the (truncated SDF) band (increased by backBandFactor), along the ray cast from the camera through the
		// point, in camera space
		ITMLib::Segment march_segment = findHashBlockSegmentAlongCameraRayWithinRangeFromDepth(surface_distance_cutoff,
		                                                                                       depth_measure,
		                                                                                       x, y,
		                                                                                       inverted_camera_pose,
		                                                                                       inverted_projection_parameters,
		                                                                                       hash_block_size_reciprocal);


		findVoxelHashBlocksAlongSegment(hash_entry_allocation_states, hash_block_coordinates,
		                                hash_block_visibility_types,
		                                hash_table, march_segment, collision_detected);
	}

	bool collision_detected;

protected:

	float near_clipping_distance;
	float far_clipping_distance;
	Matrix4f inverted_camera_pose;
	Vector4f inverted_projection_parameters;

	float surface_distance_cutoff;

	float hash_block_size_reciprocal;
	HashEntryAllocationState* hash_entry_allocation_states;
	Vector3s* hash_block_coordinates;
	HashBlockVisibility* hash_block_visibility_types;
	HashEntry* hash_table;
};

template<MemoryDeviceType TMemoryDeviceType>
struct TwoSurfaceBasedAllocationFunctor
		: public DepthBasedAllocationFunctor<TMemoryDeviceType> {
public:
	TwoSurfaceBasedAllocationFunctor(VoxelBlockHash& index,
	                                 const VoxelVolumeParameters* volume_parameters, const ITMLib::ITMView* view,
	                                 const CameraTrackingState* tracking_state, float surface_distance_cutoff) :
			DepthBasedAllocationFunctor<TMemoryDeviceType>(index, volume_parameters, view, tracking_state->pose_d->GetM(), surface_distance_cutoff) {}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const float& surface1_depth, const Vector4f& surface2_point, int x, int y) {
		bool has_surface1 = false, has_surface2 = false;

		if (!(surface1_depth <= 0 || (surface1_depth - surface_distance_cutoff) < 0 ||
		      (surface1_depth - surface_distance_cutoff) < near_clipping_distance ||
		      (surface1_depth + surface_distance_cutoff) > far_clipping_distance))
			has_surface1 = true;

		if (surface2_point.x > 0.0f) has_surface2 = true;

		ITMLib::Segment march_segment;

		if (has_surface1 && has_surface2) {
			Vector4f surface1_point = imageSpacePointToCameraSpace(surface1_depth, x, y,
			                                                       inverted_projection_parameters);
			march_segment = findHashBlockSegmentAlongCameraRayWithinRangeFromAndBetweenTwoPoints(
					surface_distance_cutoff, surface1_point, surface2_point, inverted_camera_pose,
					hash_block_size_reciprocal);
		} else {
			if (has_surface1) {
				march_segment = findHashBlockSegmentAlongCameraRayWithinRangeFromDepth(
						surface_distance_cutoff, surface1_depth, x, y, inverted_camera_pose,
						inverted_projection_parameters, hash_block_size_reciprocal);
			} else if (has_surface2) {
				march_segment = findHashBlockSegmentAlongCameraRayWithinRangeFromPoint(
						surface_distance_cutoff, surface2_point, inverted_camera_pose, hash_block_size_reciprocal);
			} else {
				return; // neither surface is defined at this point, nothing to do.
			}
		}

		findVoxelHashBlocksAlongSegment(hash_entry_allocation_states, hash_block_coordinates, hash_block_visibility_types,
		                                hash_table, march_segment, collision_detected);
	}
	using DepthBasedAllocationFunctor<TMemoryDeviceType>::collision_detected;
protected:
	using DepthBasedAllocationFunctor<TMemoryDeviceType>::near_clipping_distance;
	using DepthBasedAllocationFunctor<TMemoryDeviceType>::far_clipping_distance;
	using DepthBasedAllocationFunctor<TMemoryDeviceType>::inverted_camera_pose;
	using DepthBasedAllocationFunctor<TMemoryDeviceType>::inverted_projection_parameters;

	using DepthBasedAllocationFunctor<TMemoryDeviceType>::surface_distance_cutoff;

	using DepthBasedAllocationFunctor<TMemoryDeviceType>::hash_block_size_reciprocal;
	using DepthBasedAllocationFunctor<TMemoryDeviceType>::hash_entry_allocation_states;
	using DepthBasedAllocationFunctor<TMemoryDeviceType>::hash_block_coordinates;
	using DepthBasedAllocationFunctor<TMemoryDeviceType>::hash_block_visibility_types;
	using DepthBasedAllocationFunctor<TMemoryDeviceType>::hash_table;
};

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct ReallocateDeletedHashBlocksFunctor{
	ReallocateDeletedHashBlocksFunctor(VoxelVolume<TVoxel, VoxelBlockHash>* volume) :
			hash_block_visibility_types(volume->index.GetBlockVisibilityTypes()),
			block_allocation_list(volume->localVBA.GetAllocationList())
	{
		INITIALIZE_ATOMIC(int, last_free_voxel_block_id, volume->localVBA.lastFreeBlockId);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(HashEntry& entry, int hash_code){
		if (hash_block_visibility_types[hash_code] > 0 && entry.ptr == -1) {
			int current_last_id = ATOMIC_ADD(last_free_voxel_block_id, -1);
			if (current_last_id >= 0) {
				entry.ptr = block_allocation_list[current_last_id];
			}else{
				ATOMIC_ADD(last_free_voxel_block_id, 1);
			}
		}
	}
	DECLARE_ATOMIC(int, last_free_voxel_block_id);
private:
	HashBlockVisibility* hash_block_visibility_types;
	int* block_allocation_list;
};