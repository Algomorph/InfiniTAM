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
#ifdef __CUDACC__
#include "IndexingEngine_Kernels.h"
#endif


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

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct SpecializedAllocationFunctionsEngine;

template<typename TVoxel>
struct SpecializedAllocationFunctionsEngine<TVoxel, MEMORYDEVICE_CPU>{
	static void SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(VoxelVolume<TVoxel, VoxelBlockHash>* volume){
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
	static void BuildUtilizedBlockListBasedOnVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                                         const Matrix4f& depth_camera_matrix){
// ** scene data **
		const int hash_entry_count = volume->index.hashEntryCount;
		HashBlockVisibility* hash_block_visibility_types = volume->index.GetBlockVisibilityTypes();
		int* visibleEntryHashCodes = volume->index.GetUtilizedBlockHashCodes();
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
				visibleEntryHashCodes[visibleEntryCount] = hash_code;
				visibleEntryCount++;
			}
		}
		volume->index.SetUtilizedHashBlockCount(visibleEntryCount);
	}
};

#define __CUDACC__
template<typename TVoxel>
struct SpecializedAllocationFunctionsEngine<TVoxel, MEMORYDEVICE_CUDA>{
	static void SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(VoxelVolume<TVoxel, VoxelBlockHash>* volume){
		HashBlockVisibility* utilized_block_visibility_types = volume->index.GetBlockVisibilityTypes();
		const int* utilized_block_hash_codes = volume->index.GetUtilizedBlockHashCodes();
		const int utilized_block_count = volume->index.GetUtilizedHashBlockCount();
		dim3 cudaBlockSizeVS(256, 1);
		dim3 gridSizeVS((int) ceil((float) utilized_block_count / (float) cudaBlockSizeVS.x));
		if (gridSizeVS.x > 0) {
			setVisibleEntriesToVisibleAtPreviousFrameAndUnstreamed << < gridSizeVS, cudaBlockSizeVS >> >
			                                                                        (utilized_block_visibility_types,
					                                                                        utilized_block_hash_codes, utilized_block_count);
			ORcudaKernelCheck;
		}
	}
	static void BuildUtilizedBlockListBasedOnVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                                         const Matrix4f& depth_camera_matrix){

		// ** volume data **
		const int hashEntryCount = volume->index.hashEntryCount;
		HashBlockVisibility* hashBlockVisibilityTypes_device = volume->index.GetBlockVisibilityTypes();
		int* visibleBlockHashCodes_device = volume->index.GetUtilizedBlockHashCodes();
		HashEntry* hashTable = volume->index.GetEntries();
		bool useSwapping = volume->globalCache != nullptr;
		ITMHashSwapState* swapStates = volume->Swapping() ? volume->globalCache->GetSwapStates(false) : 0;

		// ** view data **
		Vector4f depthCameraProjectionParameters = view->calib.intrinsics_d.projectionParamsSimple.all;
		Vector2i depthImgSize = view->depth->noDims;
		float voxelSize = volume->sceneParams->voxel_size;


		// ** CUDA data **
		ORUtils::MemoryBlock<int> visibleBlockCount(1, true, true);
		dim3 cudaBlockSizeAL(256, 1);
		dim3 gridSizeAL((int) ceil((float) hashEntryCount / (float) cudaBlockSizeAL.x));

		if (useSwapping) {
			buildVisibilityList_device<true> << < gridSizeAL, cudaBlockSizeAL >> >
			(hashTable, swapStates, hashEntryCount, visibleBlockHashCodes_device,
					visibleBlockCount.GetData(
							MEMORYDEVICE_CUDA), hashBlockVisibilityTypes_device, depth_camera_matrix, depthCameraProjectionParameters, depthImgSize, voxelSize);
			ORcudaKernelCheck;
		} else {
			buildVisibilityList_device<false> << < gridSizeAL, cudaBlockSizeAL >> >
			(hashTable, swapStates, hashEntryCount, visibleBlockHashCodes_device,
					visibleBlockCount.GetData(
							MEMORYDEVICE_CUDA), hashBlockVisibilityTypes_device, depth_camera_matrix, depthCameraProjectionParameters, depthImgSize, voxelSize);
			ORcudaKernelCheck;
		}
		visibleBlockCount.UpdateHostFromDevice();
		volume->index.SetUtilizedHashBlockCount(*visibleBlockCount.GetData(MEMORYDEVICE_CPU));
	}
};
#endif