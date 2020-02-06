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

#include "../../Traversal/Interface/VolumeTraversal.h"
#include "IndexingEngine_VoxelBlockHash.h"
#include "../../Traversal/Interface/ImageTraversal.h"
#include "../../Traversal/Interface/TwoImageTraversal.h"


namespace ITMLib {

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
template<WarpType TWarpType, typename TWarp>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::AllocateFromWarpedVolume(
		VoxelVolume<TWarp, VoxelBlockHash>* warpField,
		VoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDF,
		VoxelVolume<TVoxel, VoxelBlockHash>* targetTSDF) {

	assert(warpField->index.hashEntryCount == sourceTSDF->index.hashEntryCount &&
	       sourceTSDF->index.hashEntryCount == targetTSDF->index.hashEntryCount);

	HashEntryAllocationState* hashEntryStates_device = targetTSDF->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetTSDF->index.GetAllocationBlockCoordinates();

	//Mark up hash entries in the target scene that will need allocation
	WarpBasedAllocationMarkerFunctor<TWarp, TVoxel, TWarpType>
			hashMarkerFunctor(sourceTSDF, targetTSDF, blockCoordinates_device, hashEntryStates_device);

	do {
		//reset allocation flags
		targetTSDF->index.ClearHashEntryAllocationStates();
		hashMarkerFunctor.collisionDetected = false;
		VolumeTraversalEngine<TWarp, VoxelBlockHash, TMemoryDeviceType>::VoxelAndHashBlockPositionTraversal(
				warpField, hashMarkerFunctor);

		//Allocate the hash entries that will potentially have any data
		static_cast<TDerivedClass*>(this)->AllocateHashEntriesUsingLists(targetTSDF);
	} while (hashMarkerFunctor.collisionDetected);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::AllocateFromDepth(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view, const Matrix4f& depth_camera_matrix,
		bool only_update_visible_list, bool resetVisibleList) {

	if (resetVisibleList) volume->index.SetUtilizedHashBlockCount(0);

	float band_factor = configuration::get().general_voxel_volume_parameters.block_allocation_band_factor;
	float surface_distance_cutoff = band_factor * volume->sceneParams->narrow_band_half_width;

	bool use_swapping = volume->globalCache != nullptr;

	DepthBasedAllocationFunctor<TMemoryDeviceType> depth_based_allocator(
			volume->index, volume->sceneParams, view, depth_camera_matrix, surface_distance_cutoff);

	//reset visibility of formerly "visible" entries, if any
	static_cast<TDerivedClass*>(this)->SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(volume);
	do {
		volume->index.ClearHashEntryAllocationStates();
		depth_based_allocator.collision_detected = false;

		ImageTraversalEngine<float, TMemoryDeviceType>::TraverseWithPosition(view->depth, depth_based_allocator);

		if (only_update_visible_list) {
			use_swapping = false;
			depth_based_allocator.collision_detected = false;
		} else {
			static_cast<TDerivedClass*>(this)->AllocateHashEntriesUsingLists_SetVisibility(volume);
		}
	} while (depth_based_allocator.collision_detected);

	static_cast<TDerivedClass*>(this)->BuildUtilizedBlockListBasedOnVisibility(volume, view, depth_camera_matrix);
	if (use_swapping) ReallocateDeletedHashBlocks(volume);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::AllocateFromDepth(
		VoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view, const CameraTrackingState* trackingState,
		bool onlyUpdateVisibleList, bool resetVisibleList) {
	AllocateFromDepth(scene, view, trackingState->pose_d->GetM(), onlyUpdateVisibleList, resetVisibleList);
}


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::AllocateFromDepthAndSdfSpan(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const CameraTrackingState* tracking_state,
		const ITMView* view) {

	float band_factor = configuration::get().general_voxel_volume_parameters.block_allocation_band_factor;
	float surface_distance_cutoff = band_factor * volume->sceneParams->narrow_band_half_width;

	TwoSurfaceBasedAllocationFunctor<TMemoryDeviceType> depth_based_allocator(
			volume->index, volume->sceneParams, view, tracking_state, surface_distance_cutoff);
	do {
		volume->index.ClearHashEntryAllocationStates();
		depth_based_allocator.collision_detected = false;

		TwoImageTraversalEngine<float, Vector4f, TMemoryDeviceType>::TraverseWithPosition(
				view->depth, tracking_state->pointCloud->locations, depth_based_allocator);
		AllocateHashEntriesUsingLists_SetVisibility(volume);

	} while (depth_based_allocator.collision_detected);
}


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::ReallocateDeletedHashBlocks(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	ReallocateDeletedHashBlocksFunctor<TVoxel, TMemoryDeviceType> reallocationFunctor(volume);
	HashTableTraversalEngine<TMemoryDeviceType>::TraverseWithHashCode(volume->index,reallocationFunctor);
	volume->localVBA.lastFreeBlockId = GET_ATOMIC_VALUE_CPU(reallocationFunctor.last_free_voxel_block_id);
}


} //namespace ITMLib