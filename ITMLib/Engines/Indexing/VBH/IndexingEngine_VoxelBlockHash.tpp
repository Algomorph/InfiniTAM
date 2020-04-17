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

//local
#include "IndexingEngine_VoxelBlockHash.h"
#include "../../Traversal/Interface/ImageTraversal.h"
#include "../../Traversal/Interface/TwoImageTraversal.h"
#include "../../Traversal/Interface/HashTableTraversal.h"
#include "../../Traversal/Interface/VolumeTraversal.h"
#include "../Shared/IndexingEngine_Functors.h"
#include "../../../Utils/Configuration.h"

namespace ITMLib {


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::ResetUtilizedBlockList(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	volume->index.SetUtilizedBlockCount(0);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::AllocateNearSurface(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view, const Matrix4f& depth_camera_matrix) {

	float band_factor = configuration::get().general_voxel_volume_parameters.block_allocation_band_factor;
	float surface_distance_cutoff = band_factor * volume->GetParameters().narrow_band_half_width;

	DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType> depth_based_allocator(
			volume->index, volume->GetParameters(), view, depth_camera_matrix, surface_distance_cutoff);

	do {
		volume->index.ClearHashEntryAllocationStates();
		depth_based_allocator.resetFlagsAndCounters();
		ImageTraversalEngine<float, TMemoryDeviceType>::TraverseWithPosition(view->depth, depth_based_allocator);
		static_cast<TDerivedClass*>(this)->AllocateHashEntriesUsingAllocationStateList(volume);
		static_cast<TDerivedClass*>(this)->AllocateBlockList(volume, depth_based_allocator.colliding_block_positions,
		                                                     depth_based_allocator.getCollidingBlockCount());
	} while (depth_based_allocator.encounteredUnresolvableCollision());
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::AllocateNearSurface(
		VoxelVolume<TVoxel, VoxelBlockHash>* scene, const View* view, const CameraTrackingState* trackingState) {
	AllocateNearSurface(scene, view, trackingState->pose_d->GetM());
}


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::AllocateNearAndBetweenTwoSurfaces(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view, const CameraTrackingState* tracking_state) {

	volume->index.SetUtilizedBlockCount(0);

	float band_factor = configuration::get().general_voxel_volume_parameters.block_allocation_band_factor;
	float surface_distance_cutoff = band_factor * volume->GetParameters().narrow_band_half_width;

	TwoSurfaceBasedAllocationStateMarkerFunctor<TMemoryDeviceType> depth_based_allocator(
			volume->index, volume->GetParameters(), view, tracking_state, surface_distance_cutoff);

	do {
		volume->index.ClearHashEntryAllocationStates();
		depth_based_allocator.resetFlagsAndCounters();
		TwoImageTraversalEngine<float, Vector4f, TMemoryDeviceType>::TraverseWithPosition(
				view->depth, tracking_state->pointCloud->locations, depth_based_allocator);
		static_cast<TDerivedClass*>(this)->AllocateHashEntriesUsingAllocationStateList(volume);
		static_cast<TDerivedClass*>(this)->AllocateBlockList(volume, depth_based_allocator.colliding_block_positions,
		                                                     depth_based_allocator.getCollidingBlockCount());
	} while (depth_based_allocator.encounteredUnresolvableCollision());
}


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::ReallocateDeletedHashBlocks(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	ReallocateDeletedHashBlocksFunctor<TVoxel, TMemoryDeviceType> reallocation_functor(volume);
	HashTableTraversalEngine<TMemoryDeviceType>::TraverseAllWithHashCode(volume->index, reallocation_functor);
	volume->index.SetLastFreeBlockListId(GET_ATOMIC_VALUE_CPU(reallocation_functor.last_free_voxel_block_id));
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::AllocateGridAlignedBox(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const Extent3Di& box) {
	const Vector3i box_min_blocks = box.min() / VOXEL_BLOCK_SIZE;
	const Vector3i box_size_voxels_sans_front_margins = (box.max() - (box_min_blocks * VOXEL_BLOCK_SIZE));
	const Vector3i box_size_blocks = Vector3i(
			ceil_of_integer_quotient(box_size_voxels_sans_front_margins.x, VOXEL_BLOCK_SIZE),
			ceil_of_integer_quotient(box_size_voxels_sans_front_margins.y, VOXEL_BLOCK_SIZE),
			ceil_of_integer_quotient(box_size_voxels_sans_front_margins.z, VOXEL_BLOCK_SIZE));

	const int block_count = box_size_blocks.x * box_size_blocks.y * box_size_blocks.z;
	ORUtils::MemoryBlock<Vector3s> block_positions(block_count, true, true);
	Vector3s* block_positions_CPU = block_positions.GetData(MEMORYDEVICE_CPU);

	const GridAlignedBox ga_box(box_size_blocks, box_min_blocks);
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(block_positions_CPU)
#endif
	for (int i_block = 0; i_block < block_count; i_block++) {
		Vector3i position;
		ComputePositionFromLinearIndex_PlainVoxelArray(position.x, position.y, position.z, &ga_box, i_block);
		block_positions_CPU[i_block] = position.toShort();
	}

	block_positions.UpdateDeviceFromHost();
	static_cast<TDerivedClass*>(this)->AllocateBlockList(volume, block_positions, block_count);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::RebuildUtilizedBlockList(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	BuildUtilizedBlockListFunctor<TVoxel, TMemoryDeviceType> utilized_block_list_functor(volume);
	HashTableTraversalEngine<TMemoryDeviceType>::TraverseAllWithHashCode(volume->index, utilized_block_list_functor);
	volume->index.SetUtilizedBlockCount(GET_ATOMIC_VALUE_CPU(utilized_block_list_functor.utilized_block_count));
}


} //namespace ITMLib