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
#include "../../Traversal/Interface/MemoryBlockTraversal.h"
#include "../../Traversal/Interface/TwoImageTraversal.h"
#include "../../Traversal/Interface/HashTableTraversal.h"
#include "../../Traversal/Interface/VolumeTraversal.h"
#include "../Shared/IndexingEngine_Functors.h"
#include "../../../Utils/Configuration/Configuration.h"


using namespace ITMLib;

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
template<typename TAllocationFunctor>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::
AllocateHashEntriesUsingAllocationStateList_Generic(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	TAllocationFunctor allocation_functor(volume->index);

	HashEntryAllocationState* hash_entry_allocation_states = volume->index.GetHashEntryAllocationStates();
	const int hash_entry_count = volume->index.hash_entry_count;
	MemoryBlockTraversalEngine<TMemoryDeviceType>::TraverseRaw(hash_entry_allocation_states,
	                                                           static_cast<unsigned int>(hash_entry_count),
	                                                           allocation_functor);
	allocation_functor.UpdateIndexCounters(volume->index);
}

/**
 * \brief Uses hash_entry_allocation_states of the given volume (presumably, previously initialized with an allocation-state-marker function)
 * to allocate new entries in the voxel block hash table index of the same volume. Does not modify block visibility.
 * \param volume volume where to allocate new voxel blocks.
 */
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::
AllocateHashEntriesUsingAllocationStateList(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	this->AllocateHashEntriesUsingAllocationStateList_Generic<HashEntryStateBasedAllocationFunctor<TMemoryDeviceType>>(
			volume);
}

/**
 * \brief Uses hash_entry_allocation_states of the given volume (presumably, previously initialized with an allocation-state-marker function)
 * to allocate new entries in the voxel block hash table index of the same volume. Modifies block visibility.
 * \details If an entry is successfully allocated in the excess list, sets visibility for that entry index to IN_MEMORY_AND_VISIBLE.
 * If no room is left in the ordered list, does not assign the voxel block pointer to the entry and sets the visibility
 * for that entry index to INVISIBLE.
 * \param volume volume where to allocate new voxel blocks.
 */
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::
AllocateHashEntriesUsingAllocationStateList_SetVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	this->AllocateHashEntriesUsingAllocationStateList_Generic<HashEntryStateBasedAllocationFunctor_SetVisibility<TMemoryDeviceType>>(
			volume);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::ResetUtilizedBlockList(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	volume->index.SetUtilizedBlockCount(0);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::ResetVisibleBlockList(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	volume->index.SetVisibleBlockCount(0);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::AllocateNearSurface(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view, const Matrix4f& depth_camera_matrix) {

	float band_factor = configuration::get().general_voxel_volume_parameters.block_allocation_band_factor;
	float surface_distance_cutoff = band_factor * volume->GetParameters().narrow_band_half_width;
	Matrix4f inverse_depth_camera_matrix;
	depth_camera_matrix.inv(inverse_depth_camera_matrix);

	DepthBasedAllocationStateMarkerFunctor<TMemoryDeviceType> depth_based_allocator(
			volume->index, volume->GetParameters(), view, inverse_depth_camera_matrix, surface_distance_cutoff);

	do {
		volume->index.ClearHashEntryAllocationStates();
		depth_based_allocator.ResetFlagsAndCounters();
		ImageTraversalEngine<float, TMemoryDeviceType>::TraverseWithPosition(view->depth, depth_based_allocator);
		this->AllocateHashEntriesUsingAllocationStateList(volume);
		this->AllocateBlockList(volume, depth_based_allocator.colliding_block_positions,
		                        depth_based_allocator.GetCollidingBlockCount());
	} while (depth_based_allocator.EncounteredUnresolvableCollision());
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::AllocateNearSurface(
		VoxelVolume<TVoxel, VoxelBlockHash>* scene, const View* view, const CameraTrackingState* trackingState) {
	AllocateNearSurface(scene, view, trackingState->pose_d->GetM());
}


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::AllocateNearAndBetweenTwoSurfaces(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view, const CameraTrackingState* tracking_state) {
	volume->index.SetUtilizedBlockCount(0);

	float band_factor = configuration::get().general_voxel_volume_parameters.block_allocation_band_factor;
	float surface_distance_cutoff = band_factor * volume->GetParameters().narrow_band_half_width;

	TwoSurfaceBasedAllocationStateMarkerFunctor<TMemoryDeviceType, TExecutionMode> depth_based_allocator(
			volume->index, volume->GetParameters(), view, tracking_state, surface_distance_cutoff, execution_mode_specialized_engine);
	do {
		volume->index.ClearHashEntryAllocationStates();
		depth_based_allocator.ResetFlagsAndCounters();
		TwoImageTraversalEngine<float, Vector4f, TMemoryDeviceType>::TraverseWithPosition(
				view->depth, tracking_state->pointCloud->locations, depth_based_allocator);
		this->AllocateHashEntriesUsingAllocationStateList(volume);
		this->AllocateBlockList(volume, depth_based_allocator.colliding_block_positions,
		                        depth_based_allocator.GetCollidingBlockCount());
	} while (depth_based_allocator.EncounteredUnresolvableCollision());
	depth_based_allocator.SaveDataToDisk();
}


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::ReallocateDeletedHashBlocks(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	ReallocateDeletedHashBlocksFunctor<TVoxel, TMemoryDeviceType> reallocation_functor(volume);
	HashTableTraversalEngine<TMemoryDeviceType>::TraverseAllWithHashCode(volume->index, reallocation_functor);
	volume->index.SetLastFreeBlockListId(GET_ATOMIC_VALUE_CPU(reallocation_functor.last_free_voxel_block_id));
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::AllocateGridAlignedBox(
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
	this->AllocateBlockList(volume, block_positions, block_count);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::RebuildUtilizedBlockList(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	BuildUtilizedBlockListFunctor<TVoxel, TMemoryDeviceType> utilized_block_list_functor(volume);
	HashTableTraversalEngine<TMemoryDeviceType>::TraverseAllWithHashCode(volume->index, utilized_block_list_functor);
	volume->index.SetUtilizedBlockCount(GET_ATOMIC_VALUE_CPU(utilized_block_list_functor.utilized_block_count));
}

/**
 * \brief Allocate all hash blocks at the first [0, new_block_count) of the given block coordinates
 * \param volume volume where to allocate
 * \param new_block_positions list of coordinates of blocks to allocate (coordinates to be specified in blocks, not voxels or meters)
 * \param new_block_count only the first [0, new_block_count) new_block_positions will be used.
 * If -1 is passed, new_block_positions.size() will be used.
 */
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::AllocateBlockList(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume,
		const ORUtils::MemoryBlock<Vector3s>& new_block_positions,
		int new_block_count) {
	if (new_block_count == -1) new_block_count = new_block_positions.size();
	if (new_block_count == 0) return;

	ORUtils::MemoryBlock<Vector3s> new_positions_local(new_block_count, TMemoryDeviceType);
	new_positions_local.SetFrom(new_block_positions);
	ORUtils::MemoryBlock<Vector3s> colliding_positions_local(new_block_count, TMemoryDeviceType);

	BlockListAllocationStateMarkerFunctor<TMemoryDeviceType> marker_functor(volume->index);
	Vector3s* new_positions_device = new_positions_local.GetData(TMemoryDeviceType);
	marker_functor.colliding_positions_device = colliding_positions_local.GetData(TMemoryDeviceType);

	while (new_block_count > 0) {
		marker_functor.SetCollidingBlockCount(0);
		volume->index.ClearHashEntryAllocationStates();

		MemoryBlockTraversalEngine<TMemoryDeviceType>::TraverseRaw(new_positions_device, new_block_count,
		                                                           marker_functor);

		AllocateHashEntriesUsingAllocationStateList(volume);

		new_block_count = marker_functor.GetCollidingBlockCount();
		std::swap(new_positions_device, marker_functor.colliding_positions_device);
	}
}

/**
 * \brief Deallocate all hash blocks at the first [0, count_of_blocks_to_remove) of the given block coordinates
 * \param volume volume where to deallocate
 * \param coordinates_of_blocks_to_remove coordinates (specified in blocks, not voxels or meters) from which to remove blocks
 * \param count_of_blocks_to_remove only the first [0, count_of_blocks_to_remove) coordinates_of_blocks_to_remove will be used.
 * If -1 is passed, coordinates_of_blocks_to_remove.size() will be used.
 */
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::DeallocateBlockList(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume,
		const ORUtils::MemoryBlock<Vector3s>& coordinates_of_blocks_to_remove,
		int count_of_blocks_to_remove) {

	if (count_of_blocks_to_remove == -1) count_of_blocks_to_remove = coordinates_of_blocks_to_remove.size();
	if (count_of_blocks_to_remove == 0) return;

	// *** locally-manipulated memory blocks of hash codes & counters *** /
	ORUtils::MemoryBlock<Vector3s> coordinates_of_blocks_to_remove_local(count_of_blocks_to_remove, TMemoryDeviceType);
	coordinates_of_blocks_to_remove_local.SetFrom(coordinates_of_blocks_to_remove);
	ORUtils::MemoryBlock<Vector3s> colliding_positions(count_of_blocks_to_remove, TMemoryDeviceType);

	BlockListDeallocationFunctor<TVoxel, TMemoryDeviceType> deallocation_functor(volume);
	Vector3s* blocks_to_remove_device = coordinates_of_blocks_to_remove_local.GetData(TMemoryDeviceType);
	deallocation_functor.colliding_positions_device = colliding_positions.GetData(TMemoryDeviceType);

	while (count_of_blocks_to_remove > 0) {
		deallocation_functor.SetCollidingBlockCount(0);
		volume->index.ClearHashEntryAllocationStates();

		MemoryBlockTraversalEngine<TMemoryDeviceType>::TraverseRaw(blocks_to_remove_device,
		                                                           count_of_blocks_to_remove, deallocation_functor);

		count_of_blocks_to_remove = deallocation_functor.GetCollidingBlockCount();
		std::swap(blocks_to_remove_device, deallocation_functor.colliding_positions_device);
	}
	deallocation_functor.SetIndexFreeVoxelBlockIdAndExcessListId();
	this->RebuildUtilizedBlockList(volume);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
HashEntry
IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::FindHashEntry(const VoxelBlockHash& index,
                                                                                         const Vector3s& coordinates) {
	int hash_code;
	return internal::IndexingEngine_VoxelBlockHash_MemoryDeviceTypeSpecialized<TMemoryDeviceType, TVoxel>::FindHashEntry(index, coordinates,
	                                                                                                                     hash_code);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
HashEntry
IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::FindHashEntry(const VoxelBlockHash& index,
                                                                                         const Vector3s& coordinates,
                                                                                         int& hash_code) {
	return internal::IndexingEngine_VoxelBlockHash_MemoryDeviceTypeSpecialized<TMemoryDeviceType, TVoxel>::FindHashEntry(index, coordinates,
	                                                                                                                     hash_code);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
bool IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::AllocateHashBlockAt(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hash_code) {
	return internal::IndexingEngine_VoxelBlockHash_MemoryDeviceTypeSpecialized<TMemoryDeviceType, TVoxel>::AllocateHashBlockAt(volume, at,
	                                                                                                                           hash_code);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode, HashBlockVisibility THashBlockVisibility>
static inline void ChangeVisibleBlockVisibility_Aux(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	const int* visible_block_hash_codes = volume->index.GetVisibleBlockHashCodes();
	BlockVisibilitySetFunctor<TVoxel, TMemoryDeviceType, THashBlockVisibility> visibility_functor(volume);
	MemoryBlockTraversalEngine<TMemoryDeviceType>::TraverseRaw(visible_block_hash_codes,
	                                                           volume->index.GetVisibleBlockCount(),
	                                                           visibility_functor);
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::ChangeVisibleBlockVisibility(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, HashBlockVisibility visibility) {

	switch (visibility) {
		case VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED:
			ChangeVisibleBlockVisibility_Aux<TVoxel, TMemoryDeviceType, TExecutionMode, VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED>(
					volume);
			break;
		case STREAMED_OUT_AND_VISIBLE:
			ChangeVisibleBlockVisibility_Aux<TVoxel, TMemoryDeviceType, TExecutionMode, STREAMED_OUT_AND_VISIBLE>(
					volume);
			break;
		case IN_MEMORY_AND_VISIBLE:
			ChangeVisibleBlockVisibility_Aux<TVoxel, TMemoryDeviceType, TExecutionMode, IN_MEMORY_AND_VISIBLE>(volume);
			break;
		case INVISIBLE:
			ChangeVisibleBlockVisibility_Aux<TVoxel, TMemoryDeviceType, TExecutionMode, INVISIBLE>(volume);
			break;
	}
}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode>::RebuildVisibleBlockList(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view, const Matrix4f& depth_camera_matrix) {
	internal::IndexingEngine_VoxelBlockHash_MemoryDeviceTypeSpecialized<TMemoryDeviceType, TVoxel>::RebuildVisibleBlockList(volume, view,
	                                                                                                                        depth_camera_matrix);
}

namespace ITMLib {
namespace internal {
template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource, typename TMarkerFunctor>
void AllocateUsingOtherVolume_Generic(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume,
		TMarkerFunctor& marker_functor) {

	assert(target_volume->index.hash_entry_count == source_volume->index.hash_entry_count);
	IndexingEngine<TVoxelTarget, VoxelBlockHash, TMemoryDeviceType>& indexer =
			IndexingEngine<TVoxelTarget, VoxelBlockHash, TMemoryDeviceType>::Instance();

	do {
		marker_functor.resetFlagsAndCounters();
		target_volume->index.ClearHashEntryAllocationStates();
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseUtilizedWithHashCode(
				source_volume->index, marker_functor);
		indexer.AllocateHashEntriesUsingAllocationStateList(target_volume);
		indexer.AllocateBlockList(target_volume, marker_functor.colliding_block_positions,
		                          marker_functor.getCollidingBlockCount());
	} while (marker_functor.encounteredUnresolvableCollision());
}

template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume) {
	VolumeBasedAllocationStateMarkerFunctor<TMemoryDeviceType>
			volume_based_allocation_state_marker(target_volume->index);
	internal::AllocateUsingOtherVolume_Generic<TMemoryDeviceType>(target_volume, source_volume,
	                                                              volume_based_allocation_state_marker);
}

template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume_Bounded(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume,
		const Extent3Di& bounds) {
	VolumeBasedBoundedAllocationStateMarkerFunctor<TMemoryDeviceType> volume_based_allocation_state_marker(
			target_volume->index, bounds);
	internal::AllocateUsingOtherVolume_Generic<TMemoryDeviceType>(target_volume, source_volume,
	                                                              volume_based_allocation_state_marker);
}

template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume_OffsetAndBounded(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume,
		const Extent3Di& source_bounds, const Vector3i& target_offset) {
	internal::AllocateUsingOtherVolume_OffsetAndBounded_Executor<TMemoryDeviceType, TVoxelTarget, TVoxelSource>::Execute(
			target_volume, source_volume, source_bounds, target_offset);
}

} // namespace internal

} //namespace ITMLib