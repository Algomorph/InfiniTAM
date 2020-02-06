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

#include "IndexingEngine_CPU_VoxelBlockHash.h"
#include "../../../../Objects/Volume/RepresentationAccess.h"
#include "../../../EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../Shared/IndexingEngine_Functors.h"
#include "../../../Common/CheckBlockVisibility.h"
#include "../../../../Utils/Configuration.h"
#include "../../../../Utils/Geometry/FrustumTrigonometry.h"
#include "../../../Traversal/CPU/ImageTraversal_CPU.h"
#include "../../../Traversal/CPU/TwoImageTraversal_CPU.h"
#include "../../../Traversal/CPU/HashTableTraversal_CPU.h"

using namespace ITMLib;


template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::ReallocateDeletedHashBlocks(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	ReallocateDeletedHashBlocksFunctor<TVoxel, MEMORYDEVICE_CPU> reallocationFunctor(volume);
	HashTableTraversalEngine<MEMORYDEVICE_CPU>::TraverseWithHashCode(volume->index,reallocationFunctor);
	volume->localVBA.lastFreeBlockId = GET_ATOMIC_VALUE_CPU(reallocationFunctor.last_free_voxel_block_id);
}

template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateFromDepth(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
		const Matrix4f& depth_camera_matrix, bool only_update_visible_list, bool resetVisibleList) {

	if (resetVisibleList) volume->index.SetUtilizedHashBlockCount(0);

	float band_factor = configuration::get().general_voxel_volume_parameters.block_allocation_band_factor;
	float surface_distance_cutoff = band_factor * volume->sceneParams->narrow_band_half_width;

	bool use_swapping = volume->globalCache != nullptr;

	DepthBasedAllocationFunctor<MEMORYDEVICE_CPU> depth_based_allocator(
			volume->index, volume->sceneParams, view, depth_camera_matrix, surface_distance_cutoff);

	//reset visibility of formerly "visible" entries, if any
	SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(volume);
	do {
		volume->index.ClearHashEntryAllocationStates();
		depth_based_allocator.collision_detected = false;

		ImageTraversalEngine<float, MEMORYDEVICE_CPU>::TraverseWithPosition(view->depth, depth_based_allocator);

		if (only_update_visible_list) {
			use_swapping = false;
			depth_based_allocator.collision_detected = false;
		} else {
			AllocateHashEntriesUsingLists_SetVisibility(volume);
		}
	} while (depth_based_allocator.collision_detected);

	BuildUtilizedBlockListBasedOnVisibility(volume, view, depth_camera_matrix);
	if (use_swapping) ReallocateDeletedHashBlocks(volume);
}


template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateFromDepth(
		VoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view, const CameraTrackingState* trackingState,
		bool onlyUpdateVisibleList, bool resetVisibleList) {
	AllocateFromDepth(scene, view, trackingState->pose_d->GetM(), onlyUpdateVisibleList, resetVisibleList);
}

template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateFromDepthAndSdfSpan(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const CameraTrackingState* tracking_state,
		const ITMView* view) {

	float band_factor = configuration::get().general_voxel_volume_parameters.block_allocation_band_factor;
	float surface_distance_cutoff = band_factor * volume->sceneParams->narrow_band_half_width;

	bool use_swapping = volume->globalCache != nullptr;

	TwoSurfaceBasedAllocationFunctor<MEMORYDEVICE_CPU> depth_based_allocator(
			volume->index, volume->sceneParams, view, tracking_state, surface_distance_cutoff);
	do {
		volume->index.ClearHashEntryAllocationStates();
		depth_based_allocator.collision_detected = false;

		TwoImageTraversalEngine<float, Vector4f, MEMORYDEVICE_CPU>::TraverseWithPosition(
				view->depth, tracking_state->pointCloud->locations, depth_based_allocator);
		AllocateHashEntriesUsingLists_SetVisibility(volume);

	} while (depth_based_allocator.collision_detected);
}

template<typename TVoxel>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume) {

	assert(targetVolume->index.hashEntryCount == sourceVolume->index.hashEntryCount);

	const int hashEntryCount = targetVolume->index.hashEntryCount;

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();
	HashEntry* targetHashEntries = targetVolume->index.GetEntries();
	HashEntry* sourceHashEntries = sourceVolume->index.GetEntries();

	bool collisionDetected;

	do {
		collisionDetected = false;
		//reset target allocation states
		targetVolume->index.ClearHashEntryAllocationStates();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(sourceHashEntries, hashEntryStates_device, blockCoordinates_device, \
		targetHashEntries, collisionDetected)
#endif
		for (int sourceHash = 0; sourceHash < hashEntryCount; sourceHash++) {

			const HashEntry& currentSourceHashBlock = sourceHashEntries[sourceHash];
			//skip unfilled live blocks
			if (currentSourceHashBlock.ptr < 0) {
				continue;
			}
			Vector3s sourceHashBlockCoords = currentSourceHashBlock.pos;

			//try to find a corresponding canonical block, and mark it for allocation if not found
			int targetHash = HashCodeFromBlockPosition(sourceHashBlockCoords);

			MarkAsNeedingAllocationIfNotFound(hashEntryStates_device, blockCoordinates_device,
			                                  targetHash, sourceHashBlockCoords, targetHashEntries,
			                                  collisionDetected);
		}

		IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists(targetVolume);
	} while (collisionDetected);

}

static std::vector<Vector3s> neighborOffsets = [] { // NOLINT(cert-err58-cpp)
	std::vector<Vector3s> offsets;
	for (short zOffset = -1; zOffset < 2; zOffset++) {
		for (short yOffset = -1; yOffset < 2; yOffset++) {
			for (short xOffset = -1; xOffset < 2; xOffset++) {
				Vector3s neighborOffset(xOffset, yOffset, zOffset);
				offsets.push_back(neighborOffset);
			}
		}
	}
	return offsets;
}();

template<typename TVoxelTarget, typename TVoxelSource, typename THashBlockMarkProcedure, typename TAllocationProcedure>
void AllocateUsingOtherVolumeExpanded_Generic(VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
                                              VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume,
                                              THashBlockMarkProcedure&& hashBlockMarkProcedure,
                                              TAllocationProcedure&& allocationProcedure) {
	assert(sourceVolume->index.hashEntryCount == targetVolume->index.hashEntryCount);

	int hashEntryCount = targetVolume->index.hashEntryCount;
	HashEntry* targetHashTable = targetVolume->index.GetEntries();
	HashEntry* sourceHashTable = sourceVolume->index.GetEntries();

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();

	bool collision_detected;
	do {
		collision_detected = false;
		targetVolume->index.ClearHashEntryAllocationStates();
#ifdef WITH_OPENMP
#pragma omp parallel for default(shared)
#endif
		for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
			const HashEntry& hashEntry = sourceHashTable[hashCode];
			// skip empty blocks
			if (hashEntry.ptr < 0) continue;
			for (auto& neighborOffset : neighborOffsets) {
				Vector3s neighborBlockCoordinates = hashEntry.pos + neighborOffset;
				// try to find a corresponding canonical block, and mark it for allocation if not found
				int targetHash = HashCodeFromBlockPosition(neighborBlockCoordinates);
				std::forward<THashBlockMarkProcedure>(hashBlockMarkProcedure)(neighborBlockCoordinates, targetHash,
				                                                              collision_detected);
			}
		}
		std::forward<TAllocationProcedure>(allocationProcedure)();
	} while (collision_detected);
}

template<typename TVoxel>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume) {

	HashEntry* targetHashTable = targetVolume->index.GetEntries();

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();

	auto hashBlockMarkProcedure = [&](Vector3s neighborBlockCoordinates, int& targetHash, bool& collision_detected) {
		MarkAsNeedingAllocationIfNotFound(hashEntryStates_device, blockCoordinates_device,
		                                  targetHash, neighborBlockCoordinates, targetHashTable,
		                                  collision_detected);
	};
	auto allocationProcedure = [&]() {
		IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists(targetVolume);
	};
	AllocateUsingOtherVolumeExpanded_Generic(targetVolume, sourceVolume, hashBlockMarkProcedure, allocationProcedure);
}


template<typename TVoxel>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix) {

	HashEntry* targetHashTable = targetVolume->index.GetEntries();
	HashBlockVisibility* hashBlockVisibilityTypes_device = targetVolume->index.GetBlockVisibilityTypes();

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();

	IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(targetVolume);

	auto hashBlockMarkProcedure = [&](Vector3s neighborBlockCoordinates, int& targetHash, bool& collision_detected) {
		MarkForAllocationAndSetVisibilityTypeIfNotFound(
				hashEntryStates_device, blockCoordinates_device, hashBlockVisibilityTypes_device,
				neighborBlockCoordinates, targetHashTable, collision_detected);
	};
	auto allocationProcedure = [&]() {
		IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists_SetVisibility(targetVolume);
	};
	AllocateUsingOtherVolumeExpanded_Generic(targetVolume, sourceVolume, hashBlockMarkProcedure, allocationProcedure);

	IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().BuildUtilizedBlockListBasedOnVisibility(
			targetVolume,
			view,
			depth_camera_matrix);
}

// #define EXCEPT_ON_OUT_OF_SPACE
template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
AllocateHashEntriesUsingLists(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	const HashEntryAllocationState* hashEntryStates_device = volume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = volume->index.GetAllocationBlockCoordinates();

	const int hashEntryCount = volume->index.hashEntryCount;
	int lastFreeVoxelBlockId = volume->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = volume->index.GetLastFreeExcessListId();
	int* voxelAllocationList = volume->localVBA.GetAllocationList();
	int* excessAllocationList = volume->index.GetExcessAllocationList();
	HashEntry* hashTable = volume->index.GetEntries();

	for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
		const HashEntryAllocationState& hashEntryState = hashEntryStates_device[hashCode];
		switch (hashEntryState) {
			case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST:

				if (lastFreeVoxelBlockId >= 0) //there is room in the voxel block array
				{
					HashEntry hashEntry;
					hashEntry.pos = blockCoordinates_device[hashCode];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					hashTable[hashCode] = hashEntry;
					lastFreeVoxelBlockId--;
				}
#ifdef EXCEPT_ON_OUT_OF_SPACE
			else {
				DIEWITHEXCEPTION_REPORTLOCATION("Not enough space in ordered list.");
			}
#endif

				break;
			case NEEDS_ALLOCATION_IN_EXCESS_LIST:

				if (lastFreeVoxelBlockId >= 0 &&
				    lastFreeExcessListId >= 0) //there is room in the voxel block array and excess list
				{
					HashEntry hashEntry;
					hashEntry.pos = blockCoordinates_device[hashCode];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					int exlOffset = excessAllocationList[lastFreeExcessListId];
					hashTable[hashCode].offset = exlOffset + 1; //connect to child
					hashTable[ORDERED_LIST_SIZE + exlOffset] = hashEntry; //add child to the excess list
					lastFreeVoxelBlockId--;
					lastFreeExcessListId--;
				}
#ifdef EXCEPT_ON_OUT_OF_SPACE
			else {
				DIEWITHEXCEPTION_REPORTLOCATION("Not enough space in excess list.");
			}
#endif
				break;
			default:
				break;
		}
	}
	volume->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	volume->index.SetLastFreeExcessListId(lastFreeExcessListId);
}


template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
AllocateHashEntriesUsingLists_SetVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	const HashEntryAllocationState* hashEntryAllocationStates_device = volume->index.GetHashEntryAllocationStates();
	Vector3s* allocationBlockCoordinates_device = volume->index.GetAllocationBlockCoordinates();
	HashBlockVisibility* hashBlockVisibilityTypes_device = volume->index.GetBlockVisibilityTypes();

	int entryCount = volume->index.hashEntryCount;
	int lastFreeVoxelBlockId = volume->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = volume->index.GetLastFreeExcessListId();
	int* voxelAllocationList = volume->localVBA.GetAllocationList();
	int* excessAllocationList = volume->index.GetExcessAllocationList();
	HashEntry* hashTable = volume->index.GetEntries();

	for (int hash = 0; hash < entryCount; hash++) {
		const HashEntryAllocationState& hashEntryState = hashEntryAllocationStates_device[hash];
		switch (hashEntryState) {
			case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST:

				if (lastFreeVoxelBlockId >= 0) //there is room in the voxel block array
				{
					HashEntry hashEntry;
					hashEntry.pos = allocationBlockCoordinates_device[hash];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					hashTable[hash] = hashEntry;
					lastFreeVoxelBlockId--;
				} else {
					hashBlockVisibilityTypes_device[hash] = INVISIBLE;
				}

				break;
			case NEEDS_ALLOCATION_IN_EXCESS_LIST:

				if (lastFreeVoxelBlockId >= 0 &&
				    lastFreeExcessListId >= 0) //there is room in the voxel block array and excess list
				{
					HashEntry hashEntry;
					hashEntry.pos = allocationBlockCoordinates_device[hash];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					int exlOffset = excessAllocationList[lastFreeExcessListId];
					hashTable[hash].offset = exlOffset + 1; //connect to child
					hashTable[ORDERED_LIST_SIZE + exlOffset] = hashEntry; //add child to the excess list
					hashBlockVisibilityTypes_device[ORDERED_LIST_SIZE +
					                                exlOffset] = IN_MEMORY_AND_VISIBLE; //make child visible and in memory
					lastFreeVoxelBlockId--;
					lastFreeExcessListId--;
				}
				break;
			default:
				break;
		}
	}
	volume->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	volume->index.SetLastFreeExcessListId(lastFreeExcessListId);
}


template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::BuildUtilizedBlockListBasedOnVisibility(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view, const Matrix4f& depth_camera_matrix) {
	SpecializedAllocationFunctionsEngine<TVoxel, MEMORYDEVICE_CPU>::BuildUtilizedBlockListBasedOnVisibility(volume, view, depth_camera_matrix);
}

template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	SpecializedAllocationFunctionsEngine<TVoxel, MEMORYDEVICE_CPU>::SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(volume);
}

template<typename TVoxel>
HashEntry
IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::FindHashEntry(const VoxelBlockHash& index,
                                                                        const Vector3s& coordinates) {
	const HashEntry* entries = index.GetEntries();
	int hashCode = FindHashCodeAt(entries, coordinates);
	if (hashCode == -1) {
		return {Vector3s(0, 0, 0), 0, -2};
	} else {
		return entries[hashCode];
	}
}

template<typename TVoxel>
HashEntry
IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::FindHashEntry(const VoxelBlockHash& index,
                                                                        const Vector3s& coordinates,
                                                                        int& hashCode) {
	const HashEntry* entries = index.GetEntries();
	hashCode = FindHashCodeAt(entries, coordinates);
	if (hashCode == -1) {
		return {Vector3s(0, 0, 0), 0, -2};
	} else {
		return entries[hashCode];
	}
}

template<typename TVoxel>
bool IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateHashBlockAt(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hashCode) {

	HashEntry* hashTable = volume->index.GetEntries();
	int lastFreeVoxelBlockId = volume->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = volume->index.GetLastFreeExcessListId();
	int* voxelAllocationList = volume->localVBA.GetAllocationList();
	int* excessAllocationList = volume->index.GetExcessAllocationList();
	HashEntry* entry = nullptr;
	hashCode = -1;
	if (!FindOrAllocateHashEntry(at, hashTable, entry, lastFreeVoxelBlockId, lastFreeExcessListId,
	                             voxelAllocationList, excessAllocationList, hashCode)) {
		return false;
	}
	volume->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	volume->index.SetLastFreeExcessListId(lastFreeExcessListId);
	return true;
}