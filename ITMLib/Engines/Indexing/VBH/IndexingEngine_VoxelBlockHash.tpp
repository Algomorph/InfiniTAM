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

#include "../../Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"

namespace ITMLib {

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
template<WarpType TWarpType, typename TWarp>
void IndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::AllocateFromWarpedVolume(
		ITMVoxelVolume<TWarp, VoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* targetTSDF) {

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


} //namespace ITMLib