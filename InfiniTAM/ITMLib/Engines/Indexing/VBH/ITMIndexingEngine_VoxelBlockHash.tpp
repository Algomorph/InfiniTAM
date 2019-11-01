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

#include "ITMIndexingEngine_VoxelBlockHash.h"
#include "../../Common/ITMCommonFunctors.h"
#include "../Shared/ITMIndexingEngine_Shared.h"

namespace ITMLib {

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
template<WarpType TWarpType, typename TWarp>
void ITMIndexingEngine_VoxelBlockHash<TVoxel, TMemoryDeviceType, TDerivedClass>::AllocateFromWarpedVolume(
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) {

	assert(warpField->index.hashEntryCount == sourceTSDF->index.hashEntryCount &&
	       sourceTSDF->index.hashEntryCount == targetTSDF->index.hashEntryCount);

	int hashEntryCount = warpField->index.hashEntryCount;


	ORUtils::MemoryBlock<HashEntryState> hashEntryStates(hashEntryCount, TMemoryDeviceType);
	HashEntryState* hashEntryStates_device = hashEntryStates.GetData(TMemoryDeviceType);
	ORUtils::MemoryBlock<Vector3s> blockCoordinates(hashEntryCount, TMemoryDeviceType);
	Vector3s* blockCoordinates_device = blockCoordinates.GetData(TMemoryDeviceType);

	//Mark up hash entries in the target scene that will need allocation
	WarpBasedAllocationMarkerFunctor<TWarp, TVoxel, WarpVoxelStaticFunctor<TWarp, TWarpType>>
			hashMarkerFunctor(sourceTSDF, targetTSDF, blockCoordinates_device, hashEntryStates_device);
	do{
		//reset allocation flags
		hashEntryStates.Clear(NEEDS_NO_CHANGE);
		hashMarkerFunctor.collisionDetected = false;
		ITMSceneTraversalEngine<TWarp, ITMVoxelBlockHash, TMemoryDeviceType>::VoxelAndHashBlockPositionTraversal(
				warpField, hashMarkerFunctor);

		//Allocate the hash entries that will potentially have any data

		static_cast<TDerivedClass*>(this)->AllocateHashEntriesUsingLists(targetTSDF, hashEntryStates_device, blockCoordinates_device);
	}while(hashMarkerFunctor.collisionDetected);
}


} //namespace ITMLib