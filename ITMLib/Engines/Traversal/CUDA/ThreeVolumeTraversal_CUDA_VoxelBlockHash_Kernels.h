//  ================================================================
//  Created by Gregory Kramida on 1/31/20.
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

#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"

namespace { // CUDA kernels


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary, typename TWarp>
__global__ void
dualVoxelWarpPositionTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels, TWarp* warpVoxels,
                                      const ITMHashEntry* primaryHashTable, const ITMHashEntry* secondaryHashTable,
                                      const ITMHashEntry* warpHashTable, TFunctor* functor) {
	int hashCode = blockIdx.x;

	const ITMHashEntry& primaryHashEntry = primaryHashTable[hashCode];
	if (primaryHashEntry.ptr < 0) return;
	ITMHashEntry secondaryHashEntry = secondaryHashTable[hashCode];
	ITMHashEntry warpHashEntry = warpHashTable[hashCode];

	if (secondaryHashEntry.pos != primaryHashEntry.pos) {
		int secondaryHashCode = 0;
		if(!FindHashAtPosition(secondaryHashCode, primaryHashEntry.pos, secondaryHashTable)){
			printf("Attempted traversal of primary hash block %d at %d %d %d, but this block is absent from secondary volume.\n",
			       hashCode, primaryHashEntry.pos.x, primaryHashEntry.pos.y, primaryHashEntry.pos.z);
			DIEWITHEXCEPTION_REPORTLOCATION("No hash block with corresponding position found in secondary hash table.");
		}
		secondaryHashEntry = secondaryHashTable[secondaryHashCode];
	}
	if (warpHashEntry.pos != primaryHashEntry.pos) {
		int warpHashCode = 0;
		if(!FindHashAtPosition(warpHashCode, primaryHashEntry.pos, warpHashTable)){
			printf("Attempted traversal of primary hash block %d at %d %d %d, but this block is absent from warp volume.\n",
			       hashCode, primaryHashEntry.pos.x, primaryHashEntry.pos.y, primaryHashEntry.pos.z);
			DIEWITHEXCEPTION_REPORTLOCATION("No hash block with corresponding position found in warp hash table.");
		}
		warpHashEntry = warpHashTable[warpHashCode];
	}


	int x = threadIdx.x;
	int y = threadIdx.y;
	int z = threadIdx.z;
	int linearIndexInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	// position of the current voxel in 3D space in voxel units
	Vector3i voxelPosition = primaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE + Vector3i(x, y, z);
	TVoxelPrimary& voxelPrimary = primaryVoxels[primaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3) + linearIndexInBlock];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[secondaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3) + linearIndexInBlock];
	TWarp& warp = warpVoxels[warpHashEntry.ptr *(VOXEL_BLOCK_SIZE3) + linearIndexInBlock];
	(*functor)(voxelPrimary, voxelSecondary, warp, voxelPosition);
}

} // end anonymous namespace (CUDA kernels)