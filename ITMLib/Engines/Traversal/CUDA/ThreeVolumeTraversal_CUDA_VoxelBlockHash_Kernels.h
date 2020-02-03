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

//local
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../Shared/VolumeTraversal_Shared.h"

namespace { // CUDA kernels


template<typename TFunctor, typename TVoxel1, typename TVoxel2, typename TVoxel3>
__global__ void
threeVolumeTraversalWithPosition_device(TVoxel1* voxels1, TVoxel2* voxels2, TVoxel3* voxels3,
                                        const HashEntry* hash_table1, const HashEntry* hash_table2,
                                        const HashEntry* hash_table3, TFunctor* functor) {
	int hash_code1 = blockIdx.x;

	const HashEntry& hash_entry1 = hash_table1[hash_code1];
	if (hash_entry1.ptr < 0) return;
	HashEntry hash_entry2 = hash_table2[hash_code1];
	HashEntry hash_entry3 = hash_table3[hash_code1];

	if (hash_entry2.pos != hash_entry1.pos) {
		int hash_code2 = 0;
		if(!FindHashAtPosition(hash_code2, hash_entry1.pos, hash_table2)){
			printf("Attempted traversal of volume1 hash block %d at %d %d %d, but this block is absent from volume 2.\n",
			       hash_code1, hash_entry1.pos.x, hash_entry1.pos.y, hash_entry1.pos.z);
			DIEWITHEXCEPTION_REPORTLOCATION("No hash block with corresponding position found in hash table 2.");
		}
		hash_entry2 = hash_table2[hash_code2];
	}
	if (hash_entry3.pos != hash_entry1.pos) {
		int hash_code3 = 0;
		if(!FindHashAtPosition(hash_code3, hash_entry1.pos, hash_table3)){
			printf("Attempted traversal of volume1 hash block %d at %d %d %d, but this block is absent from volume 3.\n",
			       hash_code1, hash_entry1.pos.x, hash_entry1.pos.y, hash_entry1.pos.z);
			DIEWITHEXCEPTION_REPORTLOCATION("No hash block with corresponding position found in hash table 3.");
		}
		hash_entry3 = hash_table3[hash_code3];
	}


	int x = threadIdx.x;
	int y = threadIdx.y;
	int z = threadIdx.z;
	int linearIndexInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	// position of the current voxel in 3D space in voxel units
	Vector3i voxelPosition = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE + Vector3i(x, y, z);
	TVoxel1& voxel1 = voxels1[hash_entry1.ptr * (VOXEL_BLOCK_SIZE3) + linearIndexInBlock];
	TVoxel2& voxel2 = voxels2[hash_entry2.ptr * (VOXEL_BLOCK_SIZE3) + linearIndexInBlock];
	TVoxel3& voxel3 = voxels3[hash_entry3.ptr * (VOXEL_BLOCK_SIZE3) + linearIndexInBlock];
	(*functor)(voxel1, voxel2, voxel3, voxelPosition);
}

} // end anonymous namespace (CUDA kernels)