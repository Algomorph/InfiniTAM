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


__device__ inline void
checkOtherHashHasMatchingEntry(ITMLib::HashEntry& slave_hash_entry, const ITMLib::HashEntry* slave_hash_table,
                               const ITMLib::HashEntry& master_hash_entry, const int& master_hash_code, int volume_index) {
	if (slave_hash_entry.pos != master_hash_entry.pos) {
		int slave_hash_code = 0;
		if (!FindHashAtPosition(slave_hash_code, master_hash_entry.pos, slave_hash_table)) {
			printf("Attempted traversal of volume 1 hash block %d at %d %d %d, but this block is absent from volume %d.\n",
			       master_hash_code, master_hash_entry.pos.x, master_hash_entry.pos.y, master_hash_entry.pos.z,
			       volume_index);
			DIEWITHEXCEPTION_REPORTLOCATION("No hash block with corresponding position found in other hash table.");
		}
		slave_hash_entry = slave_hash_table[slave_hash_code];
	}
}

__device__ inline bool
getVoxelIndicesAndPositionInHashTables(
		int& voxel1_index, int& voxel2_index, int& voxel3_index, Vector3i& voxel_position,
		const int hash_code1,
		const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2, const ITMLib::HashEntry* hash_table3) {

	const ITMLib::HashEntry& hash_entry1 = hash_table1[hash_code1];
	if (hash_entry1.ptr < 0){
		return false;
	}
	ITMLib::HashEntry hash_entry2 = hash_table2[hash_code1];
	ITMLib::HashEntry hash_entry3 = hash_table3[hash_code1];

	checkOtherHashHasMatchingEntry(hash_entry2, hash_table2, hash_entry1, hash_code1, 2);
	checkOtherHashHasMatchingEntry(hash_entry3, hash_table3, hash_entry1, hash_code1, 3);

	int x = threadIdx.x;
	int y = threadIdx.y;
	int z = threadIdx.z;
	int linear_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	// position of the current voxel in 3D space in voxel units
	voxel_position = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE + Vector3i(x, y, z);

	voxel1_index = hash_entry1.ptr * (VOXEL_BLOCK_SIZE3) + linear_index_in_block;
	voxel2_index = hash_entry2.ptr * (VOXEL_BLOCK_SIZE3) + linear_index_in_block;
	voxel3_index = hash_entry3.ptr * (VOXEL_BLOCK_SIZE3) + linear_index_in_block;
	return true;
}

template<typename TFunctor, typename TVoxel1, typename TVoxel2, typename TVoxel3>
__global__ void
traverseAllWithPosition_device(TVoxel1* voxels1, TVoxel2* voxels2, TVoxel3* voxels3,
                               const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
                               const ITMLib::HashEntry* hash_table3, TFunctor* functor) {
	int hash_code1 = blockIdx.x;
	int voxel1_index, voxel2_index, voxel3_index;
	Vector3i voxel_position;
	if (!getVoxelIndicesAndPositionInHashTables(voxel1_index, voxel2_index, voxel3_index, voxel_position, hash_code1,
	                                       hash_table1, hash_table2, hash_table3)) return;

	TVoxel1& voxel1 = voxels1[voxel1_index];
	TVoxel2& voxel2 = voxels2[voxel2_index];
	TVoxel3& voxel3 = voxels3[voxel3_index];
	(*functor)(voxel1, voxel2, voxel3, voxel_position);
}


template<typename TFunctor, typename TVoxel1, typename TVoxel2, typename TVoxel3>
__global__ void
traverseUtilizedWithPosition_device(TVoxel1* voxels1, TVoxel2* voxels2, TVoxel3* voxels3,
                               const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
                               const ITMLib::HashEntry* hash_table3, const int* utilized_hash_codes, TFunctor* functor) {
	int hash_code1 = utilized_hash_codes[blockIdx.x];
	int voxel1_index, voxel2_index, voxel3_index;
	Vector3i voxel_position;
	getVoxelIndicesAndPositionInHashTables(voxel1_index, voxel2_index, voxel3_index, voxel_position, hash_code1,
	                                       hash_table1, hash_table2, hash_table3);

	TVoxel1& voxel1 = voxels1[voxel1_index];
	TVoxel2& voxel2 = voxels2[voxel2_index];
	TVoxel3& voxel3 = voxels3[voxel3_index];
	(*functor)(voxel1, voxel2, voxel3, voxel_position);
}

} // end anonymous namespace (CUDA kernels)