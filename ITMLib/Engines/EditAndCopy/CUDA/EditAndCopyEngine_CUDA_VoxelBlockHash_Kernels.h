//  ================================================================
//  Created by Gregory Kramida on 10/2/19.
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

#include "../Shared/EditAndCopyEngine_Shared.h"
#include "../../../Utils/Enums/HashBlockProperties.h"
#include "../../../Utils/Geometry/GeometryBooleanOperations.h"

using namespace ITMLib;

namespace {

template<class TVoxel>
__global__ void noOffsetCopy_device(TVoxel* target_voxels, const TVoxel* source_voxels,
                                    const HashEntry* target_hash_table,
                                    const HashEntry* source_hash_table,
                                    int first_utilized_block_index,
                                    const int* target_utilized_block_hash_codes,
                                    int total_blocks_to_copy,
                                    const Vector6i bounds) {

	if (blockIdx.x >= total_blocks_to_copy) return;
	int utilized_entry_index = blockIdx.x + first_utilized_block_index;
	int target_hash_code = target_utilized_block_hash_codes[utilized_entry_index];
	const HashEntry& target_entry = target_hash_table[target_hash_code];
	int source_hash_code = FindHashCodeAt(source_hash_table, target_entry.pos);
	const HashEntry& source_entry = source_hash_table[source_hash_code];
	Vector3i hash_position_voxels = target_entry.pos.toInt() * VOXEL_BLOCK_SIZE;

	TVoxel * target_voxel_block = &(target_voxels[target_entry.ptr * VOXEL_BLOCK_SIZE3]);
	const TVoxel* source_voxel_block = &(source_voxels[source_entry.ptr * VOXEL_BLOCK_SIZE3]);

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	Vector3i globalPos = target_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
	globalPos.x += x;
	globalPos.y += y;
	globalPos.z += z;

	if (IsHashBlockFullyInBounds(hash_position_voxels, bounds) || IsPointInBounds(globalPos, bounds)) {
		target_voxel_block[locId] = source_voxel_block[locId];
	}
}

template<class TVoxel>
__global__ void offsetCopy_device(TVoxel* destinationVoxels, const TVoxel* source_voxels,
                                  const HashEntry* destinationHashTable,
                                  const HashEntry* sourceHashTable,
                                  int first_utilized_block_index,
                                  const int* target_utilized_block_hash_codes,
                                  int total_blocks_to_copy,
                                  const Vector3i offset,
                                  const Vector6i bounds) {
	// assume grid is one-dimensional, blockIdx corresponds to the destination block to copy
	if (blockIdx.x >= total_blocks_to_copy) return;
	int utilized_entry_index = blockIdx.x + first_utilized_block_index;
	int target_hash_code = target_utilized_block_hash_codes[utilized_entry_index];
	const HashEntry& target_hash_entry = destinationHashTable[target_hash_code];

	TVoxel* target_hash_block = &(destinationVoxels[target_hash_entry.ptr * VOXEL_BLOCK_SIZE3]);

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int target_voxel_index = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	Vector3i source_point = target_hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE - offset;
	source_point.x += x;
	source_point.y += y;
	source_point.z += z;

	int vmIndex;
	int source_voxel_index = findVoxel(sourceHashTable, source_point, vmIndex);
	if (!vmIndex) return;

	if (IsPointInBounds(source_point, bounds)) {
		target_hash_block[target_voxel_index] = source_voxels[source_voxel_index];
	}
}

template<typename TVoxel>
__global__ void setVoxel_device(TVoxel* voxels, HashEntry* hash_table, const int hash_code,
                                int voxel_index_in_block, TVoxel value) {
	TVoxel* localVoxelBlock = &(voxels[hash_table[hash_code].ptr * (VOXEL_BLOCK_SIZE3)]);
	localVoxelBlock[voxel_index_in_block] = value;
}

template<class TVoxel>
__global__ void readVoxel_device(TVoxel* voxelArray, const HashEntry* hashTable,
                                 const Vector3i at, ReadVoxelResult<TVoxel>* result) {
	int vmIndex = 0;
	int arrayIndex = findVoxel(hashTable, at, vmIndex);
	if (arrayIndex < 0) {
		result->found = false;
	} else {
		result->found = true;
		result->voxel = voxelArray[arrayIndex];
	}
}

template<class TVoxel>
__global__ void readVoxel_device(TVoxel* voxelArray, const HashEntry* hashTable,
                                 Vector3i at, ReadVoxelResult<TVoxel>* result,
                                 ITMLib::VoxelBlockHash::IndexCache* cache) {
	int vmIndex = 0;
	int arrayIndex = findVoxel(hashTable, at, vmIndex);
	if (arrayIndex < 0) {
		result->found = false;
		result->index = -1;
	} else {
		result->found = true;
		result->index = arrayIndex;
		result->voxel = voxelArray[arrayIndex];
	}
}


} // end anonymous namespace (CUDA kernels)
