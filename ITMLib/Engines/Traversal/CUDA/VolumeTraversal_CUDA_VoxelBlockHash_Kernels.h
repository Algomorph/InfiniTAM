//  ================================================================
//  Created by Gregory Kramida on 8/13/19.
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

#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../Shared/VolumeTraversal_Shared.h"
#include "../../../Utils/Analytics/IsAltered.h"

namespace {
// CUDA kernels

template<typename TStaticFunctor, typename TVoxel>
__global__ void
traverseAll_StaticFunctor_device(TVoxel* voxels, const ITMLib::HashEntry* hash_table) {
	int hash_code = blockIdx.x;
	const ITMLib::HashEntry& hash_entry = hash_table[hash_code];
	if (hash_entry.ptr < 0) return;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	TVoxel& voxel = voxels[hash_entry.ptr * VOXEL_BLOCK_SIZE3 + locId];
	TStaticFunctor::run(voxel);
}

template<typename TStaticFunctor, typename TVoxel>
__global__ void
traverseUtilized_StaticFunctor_device(TVoxel* voxels, const ITMLib::HashEntry* hash_table, const int* utilized_hash_codes) {
	int hash_code = utilized_hash_codes[blockIdx.x];
	const ITMLib::HashEntry& hash_entry = hash_table[hash_code];
	if (hash_entry.ptr < 0) return;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	TVoxel& voxel = voxels[hash_entry.ptr * VOXEL_BLOCK_SIZE3 + locId];
	TStaticFunctor::run(voxel);
}

template<typename TFunctor, typename TVoxel>
__global__ void
traverseAll_device(TVoxel* voxels, const ITMLib::HashEntry* hash_table, TFunctor* functor) {
	int hash = blockIdx.x;
	const ITMLib::HashEntry& hash_entry = hash_table[hash];
	if (hash_entry.ptr < 0) return;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int voxel_index_within_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	TVoxel& voxel = voxels[hash_entry.ptr * VOXEL_BLOCK_SIZE3 + voxel_index_within_block];
	(*functor)(voxel);
}

template<typename TFunctor, typename TVoxel>
__global__ void
traverseUtilized_device(TVoxel* voxels, const ITMLib::HashEntry* hash_table, const int* utilized_hash_codes,
                        TFunctor* functor) {
	int hash_code = utilized_hash_codes[blockIdx.x];
	const ITMLib::HashEntry& hash_entry = hash_table[hash_code];

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int voxel_index_within_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	TVoxel& voxel = voxels[hash_entry.ptr * VOXEL_BLOCK_SIZE3 + voxel_index_within_block];
	(*functor)(voxel);
}

template<typename TFunctor, typename TVoxel>
__global__ void
traverseAllWithPosition_device(TVoxel* voxels, const ITMLib::HashEntry* hash_table,
                               TFunctor* functor) {
	int hash = blockIdx.x;
	const ITMLib::HashEntry& hash_entry = hash_table[hash];
	if (hash_entry.ptr < 0) return;

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int voxel_index_within_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	// position of the current entry in 3D space in voxel units
	Vector3i hash_block_position = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
	Vector3i voxel_position = hash_block_position + Vector3i(x, y, z);

	TVoxel& voxel = voxels[hash_entry.ptr * VOXEL_BLOCK_SIZE3 + voxel_index_within_block];
	(*functor)(voxel, voxel_position);
}

template<typename TFunctor, typename TVoxel>
__global__ void
traverseUtilizedWithPosition_device(TVoxel* voxels, const ITMLib::HashEntry* hash_table, const int* utilized_hash_codes,
                        TFunctor* functor) {
	int hash_code = utilized_hash_codes[blockIdx.x];
	const ITMLib::HashEntry& hash_entry = hash_table[hash_code];

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int voxel_index_within_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	// position of the current entry in 3D space in voxel units
	Vector3i hash_block_position = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
	Vector3i voxel_position = hash_block_position + Vector3i(x, y, z);

	TVoxel& voxel = voxels[hash_entry.ptr * VOXEL_BLOCK_SIZE3 + voxel_index_within_block];
	(*functor)(voxel, voxel_position);
}

template<typename TFunctor, typename TVoxel>
__global__ void
traverseAllWithPositionAndBlockPosition_device(TVoxel* voxels, const ITMLib::HashEntry* hash_table,
                                               TFunctor* functor) {
	int hash = blockIdx.x;
	const ITMLib::HashEntry& hash_entry = hash_table[hash];
	if (hash_entry.ptr < 0) return;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	// position of the current entry in 3D space in voxel units
	Vector3i hashBlockPosition = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
	Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);

	TVoxel& voxel = voxels[hash_entry.ptr * VOXEL_BLOCK_SIZE3 + locId];
	(*functor)(voxel, voxelPosition, hash_entry.pos);
}


}// end anonymous namespace (CUDA kernels)
