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
#include "../../../../ORUtils/JetbrainsCUDASyntax.hpp"
#include "../Shared/VolumeTraversal_Shared.h"
#include "../../../Utils/Analytics/IsAltered.h"

namespace {
// CUDA kernels

template<typename TStaticFunctor, typename TVoxel>
__global__ void
staticVoxelTraversal_device(TVoxel* voxels, const ITMHashEntry* hashTable) {
	int hash = blockIdx.x;
	const ITMHashEntry& hashEntry = hashTable[hash];
	if (hashEntry.ptr < 0) return;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	TVoxel& voxel = voxels[hashEntry.ptr * VOXEL_BLOCK_SIZE3 + locId];
	TStaticFunctor::run(voxel);
}

template<typename TFunctor, typename TVoxel>
__global__ void
voxelTraversal_device(TVoxel* voxels, const ITMHashEntry* hashTable,
                      TFunctor* functor) {
	int hash = blockIdx.x;
	const ITMHashEntry& hashEntry = hashTable[hash];
	if (hashEntry.ptr < 0) return;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	TVoxel& voxel = voxels[hashEntry.ptr * VOXEL_BLOCK_SIZE3 + locId];
	(*functor)(voxel);
}

template<typename TFunctor, typename TVoxel>
__global__ void
voxelPositionTraversal_device(TVoxel* voxels, const ITMHashEntry* hashTable,
                      TFunctor* functor) {
	int hash = blockIdx.x;
	const ITMHashEntry& hashEntry = hashTable[hash];
	if (hashEntry.ptr < 0) return;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	// position of the current entry in 3D space in voxel units
	Vector3i hashBlockPosition = hashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
	Vector3i voxelPosition = hashBlockPosition + Vector3i(x,y,z);

	TVoxel& voxel = voxels[hashEntry.ptr * VOXEL_BLOCK_SIZE3 + locId];
	(*functor)(voxel, voxelPosition);
}

template<typename TFunctor, typename TVoxel>
__global__ void
voxelAndHashBlockPositionTraversal_device(TVoxel* voxels, const ITMHashEntry* hashTable,
                              TFunctor* functor) {
	int hash = blockIdx.x;
	const ITMHashEntry& hashEntry = hashTable[hash];
	if (hashEntry.ptr < 0) return;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	// position of the current entry in 3D space in voxel units
	Vector3i hashBlockPosition = hashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
	Vector3i voxelPosition = hashBlockPosition + Vector3i(x,y,z);

	TVoxel& voxel = voxels[hashEntry.ptr * VOXEL_BLOCK_SIZE3 + locId];
	(*functor)(voxel, voxelPosition, hashEntry.pos);
}



}// end anonymous namespace (CUDA kernels)
