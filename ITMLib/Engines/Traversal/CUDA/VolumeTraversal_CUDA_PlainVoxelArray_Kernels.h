//  ================================================================
//  Created by Gregory Kramida on 7/26/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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
#include "../Interface/VolumeTraversal.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"

namespace {
//CUDA device functions
template<typename TStaticFunctor, typename TVoxel>
__global__ void
StaticTraverseAll_device(TVoxel* voxels, const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;
	TVoxel& voxel = voxels[locId];
	TStaticFunctor::run(voxel);
}

template<typename TFunctor, typename TVoxel>
__global__ void
traverseAll_device(TVoxel* voxels, const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
                   TFunctor* functor) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;
	TVoxel& voxel = voxels[locId];
	(*functor)(voxel);
}

template<typename TFunctor, typename TVoxel>
__global__ void
traverseAllWithPosition_device(TVoxel* voxels, const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
                               TFunctor* functor) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	Vector3i voxelPosition(
			x + arrayInfo->offset.x,
			y + arrayInfo->offset.y,
			z + arrayInfo->offset.z);

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;
	TVoxel& voxel = voxels[locId];
	(*functor)(voxel, voxelPosition);
}





}// end anonymous namespace (CUDA kernels)