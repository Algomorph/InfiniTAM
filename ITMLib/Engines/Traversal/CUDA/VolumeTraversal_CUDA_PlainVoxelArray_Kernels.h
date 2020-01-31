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
#include "../../../Objects/Scene/VoxelVolume.h"
#include "../../../Objects/Scene/PlainVoxelArray.h"

namespace {
//CUDA device functions
template<typename TStaticFunctor, typename TVoxel>
__global__ void
staticVoxelTraversal_device(TVoxel* voxels, const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo) {
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
voxelTraversal_device(TVoxel* voxels, const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
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
voxelPositionTraversal_device(TVoxel* voxels, const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
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

template<typename TStaticFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void
staticDualVoxelTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                                const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;
	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];
	TStaticFunctor::run(voxelPrimary, voxelSecondary);
}


template<typename TStaticBooleanFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void
staticDualVoxelTraversal_AllTrue_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                                        const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
                                        bool* falseEncountered) {

	if (*falseEncountered) return;

	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;
	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];
	if(!TStaticBooleanFunctor::run(voxelPrimary, voxelSecondary)){
		*falseEncountered = true;
	}

}

template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void
dualVoxelPositionTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                                  const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
                                  TFunctor* functor) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	Vector3i voxelPosition(
	x + arrayInfo->offset.x,
	y + arrayInfo->offset.y,
	z + arrayInfo->offset.z);

	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];

	(*functor)(voxelPrimary, voxelSecondary, voxelPosition);
}

template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void
dualVoxelTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                          const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
                          TFunctor* functor) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];
	(*functor)(voxelPrimary, voxelSecondary);
}


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void
dualVoxelTraversal_AllTrue_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                                  const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
                                  TFunctor* functor, bool* falseEncountered) {
	if(*falseEncountered) return;

	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];
	if(!(*functor)(voxelPrimary, voxelSecondary)){
		*falseEncountered = true;
	}
}

template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void
dualVoxelPositionTraversal_AllTrue_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                                  const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
                                  TFunctor* functor, bool* falseEncountered) {
	if(*falseEncountered) return;

	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];

	Vector3i voxelPosition;

	voxelPosition.x = x + arrayInfo->offset.x;
	voxelPosition.y = y + arrayInfo->offset.y;
	voxelPosition.z = z + arrayInfo->offset.z;

	if(!(*functor)(voxelPrimary, voxelSecondary, voxelPosition)){
		*falseEncountered = true;
	}
}

template<typename TStaticFunctor, typename TVoxelPrimary, typename TVoxelSecondary, typename TWarp>
__global__ void
staticDualVoxelWarpTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                                    TWarp* warpVoxels, const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];
	TWarp& warp = warpVoxels[locId];

	TStaticFunctor::run(voxelPrimary, voxelSecondary, warp);
}


template<typename TStaticFunctor, typename TVoxelPrimary, typename TVoxelSecondary, typename TWarp>
__global__ void
staticDualVoxelWarpPositionTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                                            TWarp* warpVoxels,
                                            const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	Vector3i voxelPosition;

	voxelPosition.x = x + arrayInfo->offset.x;
	voxelPosition.y = y + arrayInfo->offset.y;
	voxelPosition.z = z + arrayInfo->offset.z;

	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];
	TWarp& warp = warpVoxels[locId];

	TStaticFunctor::run(voxelPrimary, voxelSecondary, warp, voxelPosition);

}


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary, typename TWarp>
__global__ void
dualVoxelWarpTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                              TWarp* warpVoxels, const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
                              TFunctor& functor) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];
	TWarp& warp = warpVoxels[locId];

	functor(voxelPrimary, voxelSecondary, warp);

}

template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary, typename TWarp>
__global__ void
dualVoxelWarpPositionTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                                      TWarp* warpVoxels, const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
                                      TFunctor* functor) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	Vector3i voxelPosition;

	voxelPosition.x = x + arrayInfo->offset.x;
	voxelPosition.y = y + arrayInfo->offset.y;
	voxelPosition.z = z + arrayInfo->offset.z;

	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];
	TWarp& warp = warpVoxels[locId];

	(*functor)(voxelPrimary, voxelSecondary, warp, voxelPosition);

}

}// end anonymous namespace (CUDA kernels)