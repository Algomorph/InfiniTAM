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
#include "../../../Utils/Math.h"

namespace { // CUDA kernels


template<typename TStaticFunctor, typename TVoxel1, typename TVoxel2, typename TVoxel3>
__global__ void
staticThreeVolumeTraversal_device(TVoxel1* voxels1, TVoxel2* voxels2,
                                  TVoxel3* voxels3, const ITMLib::GridAlignedBox* arrayInfo) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	TVoxel1& voxel1 = voxels1[locId];
	TVoxel2& voxel2 = voxels2[locId];
	TVoxel3& voxel3 = voxels3[locId];


	TStaticFunctor::run(voxel1, voxel2, voxel3);
}

template<typename TStaticFunctor, typename TVoxel1, typename TVoxel2, typename TVoxel3>
__global__ void
staticThreeVolumeTraversalWithPosition_device(TVoxel1* voxels1, TVoxel2* voxels2,
                                              TVoxel3* voxels3,
                                              const ITMLib::GridAlignedBox* arrayInfo) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	if (x >= arrayInfo->size.x || y >= arrayInfo->size.y || z >= arrayInfo->size.z) return;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	Vector3i voxelPosition;

	voxelPosition.x = x + arrayInfo->offset.x;
	voxelPosition.y = y + arrayInfo->offset.y;
	voxelPosition.z = z + arrayInfo->offset.z;

	TVoxel1& voxel1 = voxels1[locId];
	TVoxel2& voxel2 = voxels2[locId];
	TVoxel3& voxel3 = voxels3[locId];

	TStaticFunctor::run(voxel1, voxel2, voxel3, voxelPosition);

}


template<typename TFunctor, typename TVoxel1, typename TVoxel2, typename TVoxel3>
__global__ void
threeVolumeTraversal_device(TVoxel1* voxels1, TVoxel2* voxels2, TVoxel3* voxels3,
                            const ITMLib::GridAlignedBox* arrayInfo,
                            TFunctor& functor) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	TVoxel1& voxel1 = voxels1[locId];
	TVoxel2& voxel2 = voxels2[locId];
	TVoxel3& voxel3 = voxels3[locId];

	functor(voxel1, voxel2, voxel3);

}

template<typename TFunctor, typename TVoxel1, typename TVoxel2, typename TVoxel3>
__global__ void
threeVolumeTraversalWithPosition_device(TVoxel1* voxels1, TVoxel2* voxels2, TVoxel3* voxels3,
                                        const ITMLib::GridAlignedBox* arrayInfo,
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

	TVoxel1& voxel1 = voxels1[locId];
	TVoxel2& voxel2 = voxels2[locId];
	TVoxel3& voxel3 = voxels3[locId];

	(*functor)(voxel1, voxel2, voxel3, voxelPosition);

}

} // end anonymous namespace (CUDA kernels)
