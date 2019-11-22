//  ================================================================
//  Created by Gregory Kramida on 10/9/19.
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

#include "../Shared/ITMDynamicSceneReconstructionEngine_Shared.h"


//TODO: refactor this to use CUDA-based traversal methods where applicable -Greg (GitHub: Algomorph)

using namespace ITMLib;

namespace {
// CUDA kernels

template<typename TVoxel>
__global__ void integrateIntoScene_device(TVoxel* voxelArray, const ITMLib::ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo,
                                          const Vector4u* rgb, Vector2i rgbImgSize, const float* depth,
                                          const float* confidence,
                                          Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d,
                                          Vector4f projParams_rgb, float _voxelSize, float mu, int maxW) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	Vector4f pt_model;
	int locId;

	locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	pt_model.x = (float) (x + arrayInfo->offset.x) * _voxelSize;
	pt_model.y = (float) (y + arrayInfo->offset.y) * _voxelSize;
	pt_model.z = (float) (z + arrayInfo->offset.z) * _voxelSize;
	pt_model.w = 1.0f;

//	_DEBUG
	if(x == 16 & y == 91 && z == 160){
		printf("GOTCHA: (%E, %E, %E)\n", pt_model.x, pt_model.y, pt_model.z);
	}

	ComputeUpdatedLiveVoxelInfo<TVoxel::hasColorInformation, TVoxel::hasConfidenceInformation, TVoxel::hasSemanticInformation, TVoxel>::compute(
			voxelArray[locId], pt_model, M_d,
			projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, confidence,
			depthImgSize, rgb, rgbImgSize);
}

template<typename TVoxel>
__global__ void fuseSdf2Sdf_device(TVoxel* voxelArrayLive, TVoxel* voxelArrayCanonical,
                                   const ITMLib::ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo, int maximumWeight) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	fuseLiveVoxelIntoCanonical(voxelArrayLive[locId], maximumWeight, voxelArrayCanonical[locId]);
}

template<typename TVoxel>
__global__ void
clearFields_device(TVoxel* voxelArray, const ITMLib::ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;
	TVoxel& voxel = voxelArray[locId];
	voxel.flags = ITMLib::VOXEL_UNKNOWN;
	voxel.sdf = TVoxel::SDF_initialValue();
}

template<typename TVoxel, typename TWarp, typename TLookupPositionFunctor>
__global__ void interpolateTriliearlyUsingWarps(
		TWarp* warpSourceVoxels,
		TVoxel* sdfSourceVoxels,
		TVoxel* sdfTargetVoxels,
		const ITMPlainVoxelArray::IndexData* sdfSourceIndexData) {

	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * sdfSourceIndexData->size.x + z * sdfSourceIndexData->size.x * sdfSourceIndexData->size.y;

	TVoxel& destinationVoxel = sdfTargetVoxels[locId];
	TWarp& warp = warpSourceVoxels[locId];
	ITMPlainVoxelArray::IndexCache sdfSourceCache;

	Vector3i warpAndDestinationVoxelPosition;

	warpAndDestinationVoxelPosition.x = x + sdfSourceIndexData->offset.x;
	warpAndDestinationVoxelPosition.y = y + sdfSourceIndexData->offset.y;
	warpAndDestinationVoxelPosition.z = z + sdfSourceIndexData->offset.z;

	interpolateTSDFVolume<TVoxel, TWarp, ITMPlainVoxelArray, TLookupPositionFunctor>(
			sdfSourceVoxels, sdfSourceIndexData, sdfSourceCache,
			warp, destinationVoxel,
			warpAndDestinationVoxelPosition, false);
}

template<typename TVoxelMulti>
__global__ void
copyScene_device(TVoxelMulti* sourceVoxels,
                 TVoxelMulti* targetVoxels,
                 const ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;
	TVoxelMulti& targetVoxel = targetVoxels[locId];
	TVoxelMulti& sourceVoxel = sourceVoxels[locId];
	sourceVoxel.sdf = targetVoxel.sdf;
	sourceVoxel.flags = targetVoxel.flags;

}

} // end anonymous namespace (CUDA kernels)
