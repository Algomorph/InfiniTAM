//  ================================================================
//  Created by Gregory Kramida on 10/8/19.
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

#include "../../../ORUtils/JetbrainsCUDASyntax.hpp"
#include "../Reconstruction/Shared/SceneReconstructionEngine_Shared.h"
#include "../../Utils/ITMCUDAUtils.h"
#include "../../Objects/Scene/ITMGlobalCache.h"
#include "ITMDynamicSceneReconstructionEngine_Shared.h"

namespace {
//CUDA kernels

template<class TVoxel>
__global__ void integrateIntoScene_device(TVoxel *localVBA, const ITMHashEntry *hashTable, int *visibleEntryIDs,
                                          const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, const float *confidence, Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d,
                                          Vector4f projParams_rgb, float _voxelSize, float mu, int maxW) {
	Vector3i globalPos;
	int entryId = visibleEntryIDs[blockIdx.x];

	const ITMHashEntry& currentHashEntry = hashTable[entryId];

	if (currentHashEntry.ptr < 0) return;

	globalPos = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

	TVoxel* localVoxelBlock = &(localVBA[currentHashEntry.ptr * VOXEL_BLOCK_SIZE3]);

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;

	Vector4f pt_model;
	int locId;

	locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	pt_model.x = (float) (globalPos.x + x) * _voxelSize;
	pt_model.y = (float) (globalPos.y + y) * _voxelSize;
	pt_model.z = (float) (globalPos.z + z) * _voxelSize;
	pt_model.w = 1.0f;

	ComputeUpdatedLiveVoxelInfo<TVoxel::hasColorInformation, TVoxel::hasConfidenceInformation, TVoxel::hasSemanticInformation, TVoxel>::compute(
			localVoxelBlock[locId], pt_model, M_d,
			projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, confidence,
			depthImgSize, rgb, rgbImgSize);
}

} // end anonymous namespace (CUDA kernels)