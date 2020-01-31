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
#include "../Interface/ThreeVolumeTraversal.h"
#include "ThreeVolumeTraversal_CUDA_VoxelBlockHash_Kernels.h"

namespace ITMLib{

template<typename TVoxel, typename TWarp>
class ThreeVolumeTraversalEngine<TVoxel, TWarp, VoxelBlockHash, MEMORYDEVICE_CUDA> {
	/**
	 * \brief Concurrent traversal of three volumes with potentially different voxel types
	 * \details All volumes must be indexed with hash tables of the same size
	 */
public:
// region ================================ DYNAMIC THREE-SCENE TRAVERSAL ===============================================

	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			VoxelVolume<TVoxel, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxel, VoxelBlockHash>* secondaryScene,
			VoxelVolume<TWarp, VoxelBlockHash>* warpField,
			TFunctor& functor) {
// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int hashEntryCount = warpField->index.hashEntryCount;

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize_HashPerBlock(hashEntryCount);

		dualVoxelWarpPositionTraversal_device<TFunctor, TVoxel, TVoxel, TWarp>
				<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> >
		                                    (primaryVoxels, secondaryVoxels, warpVoxels,
				                                    primaryHashTable, secondaryHashTable, warpHashTable, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}
// endregion ===========================================================================================================
};

} // namespace ITMLib