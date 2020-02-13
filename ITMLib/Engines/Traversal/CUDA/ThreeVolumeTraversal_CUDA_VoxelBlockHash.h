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
#include "../../../Objects/Volume/VoxelVolume.h"

namespace ITMLib{

template<typename TVoxel1, typename TVoxel2, typename TVoxel3>
class ThreeVolumeTraversalEngine<TVoxel1, TVoxel2, TVoxel3, VoxelBlockHash, MEMORYDEVICE_CUDA> {
	/**
	 * \brief Concurrent traversal of three volumes with potentially different voxel types
	 * \details All volumes must be indexed with hash tables of the same size
	 */
public:
// region ================================ DYNAMIC THREE-SCENE TRAVERSAL ===============================================

	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
			TFunctor& functor) {
// *** traversal vars
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		HashEntry* hash_table1 = volume1->index.GetEntries();

		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		HashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel3* voxels3 = volume3->localVBA.GetVoxelBlocks();
		HashEntry* hash_table3 = volume3->index.GetEntries();

		int hash_entry_count = volume3->index.hashEntryCount;

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize_HashPerBlock(hash_entry_count);

		threeVolumeTraversalWithPosition_device<TFunctor, TVoxel1, TVoxel2, TVoxel3>
				<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> >
		                                    (voxels1, voxels2, voxels3,
				                                    hash_table1, hash_table2, hash_table3, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}
// endregion ===========================================================================================================
};

} // namespace ITMLib