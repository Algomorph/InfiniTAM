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

namespace ITMLib {

template<typename TVoxel1, typename TVoxel2, typename TVoxel3>
class ThreeVolumeTraversalEngine<TVoxel1, TVoxel2, TVoxel3, VoxelBlockHash, MEMORYDEVICE_CUDA> {
	/**
	 * \brief Concurrent traversal of three volumes with potentially different voxel types
	 * \details All volumes must be indexed with hash tables of the same size
	 */
private:
	template<typename TProcessFunction, typename TFunctor>
	inline static void
	TraverseAll_Generic(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
			TProcessFunction&& process_function, TFunctor& functor) {
// *** traversal vars
		TVoxel1* voxels1 = volume1->GetVoxels();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		TVoxel2* voxels2 = volume2->GetVoxels();
		HashEntry* hash_table2 = volume2->index.GetEntries();
		TVoxel3* voxels3 = volume3->GetVoxels();
		HashEntry* hash_table3 = volume3->index.GetEntries();

		const int hash_entry_count = volume3->index.hash_entry_count;

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		dim3 voxel_per_thread_cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 hash_per_block_cuda_grid_size(hash_entry_count);

		std::forward<TProcessFunction>(process_function)(
				hash_per_block_cuda_grid_size, voxel_per_thread_cuda_block_size,
				voxels1, voxels2, voxels3, hash_table1, hash_table2, hash_table3,
				functor_device);

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TProcessFunction, typename TFunctor>
	inline static void
	TraverseUtilized_Generic(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
			TProcessFunction&& process_function, TFunctor& functor) {
// *** traversal vars
		TVoxel1* voxels1 = volume1->GetVoxels();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		TVoxel2* voxels2 = volume2->GetVoxels();
		HashEntry* hash_table2 = volume2->index.GetEntries();
		TVoxel3* voxels3 = volume3->GetVoxels();
		HashEntry* hash_table3 = volume3->index.GetEntries();

		const int utilized_entry_count = volume1->index.GetUtilizedBlockCount();
		const int* utilized_entry_codes = volume1->index.GetUtilizedBlockHashCodes();

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		dim3 voxel_per_thread_cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 hash_per_block_cuda_grid_size(utilized_entry_count);

		std::forward<TProcessFunction>(process_function)(
				hash_per_block_cuda_grid_size, voxel_per_thread_cuda_block_size,
				voxels1, voxels2, voxels3, hash_table1, hash_table2, hash_table3,
				utilized_entry_codes, functor_device);

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

public:
// region ================================ DYNAMIC THREE-SCENE TRAVERSAL ===============================================

	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
			TFunctor& functor) {
		TraverseAll_Generic(
				volume1, volume2, volume3,
				[](dim3 hash_per_block_cuda_grid_size, dim3 voxel_per_thread_cuda_block_size,
				   TVoxel1* voxels1, TVoxel2* voxels2, TVoxel3* voxels3,
				   HashEntry* hash_table1, HashEntry* hash_table2, HashEntry* hash_table3,
				   TFunctor* functor_device) {
					traverseAllWithPosition_device<TFunctor, TVoxel1, TVoxel2, TVoxel3>
							<<< hash_per_block_cuda_grid_size, voxel_per_thread_cuda_block_size >>>
					                                            (voxels1, voxels2, voxels3,
							                                            hash_table1, hash_table2, hash_table3, functor_device);
					ORcudaKernelCheck;
				},
				functor
		);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
			TFunctor& functor) {
		TraverseUtilized_Generic(
				volume1, volume2, volume3,
				[](dim3 hash_per_block_cuda_grid_size, dim3 voxel_per_thread_cuda_block_size,
				   TVoxel1* voxels1, TVoxel2* voxels2, TVoxel3* voxels3,
				   HashEntry* hash_table1, HashEntry* hash_table2, HashEntry* hash_table3,
				   const int* utilized_hash_codes, TFunctor* functor_device) {
					traverseUtilizedWithPosition_device<TFunctor, TVoxel1, TVoxel2, TVoxel3>
							<<< hash_per_block_cuda_grid_size, voxel_per_thread_cuda_block_size >>>
					                                            (voxels1, voxels2, voxels3,
							                                            hash_table1, hash_table2, hash_table3,
							                                            utilized_hash_codes, functor_device);
					ORcudaKernelCheck;
				},
				functor
		);
	}
// endregion ===========================================================================================================
};

} // namespace ITMLib