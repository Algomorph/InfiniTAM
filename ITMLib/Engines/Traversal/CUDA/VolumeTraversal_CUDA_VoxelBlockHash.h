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
#include "VolumeTraversal_CUDA_VoxelBlockHash_Kernels.h"
#include "../Interface/VolumeTraversal.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"
#include "../../../Objects/Volume/VoxelVolume.h"

namespace ITMLib {

//Nota Bene: "STATIC" and "DYNAMIC" in region titles refer to the way functors are used, i.e.
// for "static" traversal, the template argument needs to have a static "run" function, and no actual functor object
// needs to be passed. For "dynamic" traversal, the functor is actually passed in, which can be critical for any
// kind of traversal where some state is updated in a thread-safe manner: this state can be stored as a part of the
// functor.

template<typename TVoxel>
class VolumeTraversalEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> {
private:
	template<typename TFunctor, typename TDeviceFunction>
	inline static void
	TraverseAll_Generic(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor,
	                    TDeviceFunction&& deviceFunction) {
		TVoxel* voxels = volume->GetVoxels();
		const HashEntry* hash_table = volume->index.GetIndexData();
		const int hash_entry_count = volume->index.hash_entry_count;

		dim3 voxel_per_thread_cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 hash_per_block_cuda_grid_size(hash_entry_count);

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		std::forward<TDeviceFunction>(deviceFunction)(hash_per_block_cuda_grid_size,
		                                              voxel_per_thread_cuda_block_size, voxels, hash_table,
		                                              functor_device);

		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TFunctor, typename TDeviceFunction>
	inline static void
	TraverseUtilized_Generic(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor,
	                         TDeviceFunction&& deviceFunction) {
		TVoxel* voxels = volume->GetVoxels();
		const HashEntry* hash_table = volume->index.GetIndexData();
		const int utilized_block_count = volume->index.GetUtilizedBlockCount();
		const int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();

		dim3 voxel_per_thread_cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 hash_per_block_cuda_grid_size(utilized_block_count);

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		std::forward<TDeviceFunction>(deviceFunction)(hash_per_block_cuda_grid_size, voxel_per_thread_cuda_block_size,
		                                              voxels, hash_table, utilized_hash_codes, functor_device);

		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

public:
// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================

	template<typename TStaticFunctor>
	inline static void TraverseAll(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		TVoxel* voxels = volume->GetVoxels();
		const HashEntry* hash_table = volume->index.GetIndexData();
		int hash_entry_count = volume->index.hash_entry_count;

		dim3 voxel_per_thread_cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 hash_per_block_cuda_grid_size(hash_entry_count);

		traverseAll_StaticFunctor_device<TStaticFunctor, TVoxel><<<hash_per_block_cuda_grid_size, voxel_per_thread_cuda_block_size>>>(voxels, hash_table);		ORcudaKernelCheck;
	}

	template<typename TStaticFunctor>
	inline static void TraverseUtilized(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		TVoxel* voxels = volume->GetVoxels();
		const HashEntry* hash_table = volume->index.GetIndexData();
		const int utilized_block_count = volume->index.GetUtilizedBlockCount();
		const int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();

		dim3 voxel_per_thread_cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 hash_per_block_cuda_grid_size(utilized_block_count);

		traverseUtilized_StaticFunctor_device<TStaticFunctor, TVoxel><<<hash_per_block_cuda_grid_size, voxel_per_thread_cuda_block_size>>>(voxels, hash_table, utilized_hash_codes);
		ORcudaKernelCheck;
	}

// endregion ===========================================================================================================
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================
	template<typename TFunctor>
	inline static void
	TraverseAll(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor) {
		TraverseAll_Generic(volume, functor, [](dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread,
		                                        TVoxel* voxelArray, const HashEntry* hashTable,
		                                        TFunctor* functor_device) {
			traverseAll_device<TFunctor, TVoxel>
					<<< gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >>>
			                                    (voxelArray, hashTable, functor_device);
		});
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor) {
		TraverseUtilized_Generic(volume, functor, [](dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread,
		                                             TVoxel* voxels, const HashEntry* hash_table,
		                                             const int* utilized_hash_codes,
		                                             TFunctor* functor_device) {
			traverseUtilized_device<TFunctor, TVoxel>
					<<< gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >>>
			                                    (voxels, hash_table, utilized_hash_codes, functor_device);
		});
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor) {
		TraverseAll_Generic(volume, functor, [](dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread,
		                                        TVoxel* voxelArray, const HashEntry* hashTable,
		                                        TFunctor* functor_device) {
			traverseAllWithPosition_device<TFunctor, TVoxel>
					<<< gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >>>
			                                    (voxelArray, hashTable, functor_device);
		});
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor) {
		TraverseUtilized_Generic(volume, functor, [](dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread,
		                                             TVoxel* voxels, const HashEntry* hash_table,
		                                             const int* utilized_hash_codes, TFunctor* functor_device) {
			traverseUtilizedWithPosition_device<TFunctor, TVoxel>
					<<< gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >>>
			                                    (voxels, hash_table, utilized_hash_codes, functor_device);
		});
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithPositionAndBlockPosition(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor) {
		TraverseAll_Generic(volume, functor, [](dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread,
		                                        TVoxel* voxelArray, const HashEntry* hashTable,
		                                        TFunctor* functor_device) {
			traverseAllWithPositionAndBlockPosition_device<TFunctor, TVoxel>
					<<< gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >>>
			                                    (voxelArray, hashTable, functor_device);
		});
	}
// endregion ===========================================================================================================
};

} // namespace ITMLib



