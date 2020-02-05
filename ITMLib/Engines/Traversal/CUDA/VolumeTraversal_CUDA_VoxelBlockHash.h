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
	VoxelTraversal_Generic(VoxelVolume<TVoxel, VoxelBlockHash>* scene, TFunctor& functor, TDeviceFunction&& deviceFunction) {
		TVoxel* voxelArray = scene->localVBA.GetVoxelBlocks();
		const HashEntry* hashTable = scene->index.GetIndexData();
		int hashEntryCount = scene->index.hashEntryCount;

		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize_HashPerBlock(hashEntryCount);

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		std::forward<TDeviceFunction>(deviceFunction)(gridSize_HashPerBlock,
				cudaBlockSize_BlockVoxelPerThread,voxelArray, hashTable, functor_device);

		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}
public:
// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void StaticVoxelTraversal(VoxelVolume<TVoxel, VoxelBlockHash>* scene) {
		TVoxel* voxelArray = scene->localVBA.GetVoxelBlocks();
		const HashEntry* hashTable = scene->index.GetIndexData();
		int hashEntryCount = scene->index.hashEntryCount;

		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize_HashPerBlock(hashEntryCount);

		staticVoxelTraversal_device<TStaticFunctor, TVoxel>
				<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> > (voxelArray, hashTable);
		ORcudaKernelCheck;
	}

// endregion ===========================================================================================================
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================
	template<typename TFunctor>
	inline static void
	VoxelTraversal(VoxelVolume<TVoxel, VoxelBlockHash>* scene, TFunctor& functor) {
		VoxelTraversal_Generic(scene, functor, [](dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread,
		                                          TVoxel* voxelArray, const HashEntry* hashTable, TFunctor* functor_device){
			voxelTraversal_device<TFunctor, TVoxel>
					<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> >
			                                    (voxelArray, hashTable, functor_device);
		});
	}
	template<typename TFunctor>
	inline static void
	VoxelPositionTraversal(VoxelVolume<TVoxel, VoxelBlockHash>* scene, TFunctor& functor) {
		VoxelTraversal_Generic(scene, functor, [](dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread,
		                                          TVoxel* voxelArray, const HashEntry* hashTable, TFunctor* functor_device){
			voxelPositionTraversal_device<TFunctor, TVoxel>
					<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> >
			                                    (voxelArray, hashTable, functor_device);
		});
	}
	template<typename TFunctor>
	inline static void
	VoxelAndHashBlockPositionTraversal(VoxelVolume<TVoxel, VoxelBlockHash>* scene, TFunctor& functor) {
		VoxelTraversal_Generic(scene, functor, [](dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread,
		                                          TVoxel* voxelArray, const HashEntry* hashTable, TFunctor* functor_device){
			voxelAndHashBlockPositionTraversal_device<TFunctor, TVoxel>
					<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> >
			                                    (voxelArray, hashTable, functor_device);
		});
	}
// endregion ===========================================================================================================
};

} // namespace ITMLib



