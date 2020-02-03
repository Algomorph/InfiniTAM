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

//stdlib
#include <cassert>

//local
#include "../Interface/VolumeTraversal.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"
#include "../../../Utils/Configuration.h"
#include "VolumeTraversal_CUDA_PlainVoxelArray_Kernels.h"

namespace ITMLib {


//TODO: many DRY violations within this file -- figure out how to reduce them

template<typename TVoxel>
class VolumeTraversalEngine<TVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> {
public:
// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void StaticVoxelTraversal(VoxelVolume<TVoxel, PlainVoxelArray>* scene) {
		TVoxel* voxelArray = scene->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* arrayInfo = scene->index.GetIndexData();

		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(scene->index.GetVolumeSize().x / cudaBlockSize.x,
		              scene->index.GetVolumeSize().y / cudaBlockSize.y,
		              scene->index.GetVolumeSize().z / cudaBlockSize.z);

		staticVoxelTraversal_device<TStaticFunctor, TVoxel> << < gridSize, cudaBlockSize >> > (voxelArray, arrayInfo);
		ORcudaKernelCheck;
	}

// endregion ===========================================================================================================
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================
	template<typename TFunctor>
	inline static void
	VoxelTraversal(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TVoxel* voxelArray = volume->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* arrayInfo = volume->index.GetIndexData();

		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(volume->index.GetVolumeSize().x / cudaBlockSize.x,
		              volume->index.GetVolumeSize().y / cudaBlockSize.y,
		              volume->index.GetVolumeSize().z / cudaBlockSize.z);

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		voxelTraversal_device<TFunctor, TVoxel> << < gridSize, cudaBlockSize >> >
		                                                       (voxelArray, arrayInfo, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TFunctor>
	inline static void
	VoxelPositionTraversal(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TVoxel* voxelArray = volume->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* arrayInfo = volume->index.GetIndexData();

		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(volume->index.GetVolumeSize().x / cudaBlockSize.x,
		              volume->index.GetVolumeSize().y / cudaBlockSize.y,
		              volume->index.GetVolumeSize().z / cudaBlockSize.z);

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		voxelPositionTraversal_device<TFunctor, TVoxel> << < gridSize, cudaBlockSize >> >
		                                                       (voxelArray, arrayInfo, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}
// endregion ===========================================================================================================
};


}// namespace ITMLib