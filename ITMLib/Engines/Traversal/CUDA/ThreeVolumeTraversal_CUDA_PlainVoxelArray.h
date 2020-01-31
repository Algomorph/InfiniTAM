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
#include "ThreeVolumeTraversal_CUDA_PlainVoxelArray_Kernels.h"

namespace ITMLib{

template<typename TVoxel, typename TWarp>
class ThreeVolumeTraversalEngine<TVoxel, TWarp, PlainVoxelArray, MEMORYDEVICE_CUDA> {
	/**
	 * \brief Concurrent traversal of three volumes with potentially different voxel types
	 * \details All volumes must have matching dimensions
	 */
public:
// region ================================ STATIC TRHEE-SCENE TRAVERSAL ================================================

	template<typename TStaticFunctor>
	inline static void
	StaticDualVoxelTraversal(
			VoxelVolume<TVoxel, PlainVoxelArray>* primaryScene,
			VoxelVolume<TVoxel, PlainVoxelArray>* secondaryScene,
			VoxelVolume<TWarp, PlainVoxelArray>* warpField) {
		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize() &&
		       primaryScene->index.GetVolumeSize() == warpField->index.GetVolumeSize());
// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();

		const PlainVoxelArray::GridAlignedBox* arrayInfo = primaryScene->index.GetIndexData();

		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().x) / cudaBlockSize.x)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().y) / cudaBlockSize.y)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().z) / cudaBlockSize.z))
		);

		staticDualVoxelWarpTraversal_device<TStaticFunctor, TVoxel>
		<< < gridSize, cudaBlockSize >> >
		(primaryVoxels, secondaryVoxels, warpVoxels, arrayInfo);
		ORcudaKernelCheck;
	}
// endregion
// region ================================ DYNAMIC THREE-SCENE TRAVERSAL ===============================================

	template<typename TFunctor>
	inline static void
	DualVoxelTraversal(
			VoxelVolume<TVoxel, PlainVoxelArray>* primaryScene,
			VoxelVolume<TVoxel, PlainVoxelArray>* secondaryScene,
			VoxelVolume<TWarp, PlainVoxelArray>* warpField,
			TFunctor& functor) {

		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize() &&
		       primaryScene->index.GetVolumeSize() == warpField->index.GetVolumeSize());
// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();

		const PlainVoxelArray::GridAlignedBox* arrayInfo = primaryScene->index.GetIndexData();

		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().x) / cudaBlockSize.x)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().y) / cudaBlockSize.y)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().z) / cudaBlockSize.z))
		);

		dualVoxelWarpTraversal_device<TFunctor, TVoxel>
				<< < gridSize, cudaBlockSize >> >
		                       (primaryVoxels, secondaryVoxels, warpVoxels, arrayInfo, functor);
		ORcudaKernelCheck;
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			VoxelVolume<TVoxel, PlainVoxelArray>* primaryScene,
			VoxelVolume<TVoxel, PlainVoxelArray>* secondaryScene,
			VoxelVolume<TWarp, PlainVoxelArray>* warpField,
			TFunctor& functor) {

		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize() &&
		       primaryScene->index.GetVolumeSize() == warpField->index.GetVolumeSize());

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();

		const PlainVoxelArray::GridAlignedBox* arrayInfo = primaryScene->index.GetIndexData();

		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().x) / cudaBlockSize.x)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().y) / cudaBlockSize.y)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().z) / cudaBlockSize.z))
		);

		dualVoxelWarpPositionTraversal_device<TFunctor, TVoxel>
				<< < gridSize, cudaBlockSize >> >
		                       (primaryVoxels, secondaryVoxels, warpVoxels, arrayInfo, functor_device);
		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
		ORcudaKernelCheck;
	}
// endregion ===========================================================================================================
};


} // namespace ITMLib

