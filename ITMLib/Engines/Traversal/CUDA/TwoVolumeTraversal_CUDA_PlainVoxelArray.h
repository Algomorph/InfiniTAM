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
#include "../Interface/TwoVolumeTraversal.h"
#include "TwoVolumeTraversal_CUDA_PlainVoxelArray_Kernels.h"

namespace ITMLib{

template<typename TVoxelPrimary, typename TVoxelSecondary>
class TwoVolumeTraversalEngine<TVoxelPrimary, TVoxelSecondary, PlainVoxelArray, PlainVoxelArray, MEMORYDEVICE_CUDA> {
private:
	template<typename TBooleanFunctor, typename TDeviceTraversalFunction>
	inline static bool
	DualVoxelTraversal_AllTrue_Generic(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
			TBooleanFunctor& functor, TDeviceTraversalFunction&& deviceTraversalFunction) {

		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());

		// allocate boolean varaible for answer
		bool* falseEncountered_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &falseEncountered_device, sizeof(bool)));
		ORcudaSafeCall(cudaMemset(falseEncountered_device, 0, sizeof(bool)));

		// transfer functor from RAM to VRAM
		TBooleanFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TBooleanFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TBooleanFunctor), cudaMemcpyHostToDevice));


		// perform traversal on the CUDA
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* arrayInfo = primaryScene->index.GetIndexData();
		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().x) / cudaBlockSize.x)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().y) / cudaBlockSize.y)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().z) / cudaBlockSize.z))
		);
		std::forward<TDeviceTraversalFunction>(deviceTraversalFunction)(gridSize, cudaBlockSize, primaryVoxels,
		                                                                secondaryVoxels, arrayInfo, functor_device,
		                                                                falseEncountered_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TBooleanFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));

		bool falseEncountered;
		ORcudaSafeCall(cudaMemcpy(&falseEncountered, falseEncountered_device, sizeof(bool), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(falseEncountered_device));

		return !falseEncountered;
	}

public:
// region ============================== DUAL-SCENE STATIC TRAVERSAL ===================================================

	template<typename TStaticBooleanFunctor>
	inline static bool StaticDualVoxelTraversal_AllTrue(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene) {

		bool* falseEncountered_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &falseEncountered_device, sizeof(bool)));
		ORcudaSafeCall(cudaMemset(falseEncountered_device, 0, sizeof(bool)));


		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* arrayInfo = primaryScene->index.GetIndexData();
		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().x) / cudaBlockSize.x)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().y) / cudaBlockSize.y)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().z) / cudaBlockSize.z))
		);

		staticDualVoxelTraversal_AllTrue_device<TStaticBooleanFunctor, TVoxelPrimary, TVoxelSecondary>
		<< < gridSize, cudaBlockSize >> >
		(primaryVoxels, secondaryVoxels, arrayInfo, falseEncountered_device);
		ORcudaKernelCheck;

		bool falseEncountered;
		ORcudaSafeCall(cudaMemcpy(&falseEncountered, falseEncountered_device, sizeof(bool), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(falseEncountered_device));

		return !falseEncountered;
	}

// endregion
// region ============================== DUAL-SCENE DYNAMIC TRAVERSAL ==================================================

	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
			TFunctor& functor) {

		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		// perform traversal on the CUDA
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* arrayInfo = primaryScene->index.GetIndexData();
		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().x) / cudaBlockSize.x)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().y) / cudaBlockSize.y)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().z) / cudaBlockSize.z))
		);
		dualVoxelPositionTraversal_device<TFunctor, TVoxelPrimary, TVoxelSecondary>
		<< < gridSize, cudaBlockSize >> >
		(primaryVoxels, secondaryVoxels, arrayInfo, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));

	}

	template<typename TBooleanFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
			TBooleanFunctor& functor) {
		return DualVoxelTraversal_AllTrue_Generic(
				primaryScene, secondaryScene, functor,
				[&](const dim3& gridSize, const dim3& cudaBlockSize,
				    TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
				    const PlainVoxelArray::GridAlignedBox* arrayInfo,
				    TBooleanFunctor* functor_device, bool* falseEncountered_device) {
					dualVoxelTraversal_AllTrue_device<TBooleanFunctor, TVoxelPrimary, TVoxelSecondary>
					<< < gridSize, cudaBlockSize >> >
					(primaryVoxels, secondaryVoxels, arrayInfo, functor_device, falseEncountered_device);
				}
		);
	}

	template<typename TBooleanFunctor>
	inline static bool
	DualVoxelPositionTraversal_AllTrue(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
			TBooleanFunctor& functor) {
		return DualVoxelTraversal_AllTrue_Generic(
				primaryScene, secondaryScene, functor,
				[&](const dim3& gridSize, const dim3& cudaBlockSize,
				    TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
				    const PlainVoxelArray::GridAlignedBox* arrayInfo,
				    TBooleanFunctor* functor_device, bool* falseEncountered_device) {
					dualVoxelPositionTraversal_AllTrue_device<TBooleanFunctor, TVoxelPrimary, TVoxelSecondary>
					<< < gridSize, cudaBlockSize >> >
					(primaryVoxels, secondaryVoxels, arrayInfo, functor_device, falseEncountered_device);
				}
		);
	}

	template<typename TFunctor>
	inline static void
	DualVoxelTraversal(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
			TFunctor& functor) {

		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		// perform traversal on the CUDA
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* arrayInfo = primaryScene->index.GetIndexData();
		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().x) / cudaBlockSize.x)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().y) / cudaBlockSize.y)),
				static_cast<int>(ceil(static_cast<float>(primaryScene->index.GetVolumeSize().z) / cudaBlockSize.z))
		);

		dualVoxelTraversal_device<TFunctor, TVoxelPrimary, TVoxelSecondary>
		<< < gridSize, cudaBlockSize >> >
		(primaryVoxels, secondaryVoxels, arrayInfo, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

// endregion ===========================================================================================================

};

} // namespace ITMLib