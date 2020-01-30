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
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Objects/Scene/PlainVoxelArray.h"
#include "../../../Utils/Configuration.h"
#include "VolumeTraversal_CUDA_PlainVoxelArray_Kernels.h"

namespace ITMLib {


//TODO: many DRY violations within this file -- figure out how to reduce them

template<typename TVoxel>
class VolumeTraversalEngine<TVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> {
public:
// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void StaticVoxelTraversal(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene) {
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
	VoxelTraversal(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, TFunctor& functor) {
		TVoxel* voxelArray = scene->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* arrayInfo = scene->index.GetIndexData();

		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(scene->index.GetVolumeSize().x / cudaBlockSize.x,
		              scene->index.GetVolumeSize().y / cudaBlockSize.y,
		              scene->index.GetVolumeSize().z / cudaBlockSize.z);

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
	VoxelPositionTraversal(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, TFunctor& functor) {
		TVoxel* voxelArray = scene->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* arrayInfo = scene->index.GetIndexData();

		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(scene->index.GetVolumeSize().x / cudaBlockSize.x,
		              scene->index.GetVolumeSize().y / cudaBlockSize.y,
		              scene->index.GetVolumeSize().z / cudaBlockSize.z);

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

template<typename TVoxelPrimary, typename TVoxelSecondary>
class TwoVolumeTraversalEngine<TVoxelPrimary, TVoxelSecondary, PlainVoxelArray, PlainVoxelArray, MEMORYDEVICE_CUDA> {
private:
	template<typename TBooleanFunctor, typename TDeviceTraversalFunction>
	inline static bool
	DualVoxelTraversal_AllTrue_Generic(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
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
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene) {

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
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
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
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
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
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
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
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
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


template<typename TVoxel, typename TWarp>
class ThreeVolumeTraversalEngine<TVoxel, TWarp, PlainVoxelArray, MEMORYDEVICE_CUDA> {
	/**
	 * \brief Concurrent traversal of 2 scenes with the same voxel type and a warp field
	 * \details All scenes must have matching dimensions
	 */
public:
// region ================================ STATIC TWO-SCENE TRAVERSAL WITH WARPS =======================================

	template<typename TStaticFunctor>
	inline static void
	StaticDualVoxelTraversal(
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* secondaryScene,
			ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField) {
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
// region ================================ DYNAMIC TWO-SCENE TRAVERSAL WITH WARPS ======================================



	template<typename TFunctor>
	inline static void
	DualVoxelTraversal(
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* secondaryScene,
			ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
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
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* secondaryScene,
			ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
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

}// namespace ITMLib