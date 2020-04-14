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
	inline static void TraverseAll(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TVoxel* voxels = volume->voxels.GetVoxelBlocks();
		const GridAlignedBox* array_info = volume->index.GetIndexData();

		dim3 cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 cuda_grid_size(volume->index.GetVolumeSize().x / cuda_block_size.x,
		              volume->index.GetVolumeSize().y / cuda_block_size.y,
		              volume->index.GetVolumeSize().z / cuda_block_size.z);

		TraverseAll_device<TStaticFunctor, TVoxel> <<< cuda_grid_size, cuda_block_size >>> (voxels, array_info);
		ORcudaKernelCheck;
	}

	template<typename TStaticFunctor>
	inline static void TraverseUtilized(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TraverseAll<TStaticFunctor>(volume);
	}

// endregion ===========================================================================================================
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================
	template<typename TFunctor>
	inline static void
	TraverseAll(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TVoxel* voxels = volume->voxels.GetVoxelBlocks();
		const GridAlignedBox* array_info = volume->index.GetIndexData();

		dim3 cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 cuda_grid_size(volume->index.GetVolumeSize().x / cuda_block_size.x,
		              volume->index.GetVolumeSize().y / cuda_block_size.y,
		              volume->index.GetVolumeSize().z / cuda_block_size.z);

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		traverseAll_device < TFunctor, TVoxel > <<< cuda_grid_size, cuda_block_size >>>
		                                                    (voxels, array_info, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAll(volume, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TVoxel* voxels = volume->voxels.GetVoxelBlocks();
		const GridAlignedBox* array_info = volume->index.GetIndexData();

		dim3 cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 cuda_grid_size(volume->index.GetVolumeSize().x / cuda_block_size.x,
		              volume->index.GetVolumeSize().y / cuda_block_size.y,
		              volume->index.GetVolumeSize().z / cuda_block_size.z);

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		traverseAllWithPosition_device < TFunctor, TVoxel > <<< cuda_grid_size, cuda_block_size >>>
		                                                                (voxels, array_info, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAllWithPosition(volume, functor);
	}
// endregion ===========================================================================================================
};


}// namespace ITMLib