//  ================================================================
//  Created by Gregory Kramida on 7/26/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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
#include "../../../Utils/Configuration/Configuration.h"
#include "../Shared/CudaCallWrappers.cuh"
#include "VolumeTraversal_CUDA_PlainVoxelArray_Kernels.h"

namespace ITMLib {


//TODO: many DRY violations within this file -- figure out how to reduce them

template<typename TVoxel>
class VolumeTraversalEngine<TVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> {
private: // static functions
	// using functor as second template here because it does not auto-resolve and has to be manually specified
	template<typename TVoxel_Modifiers, typename TStaticFunctor, typename TVolume>
	inline static void TraverseAll_Generic(TVolume* volume) {
		TVoxel_Modifiers* voxels = volume->GetVoxels();
		const GridAlignedBox* array_info = volume->index.GetIndexData();

		dim3 cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 cuda_grid_size(volume->index.GetVolumeSize().x / cuda_block_size.x,
		                    volume->index.GetVolumeSize().y / cuda_block_size.y,
		                    volume->index.GetVolumeSize().z / cuda_block_size.z);

		TraverseAll_device<TStaticFunctor, TVoxel_Modifiers> <<< cuda_grid_size, cuda_block_size >>>(voxels, array_info);
		ORcudaKernelCheck;
	}

	template<typename TVoxel_Modifiers, typename TVolume, typename TFunctor>
	inline static void
	TraverseAll_Generic(TVolume* volume, TFunctor& functor) {
		TVoxel_Modifiers* voxels = volume->GetVoxels();
		const GridAlignedBox* array_info = volume->index.GetIndexData();
		const Vector3i volume_size = volume->index.GetVolumeSize();
		internal::CallCUDAonUploadedFunctor(
				functor,
				[&voxels, &array_info, &volume_size](TFunctor* functor_device) {
					dim3 cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
					dim3 cuda_grid_size(volume_size.x / cuda_block_size.x,
					                    volume_size.y / cuda_block_size.y,
					                    volume_size.z / cuda_block_size.z);
					traverseAll_device<TFunctor, TVoxel_Modifiers> <<< cuda_grid_size, cuda_block_size >>>(voxels, array_info, functor_device);
				}
		);
	}

	template<typename TVoxel_Modifiers, typename TVolume, typename TFunctor>
	inline static void
	TraverseAllWithPosition_Generic(TVolume* volume, TFunctor& functor) {
		TVoxel_Modifiers* voxels = volume->GetVoxels();
		const GridAlignedBox* array_info = volume->index.GetIndexData();
		const Vector3i volume_size = volume->index.GetVolumeSize();
		internal::CallCUDAonUploadedFunctor(
				functor,
				[&voxels, &array_info, &volume_size](TFunctor* functor_device){
					dim3 cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
					dim3 cuda_grid_size(volume_size.x / cuda_block_size.x,
					                    volume_size.y / cuda_block_size.y,
					                    volume_size.z / cuda_block_size.z);
					traverseAllWithPosition_device<TFunctor, TVoxel_Modifiers> <<< cuda_grid_size, cuda_block_size >>>(voxels, array_info, functor_device);
				}
		);
	}

public: // static functions
// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void TraverseAll(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TraverseAll_Generic<TVoxel, TStaticFunctor>(volume);
	}

	template<typename TStaticFunctor>
	inline static void TraverseAll(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TraverseAll_Generic<const TVoxel, TStaticFunctor>(volume);
	}

	template<typename TStaticFunctor>
	inline static void TraverseUtilized(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TraverseAll<TStaticFunctor>(volume);
	}

	template<typename TStaticFunctor>
	inline static void TraverseUtilized(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TraverseAll<TStaticFunctor>(volume);
	}

// endregion ===========================================================================================================
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================
	template<typename TFunctor>
	inline static void
	TraverseAll(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAll_Generic<TVoxel>(volume, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseAll(const VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAll_Generic<const TVoxel>(volume, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAll(volume, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(const VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAll(volume, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAllWithPosition_Generic<TVoxel>(volume, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(const VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAllWithPosition_Generic<const TVoxel>(volume, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAllWithPosition(volume, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(const VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAllWithPosition(volume, functor);
	}
// endregion ===========================================================================================================
};


}// namespace ITMLib