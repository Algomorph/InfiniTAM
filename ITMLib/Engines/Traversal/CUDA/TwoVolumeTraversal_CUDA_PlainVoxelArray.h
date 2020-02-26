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

namespace ITMLib {

template<typename TVoxelPrimary, typename TVoxelSecondary>
class TwoVolumeTraversalEngine<TVoxelPrimary, TVoxelSecondary, PlainVoxelArray, PlainVoxelArray, MEMORYDEVICE_CUDA> {
private:
	template<typename TBooleanFunctor, typename TDeviceTraversalFunction>
	inline static bool
	TraverseAndCompareAll_Generic(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* volume2,
			TBooleanFunctor& functor, TDeviceTraversalFunction&& comparison_function) {

		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());

		// allocate boolean varaible for answer
		bool* mismatch_encountered_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &mismatch_encountered_device, sizeof(bool)));
		ORcudaSafeCall(cudaMemset(mismatch_encountered_device, 0, sizeof(bool)));

		// transfer functor from RAM to VRAM
		TBooleanFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TBooleanFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TBooleanFunctor), cudaMemcpyHostToDevice));


		// perform traversal on the CUDA
		TVoxelPrimary* voxels1 = volume1->voxels.GetVoxelBlocks();
		TVoxelSecondary* voxels2 = volume2->voxels.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* array_info = volume1->index.GetIndexData();
		dim3 cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 cuda_grid_size(
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().x) / cuda_block_size.x)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().y) / cuda_block_size.y)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().z) / cuda_block_size.z))
		);
		std::forward<TDeviceTraversalFunction>(comparison_function)(cuda_grid_size, cuda_block_size, voxels1,
		                                                            voxels2, array_info, functor_device,
		                                                            mismatch_encountered_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TBooleanFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));

		bool mismatch_encountered;
		ORcudaSafeCall(
				cudaMemcpy(&mismatch_encountered, mismatch_encountered_device, sizeof(bool), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(mismatch_encountered_device));

		return !mismatch_encountered;
	}

public:
// region ============================== DUAL-SCENE STATIC TRAVERSAL ===================================================

	template<typename TStaticBooleanFunctor>
	inline static bool TraverseAndCompareAll(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* volume2) {

		bool* mismatch_encountered_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &mismatch_encountered_device, sizeof(bool)));
		ORcudaSafeCall(cudaMemset(mismatch_encountered_device, 0, sizeof(bool)));


		TVoxelPrimary* voxels1 = volume1->voxels.GetVoxelBlocks();
		TVoxelSecondary* voxels2 = volume2->voxels.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* array_info = volume1->index.GetIndexData();
		dim3 cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 cuda_grid_size(
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().x) / cuda_block_size.x)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().y) / cuda_block_size.y)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().z) / cuda_block_size.z))
		);

		TraverseAndCompareAll_device<TStaticBooleanFunctor, TVoxelPrimary, TVoxelSecondary>
				<< < cuda_grid_size, cuda_block_size >> >
		                             (voxels1, voxels2, array_info, mismatch_encountered_device);
		ORcudaKernelCheck;

		bool mismatch_encountered;
		ORcudaSafeCall(
				cudaMemcpy(&mismatch_encountered, mismatch_encountered_device, sizeof(bool), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(mismatch_encountered_device));

		return !mismatch_encountered;
	}

// endregion
// region ============================== DUAL-SCENE DYNAMIC TRAVERSAL ==================================================

	template<typename TFunctor>
	inline static void
	TraverseAll(VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
	            VoxelVolume<TVoxelSecondary, PlainVoxelArray>* volume2,
	            TFunctor& functor) {

		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		// perform traversal on the CUDA
		TVoxelPrimary* voxels1 = volume1->voxels.GetVoxelBlocks();
		TVoxelSecondary* voxels2 = volume2->voxels.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* array_info = volume1->index.GetIndexData();
		dim3 cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 cuda_grid_size(
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().x) / cuda_block_size.x)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().y) / cuda_block_size.y)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().z) / cuda_block_size.z))
		);

		dualVoxelTraversal_device<TFunctor, TVoxelPrimary, TVoxelSecondary>
				<< < cuda_grid_size, cuda_block_size >> >
		                             (voxels1, voxels2, array_info, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
	            VoxelVolume<TVoxelSecondary, PlainVoxelArray>* volume2,
	            TFunctor& functor) {
		TraverseAll(volume1, volume2, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
	                        VoxelVolume<TVoxelSecondary, PlainVoxelArray>* volume2,
	                        TFunctor& functor) {

		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		// perform traversal on the CUDA
		TVoxelPrimary* voxels1 = volume1->voxels.GetVoxelBlocks();
		TVoxelSecondary* voxels2 = volume2->voxels.GetVoxelBlocks();
		const PlainVoxelArray::GridAlignedBox* array_info = volume1->index.GetIndexData();
		dim3 cuda_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 cuda_cuda_grid_size(
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().x) / cuda_block_size.x)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().y) / cuda_block_size.y)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().z) / cuda_block_size.z))
		);
		dualVoxelPositionTraversal_device<TFunctor, TVoxelPrimary, TVoxelSecondary>
				<< < cuda_cuda_grid_size, cuda_block_size >> >
		                                  (voxels1, voxels2, array_info, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));

	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
	                 VoxelVolume<TVoxelSecondary, PlainVoxelArray>* volume2,
	                 TFunctor& functor) {
		TraverseAllWithPosition(volume1, volume2, functor);
	}

	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareAll(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* volume2,
			TBooleanFunctor& functor) {
		return TraverseAndCompareAll_Generic(
				volume1, volume2, functor,
				[&](const dim3& cuda_grid_size, const dim3& cuda_block_size,
				    TVoxelPrimary* voxels1, TVoxelSecondary* voxels2,
				    const PlainVoxelArray::GridAlignedBox* array_info,
				    TBooleanFunctor* functor_device, bool* mismatch_encountered_device) {
					TraverseAndCompareAll_device<TBooleanFunctor, TVoxelPrimary, TVoxelSecondary>
							<< < cuda_grid_size, cuda_block_size >> >
					                             (voxels1, voxels2, array_info, functor_device, mismatch_encountered_device);
				}
		);
	}

	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareAllWithPosition(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* volume2,
			TBooleanFunctor& functor) {
		return TraverseAndCompareAll_Generic(
				volume1, volume2, functor,
				[&](const dim3& cuda_grid_size, const dim3& cuda_block_size,
				    TVoxelPrimary* voxels1, TVoxelSecondary* voxels2,
				    const PlainVoxelArray::GridAlignedBox* array_info,
				    TBooleanFunctor* functor_device, bool* mismatch_encountered_device) {
					TraverseAndCompareAllWithPosition_device<TBooleanFunctor, TVoxelPrimary, TVoxelSecondary>
							<< < cuda_grid_size, cuda_block_size >> >
					                             (voxels1, voxels2, array_info, functor_device, mismatch_encountered_device);
				}
		);
	}


// endregion ===========================================================================================================

};

} // namespace ITMLib