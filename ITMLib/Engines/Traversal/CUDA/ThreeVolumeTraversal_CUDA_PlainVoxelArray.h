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
#include "../../../Objects/Volume/PlainVoxelArray.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../../ORUtils/MemoryDeviceType.h"
#include "ThreeVolumeTraversal_CUDA_PlainVoxelArray_Kernels.h"

namespace ITMLib {

template<typename TVoxel1, typename TVoxel2, typename TVoxel3>
class ThreeVolumeTraversalEngine<TVoxel1, TVoxel2, TVoxel3, PlainVoxelArray, MEMORYDEVICE_CUDA> {
	/**
	 * \brief Concurrent traversal of three volumes with potentially different voxel types
	 * \details All volumes must have matching dimensions
	 */
public:
// region ================================ STATIC TRHEE-SCENE TRAVERSAL ================================================

	template<typename TStaticFunctor>
	inline static void
	TraverseAll(VoxelVolume <TVoxel1, ITMLib::PlainVoxelArray>* volume1,
	            VoxelVolume <TVoxel2, ITMLib::PlainVoxelArray>* volume2,
	            VoxelVolume <TVoxel3, ITMLib::PlainVoxelArray>* volume3) {
		assert(volume2->index.GetVolumeSize() == volume3->index.GetVolumeSize() &&
		       volume2->index.GetVolumeSize() == volume1->index.GetVolumeSize());
// *** traversal vars
		TVoxel1* voxels1 = volume1->voxels.GetVoxelBlocks();
		TVoxel2* voxels2 = volume2->voxels.GetVoxelBlocks();
		TVoxel3* voxels3 = volume3->voxels.GetVoxelBlocks();

		const GridAlignedBox* arrayInfo = volume1->index.GetIndexData();

		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().x) / cudaBlockSize.x)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().y) / cudaBlockSize.y)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().z) / cudaBlockSize.z))
		);

		staticThreeVolumeTraversal_device < TStaticFunctor, TVoxel1, TVoxel2, TVoxel3 >
		<<< gridSize, cudaBlockSize >>>
		(voxels1, voxels2, voxels3, arrayInfo);
		ORcudaKernelCheck;
	}
// endregion
// region ================================ DYNAMIC THREE-SCENE TRAVERSAL ===============================================

	template<typename TFunctor>
	inline static void
	TraverseAll(
			ITMLib::VoxelVolume<TVoxel1, ITMLib::PlainVoxelArray>* volume1,
			ITMLib::VoxelVolume<TVoxel2, ITMLib::PlainVoxelArray>* volume2,
			ITMLib::VoxelVolume<TVoxel3, ITMLib::PlainVoxelArray>* volume3,
			TFunctor& functor) {

		assert(volume2->index.GetVolumeSize() == volume3->index.GetVolumeSize() &&
		       volume2->index.GetVolumeSize() == volume1->index.GetVolumeSize());
// *** traversal vars
		TVoxel1* voxels1 = volume1->voxels.GetVoxelBlocks();
		TVoxel2* voxels2 = volume2->voxels.GetVoxelBlocks();
		TVoxel3* voxels3 = volume3->voxels.GetVoxelBlocks();

		const GridAlignedBox* arrayInfo = volume2->index.GetIndexData();

		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().x) / cudaBlockSize.x)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().y) / cudaBlockSize.y)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().z) / cudaBlockSize.z))
		);

		threeVolumeTraversal_device<TFunctor, TVoxel2>
				<<< gridSize, cudaBlockSize >>>
		                       (voxels1, voxels2, voxels3, arrayInfo, functor);
		ORcudaKernelCheck;
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(
			ITMLib::VoxelVolume<TVoxel1, ITMLib::PlainVoxelArray>* volume1,
			ITMLib::VoxelVolume<TVoxel2, ITMLib::PlainVoxelArray>* volume2,
			ITMLib::VoxelVolume<TVoxel3, ITMLib::PlainVoxelArray>* volume3,
			TFunctor& functor) {
		TraverseAll(volume1,volume2,volume3, functor);
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(
			ITMLib::VoxelVolume<TVoxel1, ITMLib::PlainVoxelArray>* volume1,
			ITMLib::VoxelVolume<TVoxel2, ITMLib::PlainVoxelArray>* volume2,
			ITMLib::VoxelVolume<TVoxel3, ITMLib::PlainVoxelArray>* volume3,
			TFunctor& functor) {

		assert(volume2->index.GetVolumeSize() == volume3->index.GetVolumeSize() &&
		       volume2->index.GetVolumeSize() == volume1->index.GetVolumeSize());

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

// *** traversal vars
		TVoxel1* voxels1 = volume1->voxels.GetVoxelBlocks();
		TVoxel2* voxels2 = volume2->voxels.GetVoxelBlocks();
		TVoxel3* voxels3 = volume3->voxels.GetVoxelBlocks();

		const GridAlignedBox* arrayInfo = volume2->index.GetIndexData();

		dim3 cudaBlockSize(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize(
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().x) / cudaBlockSize.x)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().y) / cudaBlockSize.y)),
				static_cast<int>(ceil(static_cast<float>(volume1->index.GetVolumeSize().z) / cudaBlockSize.z))
		);

		threeVolumeTraversalWithPosition_device<TFunctor, TVoxel1, TVoxel2, TVoxel3>
				<<< gridSize, cudaBlockSize >>>
		                       (voxels1, voxels2, voxels3, arrayInfo, functor_device);
		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
		ORcudaKernelCheck;
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(
			ITMLib::VoxelVolume<TVoxel1, ITMLib::PlainVoxelArray>* volume1,
			ITMLib::VoxelVolume<TVoxel2, ITMLib::PlainVoxelArray>* volume2,
			ITMLib::VoxelVolume<TVoxel3, ITMLib::PlainVoxelArray>* volume3,
			TFunctor& functor){
		TraverseAllWithPosition(volume1,volume2,volume3, functor);
	}
// endregion ===========================================================================================================
};


} // namespace ITMLib

