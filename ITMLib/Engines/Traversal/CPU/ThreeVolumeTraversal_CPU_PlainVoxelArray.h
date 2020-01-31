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
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"

namespace ITMLib{

//======================================================================================================================
//                            CONTAINS TRAVERSAL METHODS FOR VOLUMES USING PlainVoxelArray FOR INDEXING
//                                  (THREE VOLUMES WITH DIFFERING VOXEL TYPES)
//======================================================================================================================
template<typename TVoxel, typename TWarp>
class ThreeVolumeTraversalEngine<TVoxel, TWarp, PlainVoxelArray, MEMORYDEVICE_CPU> {
	/**
	 * \brief Concurrent traversal of three volumes with potentially-differing voxel types
	 * \details All volumes must have matching dimensions
	 */
public:
// region ================================ STATIC TWO-SCENE TRAVERSAL WITH WARPS =======================================

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
		//asserted to be the same
		int voxelCount = primaryScene->index.GetVolumeSize().x * primaryScene->index.GetVolumeSize().y *
		                 primaryScene->index.GetVolumeSize().z;


#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			TVoxel& primaryVoxel = primaryVoxels[linearIndex];
			TVoxel& secondaryVoxel = secondaryVoxels[linearIndex];
			TWarp& warpVoxel = warpVoxels[linearIndex];
			TStaticFunctor::run(primaryVoxel, secondaryVoxel, warpVoxel);
		}
	}
// endregion
// region ================================ DYNAMIC TWO-SCENE TRAVERSAL WITH WARPS ======================================



	template<typename TFunctor>
	inline static void
	DualVoxelTraversal(
			VoxelVolume<TVoxel, PlainVoxelArray>* primaryScene,
			VoxelVolume<TVoxel, PlainVoxelArray>* secondaryScene,
			VoxelVolume<TWarp, PlainVoxelArray>* warpField,
			TFunctor& functor) {

		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());
// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryScene->index.GetVolumeSize().x * primaryScene->index.GetVolumeSize().y *
		                 primaryScene->index.GetVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			TVoxel& primaryVoxel = primaryVoxels[linearIndex];
			TVoxel& secondaryVoxel = secondaryVoxels[linearIndex];
			TWarp& warpVoxel = warpVoxels[linearIndex];
			functor(primaryVoxel, secondaryVoxel, warpVoxel);
		}
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
// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryScene->index.GetVolumeSize().x * primaryScene->index.GetVolumeSize().y *
		                 primaryScene->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = primaryScene->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
			TVoxel& primaryVoxel = primaryVoxels[linearIndex];
			TVoxel& secondaryVoxel = secondaryVoxels[linearIndex];
			TWarp& warpVoxel = warpVoxels[linearIndex];
			functor(primaryVoxel, secondaryVoxel, warpVoxel, voxelPosition);
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal_SingleThreaded(
			VoxelVolume<TVoxel, PlainVoxelArray>* primaryScene,
			VoxelVolume<TVoxel, PlainVoxelArray>* secondaryScene,
			VoxelVolume<TWarp, PlainVoxelArray>* warpField,
			TFunctor& functor) {
		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize() &&
		       warpField->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());
		// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryScene->index.GetVolumeSize().x * primaryScene->index.GetVolumeSize().y *
		                 primaryScene->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = primaryScene->index.GetIndexData();
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
			TVoxel& primaryVoxel = primaryVoxels[linearIndex];
			TVoxel& secondaryVoxel = secondaryVoxels[linearIndex];
			TWarp& warpVoxel = warpVoxels[linearIndex];
			functor(primaryVoxel, secondaryVoxel, warpVoxel, voxelPosition);
		}
	}
// endregion ===========================================================================================================
};

} // namespace ITMLib

