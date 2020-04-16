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

namespace ITMLib {

//======================================================================================================================
//                            CONTAINS TRAVERSAL METHODS FOR VOLUMES USING PlainVoxelArray FOR INDEXING
//                                  (THREE VOLUMES WITH DIFFERING VOXEL TYPES)
//======================================================================================================================
template<typename TVoxel1, typename TVoxel2, typename TVoxel3>
class ThreeVolumeTraversalEngine<TVoxel1, TVoxel2, TVoxel3, PlainVoxelArray, MEMORYDEVICE_CPU> {
	/**
	 * \brief Concurrent traversal of three volumes with potentially-differing voxel types
	 * \details All volumes must have matching dimensions
	 */
private:
	template<typename TProcessingFunction>
	inline static void TraverseAll_Generic(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			VoxelVolume<TVoxel3, PlainVoxelArray>* volume3,
			TProcessingFunction&& processing_function) {
		assert(volume2->index.GetVolumeSize() == volume3->index.GetVolumeSize() &&
		       volume2->index.GetVolumeSize() == volume1->index.GetVolumeSize());
// *** traversal vars
		TVoxel1* voxels1 = volume1->GetVoxelBlocks();
		TVoxel2* voxels2 = volume2->GetVoxelBlocks();
		TVoxel3* voxels3 = volume3->GetVoxelBlocks();

		//asserted to be the same
		int voxel_count = volume1->index.GetVolumeSize().x * volume1->index.GetVolumeSize().y *
		                  volume1->index.GetVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linear_index = 0; linear_index < voxel_count; linear_index++) {
			TVoxel1& voxel1 = voxels1[linear_index];
			TVoxel2& voxel2 = voxels2[linear_index];
			TVoxel3& voxel3 = voxels3[linear_index];
			std::forward<TProcessingFunction>(processing_function)(voxel1, voxel2, voxel3, linear_index);
		}
	}

public:
// region ================================ STATIC TWO-SCENE TRAVERSAL WITH WARPS =======================================

	template<typename TStaticFunctor>
	inline static void
	TraverseAll(VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
	            VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
	            VoxelVolume<TVoxel3, PlainVoxelArray>* volume3) {
		TraverseAll_Generic(
				volume1, volume2, volume3,
				[](TVoxel1& voxel1, TVoxel2& voxel2, TVoxel3& voxel3, const int& linear_index) {
					TStaticFunctor::run(voxel1, voxel2, voxel3);
				}
		);
	}
// endregion
// region ================================ DYNAMIC TWO-SCENE TRAVERSAL WITH WARPS ======================================

	template<typename TFunctor>
	inline static void
	TraverseAll(VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
	            VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
	            VoxelVolume<TVoxel3, PlainVoxelArray>* volume3,
	            TFunctor& functor) {
		TraverseAll_Generic(
				volume1, volume2, volume3,
				[&functor](TVoxel1& voxel1, TVoxel2& voxel2, TVoxel3& voxel3, const int& linear_index) {
					functor(voxel1, voxel2, voxel3);
				}
		);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
	                             VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
	                             VoxelVolume<TVoxel3, PlainVoxelArray>* volume3,
	                             TFunctor& functor) {
		TraverseAll(volume1,volume2,volume3, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
	                        VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
	                        VoxelVolume<TVoxel3, PlainVoxelArray>* volume3,
	                        TFunctor& functor) {
		const ITMLib::PlainVoxelArray::IndexData* index_data = volume1->index.GetIndexData();
		TraverseAll_Generic(
				volume1, volume2, volume3,
				[&functor,&index_data](TVoxel1& voxel1, TVoxel2& voxel2, TVoxel3& voxel3, const int& linear_index) {
					Vector3i voxel_position = ComputePositionVectorFromLinearIndex_PlainVoxelArray(index_data, linear_index);
					functor(voxel1, voxel2, voxel3, voxel_position);
				}
		);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
	                        VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
	                        VoxelVolume<TVoxel3, PlainVoxelArray>* volume3,
	                        TFunctor& functor) {
		TraverseAllWithPosition(volume1,volume2,volume3, functor);
	}

	/** Single-threaded traversal **/
	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition_ST(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			VoxelVolume<TVoxel3, PlainVoxelArray>* volume3,
			TFunctor& functor) {

		assert(volume2->index.GetVolumeSize() == volume3->index.GetVolumeSize() &&
		       volume1->index.GetVolumeSize() == volume3->index.GetVolumeSize());

		// *** traversal vars
		TVoxel1* voxels1 = volume1->GetVoxelBlocks();
		TVoxel2* voxels2 = volume2->GetVoxelBlocks();
		TVoxel3* voxels3 = volume3->GetVoxelBlocks();

		//asserted to be the same
		int voxel_count = volume2->index.GetVolumeSize().x * volume2->index.GetVolumeSize().y *
		                  volume2->index.GetVolumeSize().z;

		const PlainVoxelArray::IndexData* index_data = volume2->index.GetIndexData();

		for (int linearIndex = 0; linearIndex < voxel_count; linearIndex++) {
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(index_data, linearIndex);
			TVoxel1& voxel1 = voxels1[linearIndex];
			TVoxel2& voxel2 = voxels2[linearIndex];
			TVoxel3& voxel3 = voxels3[linearIndex];
			functor(voxel1, voxel2, voxel3, voxelPosition);
		}
	}
// endregion ===========================================================================================================
};

} // namespace ITMLib

