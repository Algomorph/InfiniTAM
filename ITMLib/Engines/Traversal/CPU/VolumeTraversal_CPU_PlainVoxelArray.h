
//  ================================================================
//  Created by Gregory Kramida on 5/22/18.
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
#include "../../EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../../../Utils/Geometry/SpatialIndexConversions.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../Shared/VolumeTraversal_Shared.h"
#include "../../../Utils/Geometry/GeometryBooleanOperations.h"

namespace ITMLib {


//======================================================================================================================
//                         CONTAINS TRAVERSAL FUNCTIONS FOR SCENES USING PlainVoxelArray FOR INDEXING
//======================================================================================================================
//static-member-only classes are used here instead of namespaces to utilize template specialization (and maximize code reuse)
template<typename TVoxel>
class VolumeTraversalEngine<TVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> {
private: // static functions
	template<typename TVoxel_Modifiers, typename TVolume, typename TFunctor>
	inline static void
	TraverseAll_Generic(TVolume* volume, TFunctor& functor) {
		TVoxel_Modifiers* voxels = volume->GetVoxels();
		const int voxel_count = volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y * volume->index.GetVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(voxels, functor) firstprivate(voxel_count)
#endif
		for (int linear_index = 0; linear_index < voxel_count; linear_index++) {
			TVoxel_Modifiers& voxel = voxels[linear_index];
			functor(voxel);
		}
	}

	template<typename TVoxel_Modifiers, typename TVolume, typename TFunctor>
	inline static void
	TraverseAllWithPosition_Generic(TVolume* volume, TFunctor& functor) {
		TVoxel_Modifiers* voxels = volume->GetVoxels();
		const int voxel_count = volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y * volume->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* index_data = volume->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(voxels, functor, index_data) firstprivate(voxel_count)
#endif
		for (int linear_index = 0; linear_index < voxel_count; linear_index++) {
			Vector3i voxel_position = ComputePositionVectorFromLinearIndex_PlainVoxelArray(index_data, linear_index);
			TVoxel_Modifiers& voxel = voxels[linear_index];
			functor(voxel, voxel_position);
		}
	}
	
public: // static functions
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

	//TODO: remove
	template<typename TFunctor>
	inline static void
	TraverseAll_ST(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TVoxel* voxels = volume->GetVoxels();
		int voxel_count =
				volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y * volume->index.GetVolumeSize().z;
		for (int linear_index = 0; linear_index < voxel_count; linear_index++) {
			TVoxel& voxel = voxels[linear_index];
			functor(voxel);
		}
	}
	//TODO: remove
	template<typename TFunctor>
	inline static void
	TraverseUtilized_ST(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAll_ST(volume, functor);
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
		TraverseAllWithPosition(volume,functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(const VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAllWithPosition(volume,functor);
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithinBounds(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor, Vector6i bounds) {
		TVoxel* voxels = volume->GetVoxels();
		const PlainVoxelArray::IndexData* index_data = volume->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(voxels, functor, index_data)
#endif
		for (int z = bounds.min_z; z < bounds.max_z; z++) {
			for (int y = bounds.min_y; y < bounds.max_y; y++) {
				for (int x = bounds.min_x; x < bounds.max_x; x++) {
					int vm_index = 0;
					int linear_index = findVoxel(index_data, Vector3i(x, y, z), vm_index);
					TVoxel& voxel = voxels[linear_index];
					functor(voxel);
				}
			}
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithinBounds(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor, Vector6i bounds) {
		TraverseAllWithinBounds(volume, functor, bounds);
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithinBoundsWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor,
	                                    Vector6i bounds) {
		TVoxel* voxels = volume->GetVoxels();
		int vmIndex = 0;
		const PlainVoxelArray::IndexData* index_data = volume->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(voxels, functor)
#endif
		for (int z = bounds.min_z; z < bounds.max_z; z++) {
			for (int y = bounds.min_y; y < bounds.max_y; y++) {
				for (int x = bounds.min_x; x < bounds.max_x; x++) {
					Vector3i position(x, y, z);
					int linear_index = findVoxel(index_data, Vector3i(x, y, z), vmIndex);
					TVoxel& voxel = voxels[linear_index];
					functor(voxel, position);
				}
			}
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithinBoundsWithPositionAndHashEntry(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor,
	                                    Vector6i bounds) {
		TVoxel* voxels = volume->GetVoxels();
		int vmIndex = 0;
		const PlainVoxelArray::IndexData* index_data = volume->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(voxels, functor)
#endif
		for (int z = bounds.min_z; z < bounds.max_z; z++) {
			for (int y = bounds.min_y; y < bounds.max_y; y++) {
				for (int x = bounds.min_x; x < bounds.max_x; x++) {
					Vector3i position(x, y, z);
					int linear_index = findVoxel(index_data, Vector3i(x, y, z), vmIndex);
					TVoxel& voxel = voxels[linear_index];
					functor(voxel, position);
				}
			}
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithinBoundsWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor,
	                                    Vector6i bounds) {
		TraverseAllWithinBoundsWithPosition(volume, functor, bounds);
	}

// endregion ===========================================================================================================

// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void TraverseAll(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TVoxel* voxels = volume->GetVoxels();
		int voxel_count =
				volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y * volume->index.GetVolumeSize().z;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(voxels) firstprivate(voxel_count)
#endif
		for (int linear_index = 0; linear_index < voxel_count; linear_index++) {

			TVoxel& voxel = voxels[linear_index];
			TStaticFunctor::run(voxel);
		}
	}

	template<typename TStaticFunctor>
	inline static void TraverseUtilized(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TraverseAll<TStaticFunctor>(volume);
	}

	template<typename TStaticFunctor>
	inline static void TraverseAllWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TVoxel* voxels = volume->GetVoxels();
		int voxel_count =
				volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y * volume->index.GetVolumeSize().z;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(voxels)
#endif
		for (int linear_index = 0; linear_index < voxel_count; linear_index++) {
			Vector3i voxel_position = ComputePositionVectorFromLinearIndex_PlainVoxelArray(volume, linear_index);
			TVoxel& voxel = voxels[linear_index];
			TStaticFunctor::run(voxel, voxel_position);
		}
	}

	template<typename TStaticFunctor>
	inline static void TraverseUtilizedWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TraverseAllWithPosition(volume);
	}
// endregion

};


}//namespace ITMLib
//#pragma clang diagnostic pop