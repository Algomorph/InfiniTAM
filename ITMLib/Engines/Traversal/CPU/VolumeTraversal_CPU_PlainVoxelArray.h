
//  ================================================================
//  Created by Gregory Kramida on 5/22/18.
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
public:
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================
	template<typename TFunctor>
	inline static void
	TraverseAll(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		int voxelCount =
				volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y * volume->index.GetVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			TVoxel& voxel = voxels[linearIndex];
			functor(voxel);
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAll(volume, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseAll_ST(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		int voxelCount =
				volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y * volume->index.GetVolumeSize().z;
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			TVoxel& voxel = voxels[linearIndex];
			functor(voxel);
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized_ST(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAll_ST(volume, functor);
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		int voxelCount =
				volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y * volume->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = volume->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
			TVoxel& voxel = voxels[linearIndex];
			functor(voxel, voxelPosition);
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor) {
		TraverseAllWithPosition(volume,functor);
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithinBounds(VoxelVolume<TVoxel, PlainVoxelArray>* volume, TFunctor& functor, Vector6i bounds) {
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		int vmIndex = 0;
		const PlainVoxelArray::IndexData* indexData = volume->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int z = bounds.min_z; z < bounds.max_z; z++) {
			for (int y = bounds.min_y; y < bounds.max_y; y++) {
				for (int x = bounds.min_x; x < bounds.max_x; x++) {
					int linearIndex = findVoxel(indexData, Vector3i(x, y, z), vmIndex);
					TVoxel& voxel = voxels[linearIndex];
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
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		int vmIndex = 0;
		const PlainVoxelArray::IndexData* indexData = volume->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int z = bounds.min_z; z < bounds.max_z; z++) {
			for (int y = bounds.min_y; y < bounds.max_y; y++) {
				for (int x = bounds.min_x; x < bounds.max_x; x++) {
					Vector3i position(x, y, z);
					int linearIndex = findVoxel(indexData, Vector3i(x, y, z), vmIndex);
					TVoxel& voxel = voxels[linearIndex];
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
	inline static void StaticTraverseAll(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		int voxelCount =
				volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y * volume->index.GetVolumeSize().z;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {

			TVoxel& voxel = voxels[linearIndex];
			TStaticFunctor::run(voxel);
		}
	}

	template<typename TStaticFunctor>
	inline static void StaticTraverseUtilized(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		StaticTraverseAll<TStaticFunctor>(volume);
	}

	template<typename TStaticFunctor>
	inline static void StaticTraverseAllWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		int voxelCount =
				volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y * volume->index.GetVolumeSize().z;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(volume, linearIndex);
			TVoxel& voxel = voxels[linearIndex];
			TStaticFunctor::run(voxel, voxelPosition);
		}
	}

	template<typename TStaticFunctor>
	inline static void StaticTraverseUtilizedWithPosition(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		StaticTraverseAllWithPosition(volume);
	}
// endregion

};


}//namespace ITMLib
//#pragma clang diagnostic pop