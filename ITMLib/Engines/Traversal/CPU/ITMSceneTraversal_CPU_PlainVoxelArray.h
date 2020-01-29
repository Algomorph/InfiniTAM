
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
#include "../Interface/ITMSceneTraversal.h"
#include "../../EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../../../Utils/Geometry/ITM3DIndexConversions.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../Shared/ITMSceneTraversal_Shared.h"
#include "../../../Utils/Geometry/ITMGeometryBooleanOperations.h"

namespace ITMLib {


//======================================================================================================================
//                         CONTAINS TRAVERSAL FUNCTIONS FOR SCENES USING PlainVoxelArray FOR INDEXING
//======================================================================================================================
//static-member-only classes are used here instead of namespaces to utilize template specialization (and maximize code reuse)
template<typename TVoxel>
class ITMSceneTraversalEngine<TVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> {
public:
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================
	template<typename TFunctor>
	inline static void
	VoxelTraversal(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, TFunctor& functor) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		int voxelCount =
				scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y * scene->index.GetVolumeSize().z;

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
	VoxelTraversal_SingleThreaded(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, TFunctor& functor) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		int voxelCount =
				scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y * scene->index.GetVolumeSize().z;
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			TVoxel& voxel = voxels[linearIndex];
			functor(voxel);
		}
	}


	template<typename TFunctor>
	inline static void
	VoxelPositionTraversal(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, TFunctor& functor) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		int voxelCount =
				scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y * scene->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = scene->index.GetIndexData();
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
	VoxelTraversalWithinBounds(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, TFunctor& functor, Vector6i bounds) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		int vmIndex = 0;
		const PlainVoxelArray::IndexData* indexData = scene->index.GetIndexData();
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
	VoxelPositionTraversalWithinBounds(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, TFunctor& functor,
	                                   Vector6i bounds) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		int vmIndex = 0;
		const PlainVoxelArray::IndexData* indexData = scene->index.GetIndexData();
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
	VoxelPositionAndHashEntryTraversalWithinBounds(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, TFunctor& functor,
	                                               Vector6i bounds) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		int vmIndex = 0;
		const PlainVoxelArray::IndexData* indexData = scene->index.GetIndexData();
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

// endregion ===========================================================================================================

// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void StaticVoxelTraversal(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		int voxelCount =
				scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y * scene->index.GetVolumeSize().z;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {

			TVoxel& voxel = voxels[linearIndex];
			TStaticFunctor::run(voxel);
		}
	}

	template<typename TStaticFunctor>
	inline static void StaticVoxelPositionTraversal(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		int voxelCount =
				scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y * scene->index.GetVolumeSize().z;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(scene, linearIndex);
			TVoxel& voxel = voxels[linearIndex];
			TStaticFunctor::run(voxel, voxelPosition);
		}
	}
// endregion

};


template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, PlainVoxelArray, PlainVoxelArray, MEMORYDEVICE_CPU> {
private:

	template<typename TFunctor, typename TFunctionCall>
	inline static bool
	DualVoxelTraversal_AllTrue_AllocatedOnly_Generic(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryVolume,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryVolume,
			TFunctor& functor, TFunctionCall&& functionCall) {

		//1) Maximum bbox of volume union
		//2) Traverse that, check matching parts with functor, check rest with "isAltered"
// *** traversal vars

		TVoxelSecondary* secondaryVoxels = secondaryVolume->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::IndexData* secondaryVolumeBox = secondaryVolume->index.GetIndexData();
		TVoxelPrimary* primaryVoxels = primaryVolume->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::IndexData* primaryVolumeBox = primaryVolume->index.GetIndexData();

		const Extent3D boundingExtent = maximumExtent(*primaryVolumeBox, *secondaryVolumeBox);

		volatile bool mismatchFound = false;
#ifdef WITH_OPENMP
#pragma omp parallel for default(shared)
#endif
		for (int z = boundingExtent.min_z; z < boundingExtent.max_z; z++) {
			if (mismatchFound) continue;
			for (int y = boundingExtent.min_y; y < boundingExtent.max_y; y++) {
				if (mismatchFound) continue;
				for (int x = boundingExtent.min_x; x < boundingExtent.max_x; x++) {
					Vector3i position(x, y, z);
					if (isPointInBounds(position, *primaryVolumeBox)) {
						int primaryLinearIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(primaryVolumeBox,
						                                                                        position);
						TVoxelPrimary& primaryVoxel = primaryVoxels[primaryLinearIndex];
						if (isPointInBounds(position, *secondaryVolumeBox)) {
							int secondaryLinearIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(
									secondaryVolumeBox, position);
							TVoxelSecondary& secondaryVoxel = secondaryVoxels[secondaryLinearIndex];
							if (!std::forward<TFunctionCall>(functionCall)(primaryVoxel, secondaryVoxel, position)) {
								mismatchFound = true;
							}
						} else {
							if (isAltered(primaryVoxel)) {
								mismatchFound = true;
							}
						}
					} else {
						if (isPointInBounds(position, *secondaryVolumeBox)) {
							int secondaryLinearIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(
									secondaryVolumeBox, position);
							TVoxelSecondary& secondaryVoxel = secondaryVoxels[secondaryLinearIndex];
							if (isAltered(secondaryVoxel)) {
								mismatchFound = true;
							}
						}
					}
				}
			}
		}

		return !mismatchFound;
	}

public:
// region ================================ STATIC TWO-SCENE TRAVERSAL ==================================================

	template<typename TStaticFunctor>
	inline static void
	StaticDualVoxelTraversal(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene) {
		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());
// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryScene->index.GetVolumeSize().x * primaryScene->index.GetVolumeSize().y *
		                 primaryScene->index.GetVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
			TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
			TStaticFunctor::run(primaryVoxel, secondaryVoxel);
		}
	}


	template<typename TStaticFunctor>
	inline static bool
	StaticDualVoxelTraversal_AllTrue(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene) {
		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());
// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryScene->index.GetVolumeSize().x * primaryScene->index.GetVolumeSize().y *
		                 primaryScene->index.GetVolumeSize().z;
		volatile bool mismatchFound = false;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			if (mismatchFound) continue;
			TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
			TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
			if (!TStaticFunctor::run(primaryVoxel, secondaryVoxel)) {
				mismatchFound = true;
			}
		}
		return !mismatchFound;
	}

// endregion
// region ================================ STATIC TWO-SCENE TRAVERSAL WITH VOXEL POSITION ==============================

	template<typename TStaticFunctor>
	inline static void
	StaticDualVoxelPositionTraversal(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene) {
		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());
// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryScene->index.GetVolumeSize().x * primaryScene->index.GetVolumeSize().y *
		                 primaryScene->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = primaryScene->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
			TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
			TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
			TStaticFunctor::run(primaryVoxel, secondaryVoxel, voxelPosition);
		}
	}
// endregion

// region ================================ DYNAMIC TWO-SCENE TRAVERSAL FOR SCENES WITH DIFFERING VOXEL TYPES ===========



	template<typename TFunctor>
	inline static void
	DualVoxelTraversal(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
			TFunctor& functor) {

		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());
// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryScene->index.GetVolumeSize().x * primaryScene->index.GetVolumeSize().y *
		                 primaryScene->index.GetVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
			TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
			functor(primaryVoxel, secondaryVoxel);
		}
	}

	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryVolume,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryVolume,
			TFunctor& functor, bool verbose) {

		assert(primaryVolume->index.GetVolumeSize() == secondaryVolume->index.GetVolumeSize());
// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryVolume->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryVolume->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryVolume->index.GetVolumeSize().x * primaryVolume->index.GetVolumeSize().y *
		                 primaryVolume->index.GetVolumeSize().z;
		volatile bool mismatchFound = false;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			if (mismatchFound) continue;
			TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
			TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
			if (!(functor(primaryVoxel, secondaryVoxel))) {
				mismatchFound = true;
			}
		}
		return !mismatchFound;
	}

	template<typename TFunctor>
	inline static bool
	DualVoxelPositionTraversal_AllTrue(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryVolume,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryVolume,
			TFunctor& functor, bool verbose) {

		assert(primaryVolume->index.GetVolumeSize() == secondaryVolume->index.GetVolumeSize());
// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryVolume->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryVolume->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryVolume->index.GetVolumeSize().x * primaryVolume->index.GetVolumeSize().y *
		                 primaryVolume->index.GetVolumeSize().z;
		volatile bool mismatchFound = false;
		PlainVoxelArray::IndexData* indexData = primaryVolume->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			if (mismatchFound) continue;
			TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
			TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
			if (!(functor(primaryVoxel, secondaryVoxel, voxelPosition))) {
				mismatchFound = true;
			}
		}
		return !mismatchFound;
	}

	/**
	 * \brief Runs a functor returning true/false pairs of voxels within the allocation bounds of both
	 * voxel volumes until such locations are exhausted or one of the runs returns false.
	 * \tparam TFunctor
	 * \param primaryVolume
	 * \param secondaryVolume
	 * \param functor
	 * \return
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue_AllocatedOnly(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryVolume,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryVolume,
			TFunctor& functor) {
		return DualVoxelTraversal_AllTrue_AllocatedOnly_Generic(
				primaryVolume, secondaryVolume, functor,
				[&functor](TVoxelPrimary& voxelPrimary,
				           TVoxelSecondary& voxelSecondary,
				           const Vector3i& position) {
					return functor(voxelPrimary, voxelSecondary);
				}
		);
	}
/**
	 * \brief Runs a functor returning true/false pairs of voxels within the allocation bounds of both
	 * voxel volumes until such locations are exhausted or one of the runs returns false.
	 * \tparam TFunctor
	 * \param primaryVolume
	 * \param secondaryVolume
	 * \param functor
	 * \return
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelPositionTraversal_AllTrue_AllocatedOnly(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryVolume,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryVolume,
			TFunctor& functor) {
		return DualVoxelTraversal_AllTrue_AllocatedOnly_Generic(
				primaryVolume, secondaryVolume, functor,
				[&functor](TVoxelPrimary& voxelPrimary,
				           TVoxelSecondary& voxelSecondary,
				           const Vector3i& position) {
					return functor(voxelPrimary, voxelSecondary, position);
				}
		);
	}

	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
			TFunctor& functor) {

		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());
// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryScene->index.GetVolumeSize().x * primaryScene->index.GetVolumeSize().y *
		                 primaryScene->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = primaryScene->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
			TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
			TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
			functor(primaryVoxel, secondaryVoxel, voxelPosition);
		}
	}

	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversalWithinBounds(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
			TFunctor& functor, Vector6i bounds) {

		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());
// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryScene->index.GetVolumeSize().x * primaryScene->index.GetVolumeSize().y *
		                 primaryScene->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = primaryScene->index.GetIndexData();
		int vmIndex;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int z = bounds.min_z; z < bounds.max_z; z++) {
			for (int y = bounds.min_y; y < bounds.max_y; y++) {
				for (int x = bounds.min_x; x < bounds.max_x; x++) {
					Vector3i position(x, y, z);
					int linearIndex = findVoxel(indexData, Vector3i(x, y, z), vmIndex);
					Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData,
					                                                                              linearIndex);
					TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
					TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
					functor(primaryVoxel, secondaryVoxel, voxelPosition);
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal_SingleThreaded(
			ITMVoxelVolume<TVoxelPrimary, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, PlainVoxelArray>* secondaryScene,
			TFunctor& functor) {
		assert(primaryScene->index.GetVolumeSize() == secondaryScene->index.GetVolumeSize());
		// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxelCount = primaryScene->index.GetVolumeSize().x * primaryScene->index.GetVolumeSize().y *
		                 primaryScene->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = primaryScene->index.GetIndexData();
		for (int linearIndex = 0; linearIndex < voxelCount; linearIndex++) {
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
			TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
			TVoxelSecondary& secondaryVoxel = secondaryVoxels[linearIndex];
			functor(primaryVoxel, secondaryVoxel, voxelPosition);
		}
	}
// endregion ===========================================================================================================
};

template<typename TVoxel, typename TWarp>
class ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, PlainVoxelArray, MEMORYDEVICE_CPU> {
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
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* secondaryScene,
			ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
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
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* primaryScene,
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* secondaryScene,
			ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
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
}//namespace ITMLib
//#pragma clang diagnostic pop