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
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../../Utils/Geometry/SpatialIndexConversions.h"
#include "../../../Utils/Geometry/GeometryBooleanOperations.h"

namespace ITMLib {

//TODO: reduce DRY violations

template<typename TVoxel1, typename TVoxel2>
class TwoVolumeTraversalEngine<TVoxel1, TVoxel2, PlainVoxelArray, PlainVoxelArray, MEMORYDEVICE_CPU> {
	/**
	 * \brief Concurrent traversal of two volumes with potentially-differing voxel types
	 * \details The two volumes must have matching dimensions
	 */
private:

	template<typename TFunctor, typename TFunctionCall>
	inline static bool
	TraverseAndCompareAllocated_Generic(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			TFunctor& functor, TFunctionCall&& comparison_function) {

		//1) Maximum bbox of volume union
		//2) TraverseAll that, check matching parts with functor, check rest with "isAltered"
// *** traversal vars

		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::IndexData* volume2Box = volume2->index.GetIndexData();
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		const PlainVoxelArray::IndexData* volume1Box = volume1->index.GetIndexData();

		const Extent3D bounds = IntersectionExtent(*volume1Box, *volume2Box);

		volatile bool mismatch_found = false;
#ifdef WITH_OPENMP
#pragma omp parallel for default(shared)
#endif
		for (int z = bounds.min_z; z < bounds.max_z; z++) {
			if (mismatch_found) continue;
			for (int y = bounds.min_y; y < bounds.max_y; y++) {
				if (mismatch_found) continue;
				for (int x = bounds.min_x; x < bounds.max_x; x++) {
					Vector3i position(x, y, z);
					if (IsPointInBounds(position, *volume1Box)) {
						int primaryLinearIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(volume1Box,
						                                                                        position);
						TVoxel1& voxel1 = voxels1[primaryLinearIndex];
						if (IsPointInBounds(position, *volume2Box)) {
							int secondaryLinearIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(
									volume2Box, position);
							TVoxel2& voxel2 = voxels2[secondaryLinearIndex];
							if (!std::forward<TFunctionCall>(comparison_function)(voxel1, voxel2, position)) {
								mismatch_found = true;
							}
						} else {
							if (isAltered(voxel1)) {
								mismatch_found = true;
							}
						}
					} else {
						if (IsPointInBounds(position, *volume2Box)) {
							int secondaryLinearIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(
									volume2Box, position);
							TVoxel2& voxel2 = voxels2[secondaryLinearIndex];
							if (isAltered(voxel2)) {
								mismatch_found = true;
							}
						}
					}
				}
			}
		}

		return !mismatch_found;
	}

public:
// region ================================ STATIC TWO-VOLUME TRAVERSAL =================================================

	template<typename TStaticFunctor>
	inline static void TraverseAll(VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
	                               VoxelVolume<TVoxel2, PlainVoxelArray>* volume2) {
		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());
// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxel_count = volume1->index.GetVolumeSize().x * volume1->index.GetVolumeSize().y *
		                  volume1->index.GetVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxel_count; linearIndex++) {
			TVoxel1& voxel1 = voxels1[linearIndex];
			TVoxel2& voxel2 = voxels2[linearIndex];
			TStaticFunctor::run(voxel1, voxel2);
		}
	}

	template<typename TStaticFunctor>
	inline static bool
	TraverseAndCompareAll(VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
	                      VoxelVolume<TVoxel2, PlainVoxelArray>* volume2) {
		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());
// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxel_count = volume1->index.GetVolumeSize().x * volume1->index.GetVolumeSize().y *
		                  volume1->index.GetVolumeSize().z;
		volatile bool mismatch_found = false;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxel_count; linearIndex++) {
			if (mismatch_found) continue;
			TVoxel1& voxel1 = voxels1[linearIndex];
			TVoxel2& voxel2 = voxels2[linearIndex];
			if (!TStaticFunctor::run(voxel1, voxel2)) {
				mismatch_found = true;
			}
		}
		return !mismatch_found;
	}

// endregion
// region ================================ STATIC TWO-VOLUME TRAVERSAL WITH VOXEL POSITION =============================

	template<typename TStaticFunctor>
	inline static void
	TraverseAllWithPosition(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2) {
		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());
// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxel_count = volume1->index.GetVolumeSize().x * volume1->index.GetVolumeSize().y *
		                  volume1->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = volume1->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxel_count; linearIndex++) {
			Vector3i voxel_position = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
			TVoxel1& voxel1 = voxels1[linearIndex];
			TVoxel2& voxel2 = voxels2[linearIndex];
			TStaticFunctor::run(voxel1, voxel2, voxel_position);
		}
	}
// endregion

// region ================================ DYNAMIC TWO-VOLUME TRAVERSAL FOR VOLUMES WITH DIFFERING VOXEL TYPES =========

	template<typename TFunctor>
	inline static void
	TraverseAll(VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
	            VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
	            TFunctor& functor) {

		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());
// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxel_count = volume1->index.GetVolumeSize().x * volume1->index.GetVolumeSize().y *
		                  volume1->index.GetVolumeSize().z;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxel_count; linearIndex++) {
			TVoxel1& voxel1 = voxels1[linearIndex];
			TVoxel2& voxel2 = voxels2[linearIndex];
			functor(voxel1, voxel2);
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
	            VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
	            TFunctor& functor){
		TraverseAll(volume1, volume2, functor);
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			TFunctor& functor) {

		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());
// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxel_count = volume1->index.GetVolumeSize().x * volume1->index.GetVolumeSize().y *
		                  volume1->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = volume1->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxel_count; linearIndex++) {
			Vector3i voxel_position = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
			TVoxel1& voxel1 = voxels1[linearIndex];
			TVoxel2& voxel2 = voxels2[linearIndex];
			functor(voxel1, voxel2, voxel_position);
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
	                 VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
	                 TFunctor& functor){
		TraverseAllWithPosition(volume1, volume2, functor);
	}


	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAll(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			TFunctor& functor, bool verbose) {

		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());
// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxel_count = volume1->index.GetVolumeSize().x * volume1->index.GetVolumeSize().y *
		                  volume1->index.GetVolumeSize().z;
		volatile bool mismatch_found = false;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxel_count; linearIndex++) {
			if (mismatch_found) continue;
			TVoxel1& voxel1 = voxels1[linearIndex];
			TVoxel2& voxel2 = voxels2[linearIndex];
			if (!(functor(voxel1, voxel2))) {
				mismatch_found = true;
			}
		}
		return !mismatch_found;
	}

	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAllWithPosition(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			TFunctor& functor, bool verbose) {

		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());
// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxel_count = volume1->index.GetVolumeSize().x * volume1->index.GetVolumeSize().y *
		                  volume1->index.GetVolumeSize().z;
		volatile bool mismatch_found = false;
		PlainVoxelArray::IndexData* indexData = volume1->index.GetIndexData();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearIndex = 0; linearIndex < voxel_count; linearIndex++) {
			if (mismatch_found) continue;
			TVoxel1& voxel1 = voxels1[linearIndex];
			TVoxel2& voxel2 = voxels2[linearIndex];
			Vector3i voxel_position = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
			if (!(functor(voxel1, voxel2, voxel_position))) {
				mismatch_found = true;
			}
		}
		return !mismatch_found;
	}

	/**
	 * \brief Runs a functor returning true/false pairs of voxels within the allocation bounds of both
	 * voxel volumes until such locations are exhausted or one of the runs returns false.
	 * \tparam TFunctor
	 * \param volume1
	 * \param volume2
	 * \param functor
	 * \return
	 */
	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAllocated(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			TFunctor& functor) {
		return TraverseAndCompareAllocated_Generic(
				volume1, volume2, functor,
				[&functor](TVoxel1& voxelPrimary,
				           TVoxel2& voxelSecondary,
				           const Vector3i& position) {
					return functor(voxelPrimary, voxelSecondary);
				}
		);
	}

/**
	 * \brief Runs a functor returning true/false pairs of voxels within the allocation bounds of both
	 * voxel volumes until such locations are exhausted or one of the runs returns false.
	 * \tparam TFunctor
	 * \param volume1
	 * \param volume2
	 * \param functor
	 * \return
	 */
	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAllocatedWithPosition(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			TFunctor& functor) {
		return TraverseAndCompareAllocated_Generic(
				volume1, volume2, functor,
				[&functor](TVoxel1& voxelPrimary,
				           TVoxel2& voxelSecondary,
				           const Vector3i& position) {
					return functor(voxelPrimary, voxelSecondary, position);
				}
		);
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithinBoundsWithPosition(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			TFunctor& functor, Vector6i bounds) {

		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());
// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxel_count = volume1->index.GetVolumeSize().x * volume1->index.GetVolumeSize().y *
		                  volume1->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = volume1->index.GetIndexData();
		int vmIndex;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int z = bounds.min_z; z < bounds.max_z; z++) {
			for (int y = bounds.min_y; y < bounds.max_y; y++) {
				for (int x = bounds.min_x; x < bounds.max_x; x++) {
					Vector3i position(x, y, z);
					int linearIndex = findVoxel(indexData, Vector3i(x, y, z), vmIndex);
					Vector3i voxel_position = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData,
					                                                                               linearIndex);
					TVoxel1& voxel1 = voxels1[linearIndex];
					TVoxel2& voxel2 = voxels2[linearIndex];
					functor(voxel1, voxel2, voxel_position);
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition_ST(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			TFunctor& functor) {
		assert(volume1->index.GetVolumeSize() == volume2->index.GetVolumeSize());
		// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		//asserted to be the same
		int voxel_count = volume1->index.GetVolumeSize().x * volume1->index.GetVolumeSize().y *
		                  volume1->index.GetVolumeSize().z;
		const PlainVoxelArray::IndexData* indexData = volume1->index.GetIndexData();
		for (int linearIndex = 0; linearIndex < voxel_count; linearIndex++) {
			Vector3i voxel_position = ComputePositionVectorFromLinearIndex_PlainVoxelArray(indexData, linearIndex);
			TVoxel1& voxel1 = voxels1[linearIndex];
			TVoxel2& voxel2 = voxels2[linearIndex];
			functor(voxel1, voxel2, voxel_position);
		}
	}
// endregion ===========================================================================================================
};

} // namespace ITMLib

