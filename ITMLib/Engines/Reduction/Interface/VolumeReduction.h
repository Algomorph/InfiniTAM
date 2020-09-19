//  ================================================================
//  Created by Gregory Kramida on 2/26/20.
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

#include "../../../../ORUtils/MemoryDeviceType.h"
#include "../Shared/ReductionResult.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../../ORUtils/CrossPlatformMacros.h"
#include "../../../Utils/CUDA/CudaCallWrappers.cuh"

namespace ITMLib {

namespace internal {
template<typename TVoxel, typename TIndex, MemoryDeviceType TDeviceType>
class VolumeReductionEngine_IndexSpecialized;
} // namespace internal


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class VolumeReductionEngine {
public:
	/**
	 * \brief Computes reduction on some value from every voxel in the volume, e.g. minimum of some voxel field.
	 *
	 * \details The position output is not always relevant, i.e. in case of accumulator reductions like the sum of some field.
	 * However, it is extremely useful when trying to either locate a particular value or choose the voxel that best satisfies some criterion,
	 * e.g. minimum or maximum value of some voxel field can be found along with its location.
	 *
	 * TRetrieveSingleStaticFunctor is used to retrieve initial values from individual voxels.
	 * TReduceStaticFunctor is used to reduce the results over all initial values.
	 *
	 * \tparam TReduceStaticFunctor a function object with a static function which accepts two ReductionResult<TOutput> objects and returns a single
	 * ReductionResult<TOutput> object
	 * \tparam TRetrieveSingleStaticFunctor a function object with a static function which accepts a single TVoxel as an argument and returns a
	 * TOutput value based on this TVoxel.
	 * \tparam TOutput type of the output value, e.g. float when computing minimum of some voxel float field
	 *
	 * \param position the position of the voxel based on the indices produced by TReduceBlockLevelStaticFunctor when comparing each ReductionResult pair.
	 * \param volume the volume to run the reduction over
	 * \param ignored_value this is a sample value-index-hash triplet that will be ignored / won't skew result of the operation when issued
	 * to the reduction algorithm. For instance, when seeking a minimum of a float field, it makes sense to set this to {FLT_MAX, 0, -1}.
	 * This is necessary to normalize input sizes.
	 * \return end result based on all voxels in the volume.
	 */
	template<typename TReduceStaticFunctor, typename TRetrieveSingleStaticFunctor, typename TOutput>
	static TOutput ReduceUtilized(Vector3i& position, const VoxelVolume<TVoxel, TIndex>* volume,
	                              ReductionResult<TOutput, TIndex> ignored_value = ReductionResultInitializer<TOutput, TIndex>::Default()) {
		return internal::VolumeReductionEngine_IndexSpecialized<TVoxel, TIndex, TMemoryDeviceType>
		::template ReduceUtilized_Generic<TReduceStaticFunctor>(
				position, volume,
				CPU_AND_GPU_CAPTURE_LAMBDA()(const TVoxel* voxels, unsigned int index) {
					return TRetrieveSingleStaticFunctor::retrieve(voxels[index]);
				}, ignored_value
		);
	}

	/**
	 * \brief Computes reduction on some value from every voxel in the volume, e.g. minimum of some voxel field. Allows for two different
	 * reduction operations: one used to aggregate results within blocks and one used to aggregate the final block results, which can be
	 * useful if statistics on blocks are desired.
	 *
	 * \details The position output is not always relevant, i.e. in case of accumulator reductions like the sum of some field.
	 * However, it is extremely useful when trying to either locate a particular value or choose the voxel that best satisfies some criterion,
	 * e.g. minimum or maximum value of some voxel field can be found along with its location.
	 *
	 * TRetrieveSingleDynamicFunctor is used to retrieve initial values from individual voxels.
	 * TReduceStaticFunctor is used to reduce the results over all initial values.
	 *
	 * \tparam TReduceStaticFunctor a function object with a static function which accepts two ReductionResult<TOutput> objects and returns a single
	 * ReductionResult<TOutput> object.
	 * \tparam TRetrieveSingleDynamicFunctor a function object with a retrieve public member function which accepts a
	 *  single TVoxel as an argument and returns a TOutput value based on this TVoxel.
	 * \tparam TOutput type of the output value, e.g. float when computing minimum of some voxel float field
	 *
	 * \param position the position of the voxel based on the indices produced by TReduceBlockLevelStaticFunctor when comparing each ReductionResult pair.
	 * \param volume the volume to run the reduction over
	 * \param retrieve_functor functor object to retrieve initial TOutput value from individual voxels
	 * \param ignored_value this is a sample value-index-hash triplet that will be ignored / won't skew result of the operation when issued
	 * to the reduction algorithm. For instance, when seeking a minimum of a float field, it makes sense to set this to {FLT_MAX, UINT_MAX}.
	 * This is necessary to normalize input sizes.
	 * \return end result based on all voxels in the volume.
	 */
	template<typename TReduceStaticFunctor, typename TRetrieveSingleDynamicFunctor, typename TOutput>
	static TOutput ReduceUtilized(Vector3i& position, const VoxelVolume<TVoxel, TIndex>* volume,
	                              const TRetrieveSingleDynamicFunctor& retrieve_functor,
	                              ReductionResult<TOutput, TIndex> ignored_value = ReductionResultInitializer<TOutput, TIndex>::Default()) {
		TOutput result;
		internal::UploadConstFunctorIfNecessaryAndCall<TMemoryDeviceType>(
				retrieve_functor,
				[&result, &position, &volume, &ignored_value](const TRetrieveSingleDynamicFunctor* functor_prepared) {
					result = internal::VolumeReductionEngine_IndexSpecialized<TVoxel, TIndex, TMemoryDeviceType>::
					template ReduceUtilized_Generic<TReduceStaticFunctor>(
							position, volume,
							CPU_AND_GPU_CAPTURE_LAMBDA(&functor_prepared)(const TVoxel* voxels, unsigned int index) {
								return functor_prepared->retrieve(voxels[index]);
							}, ignored_value
					);
				}
		);
		return result;
	}

};

} // namespace ITMLib


