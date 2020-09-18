//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 9/17/20.
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
#include "../../../Objects/Volume/VoxelBlockHash.h"

namespace ITMLib {

namespace internal {
template<typename TVoxel, typename TIndex, MemoryDeviceType TDeviceType>
class VolumeReductionEngine_IndexSpecialized;
} // namespace internal


template<typename TVoxel, MemoryDeviceType TDeviceType>
class BlockVolumeReductionEngine {
public:

	/**
	 * \brief Computes reduction on some value from every voxel in the volume, e.g. minimum of some voxel field. Allows for two different
	 * reduction operations: one used to aggregate results within blocks and one used to aggregate the final block results, which can be
	 * useful if statistics on blocks are desired.
	 *
	 * \details The position output is not always relevant, i.e. in case of accumulator reductions like the sum of some field.
	 * However, it is extremely useful when trying to either locate a particular value or choose the voxel that best satisfies some criterion,
	 * e.g. minimum or maximum value of some voxel field can be found along with its location.
	 *
	 * TRetrieveSingleStaticFunctor is used to retrieve initial values from individual voxels.
	 * TReduceBlockLevelStaticFunctor is used to reduce the result on a block level (until there is 1 result per block).
	 * TReduceResultLevelStaticFunctor is used to further reduce the block level results until there is only a single,
	 * aggregate result of the whole function.
	 *
	 * \tparam TReduceBlockLevelStaticFunctor a function object with a static function which accepts two ReductionResult<TOutput> objects and returns a single ReductionResult<TOutput> object
	 * \tparam TRetrieveSingleStaticFunctor a function object with a static function which accepts a single TVoxel as an argument and returns a TOutput value based on this TVoxel.
	 * \tparam TOutput type of the output value, e.g. float when computing minimum of some voxel float field
	 *
	 * \param position the position of the voxel based on the indices produced by TReduceBlockLevelStaticFunctor when comparing each ReductionResult pair.
	 * \param volume the volume to run the reduction over
	 * \param ignored_value this is a sample value-index-hash triplet that will be ignored / won't skew result of the operation when issued
	 * to the reduction algorithm. For instance, when seeking a minimum of a float field, it makes sense to set this to
	 * ReductionResult<float, VoxelBlockHash>(FLT_MAX, 0, -1). The value is used to pad non-even inputs during the merging step in the reduction.
	 *
	 * \return final reduction result based on all voxels & blocks in the volume.
	 */
	template<typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor, typename TRetrieveSingleStaticFunctor, typename TOutput>
	static TOutput ReduceUtilizedBlocks(Vector3i& position, const VoxelVolume <TVoxel, VoxelBlockHash>* volume,
	                                    ReductionResult <TOutput, VoxelBlockHash> ignored_value = ReductionResultInitializer<TOutput, VoxelBlockHash>::Default()) {
		return internal::VolumeReductionEngine_IndexSpecialized<TVoxel, VoxelBlockHash, TDeviceType>::
		template ReduceUtilizedBlocks<TReduceBlockLevelStaticFunctor, TReduceResultLevelStaticFunctor, TRetrieveSingleStaticFunctor>(
				position, volume, ignored_value
		);
	}

	template<typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor, typename TRetrieveSingleStaticFunctor, typename TOutput>
	static TOutput ReduceUtilizedBlocks2(Vector3i& position, const VoxelVolume <TVoxel, VoxelBlockHash>* volume,
	                                     ReductionResult <TOutput, VoxelBlockHash> ignored_value = ReductionResultInitializer<TOutput, VoxelBlockHash>::Default()) {
		return internal::VolumeReductionEngine_IndexSpecialized<TVoxel, VoxelBlockHash, TDeviceType>::
		template ReduceUtilizedBlocks2<TReduceBlockLevelStaticFunctor, TReduceResultLevelStaticFunctor, TOutput>(
				position, volume,
				CPU_AND_GPU_CAPTURE_LAMBDA()(const TVoxel* block_voxels, int index_within_block) {
					return TRetrieveSingleStaticFunctor::retrieve(block_voxels[index_within_block]);
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
	 * TReduceBlockLevelStaticFunctor is used to reduce the result on a block level (until there is 1 result per block).
	 * TReduceResultLevelStaticFunctor is used to further reduce the block level results until there is only a single,
	 * aggregate result of the whole function.
	 *
	 * \tparam TReduceBlockLevelStaticFunctor a function object with a static function which accepts two ReductionResult<TOutput> objects and returns a single ReductionResult<TOutput> object
	 * \tparam TReduceResultLevelStaticFunctor a function object with a static function which accepts two ReductionResult<TOutput> objects and returns a single ReductionResult<TOutput> object
	 * \tparam TRetrieveSingleDynamicFunctor a function object with a retrieve public member function which accepts a
	 * \tparam TOutput type of the output value, e.g. float when computing minimum of some voxel float field
	 *  single TVoxel as an argument and returns a TOutput value based on this TVoxel.
	 *
	 * \param position the position of the voxel based on the indices produced by TReduceBlockLevelStaticFunctor when comparing each ReductionResult pair.
	 * \param volume the volume to run the reduction over
	 * \param retrieve_functor functor object to retrieve initial TOutput value from individual voxels
	 * \param ignored_value this is a sample value-index-hash triplet that will be ignored / won't skew result of the operation when issued
	 * to the reduction algorithm. For instance, when seeking a minimum of a float field, it makes sense to set this to
	 * ReductionResult<float, VoxelBlockHash>(FLT_MAX, 0, -1). The value is used to pad non-even inputs during the merging step in the reduction.
	 *
	 * \return final reduction result based on all voxels & blocks in the volume.
	 */
	template<typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor, typename TRetrieveSingleDynamicFunctor, typename TOutput>
	static TOutput ReduceUtilizedBlocks(Vector3i& position, const VoxelVolume <TVoxel, VoxelBlockHash>* volume,
	                                    const TRetrieveSingleDynamicFunctor& retrieve_functor,
	                                    ReductionResult <TOutput, VoxelBlockHash> ignored_value = ReductionResultInitializer<TOutput, VoxelBlockHash>::Default()) {
		return internal::VolumeReductionEngine_IndexSpecialized<TVoxel, VoxelBlockHash, TDeviceType>::
		template ReduceUtilizedBlocks<TReduceBlockLevelStaticFunctor, TReduceResultLevelStaticFunctor>(
				position, volume, retrieve_functor, ignored_value);
	}

	template<typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor, typename TRetrieveSingleDynamicFunctor, typename TOutput>
	static TOutput ReduceUtilizedBlocks2(Vector3i& position, const VoxelVolume <TVoxel, VoxelBlockHash>* volume,
	                                     const TRetrieveSingleDynamicFunctor& retrieve_functor,
	                                     ReductionResult <TOutput, VoxelBlockHash> ignored_value = ReductionResultInitializer<TOutput, VoxelBlockHash>::Default()) {
		return internal::VolumeReductionEngine_IndexSpecialized<TVoxel, VoxelBlockHash, TDeviceType>::
		template ReduceUtilizedBlocks2<TReduceBlockLevelStaticFunctor, TReduceResultLevelStaticFunctor>(
				position, volume,
				CPU_AND_GPU_CAPTURE_LAMBDA(&retrieve_functor)(const TVoxel* block_voxels, int index_within_block) {
					return retrieve_functor.retrieve(block_voxels[index_within_block]);
				}, ignored_value
		);
	}
};

} // namespace ITMLib
