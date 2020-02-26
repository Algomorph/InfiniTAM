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

//local
#include "../Interface/VolumeReduction.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../GlobalTemplateDefines.h"

namespace ITMLib {

template<typename TVoxel>
class VolumeReductionEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> {

	template <typename TRetrieveSingleFunctor, typename TReduceFunctor, typename TOutput>
	/**
	 * \brief Computes reduction on some value from every voxel in the volume, e.g. minimum of some voxel field
	 * \details The position output is not always relevant, i.e. in case of accumulator reductions like the sum of some field.
	 * However, it is extremely useful when trying to either locate a particular value or choose the voxel that best satisfies some criterion,
	 * e.g. minimum or maximum value of some voxel field can be found along with its location.
	 * \tparam TOutput type of the output value, e.g. float when computing minimum of some voxel float field
	 * \tparam TRetrieveSingleFunctor a function object with a static function which accepts a single TVoxel as an argument and returns a TOutput value based on this TVoxel.
	 * \tparam TReduceFunctor a function object with a static function which accepts two ValueAndIndex<TOutput> objects and returns a single ValueAndIndex<TOutput> object
	 * \param position the position of the voxel based on the indices produced by TReduceFunctor when comparing each ValueAndIndex pair.
	 * \return end result based on all voxels in the volume.
	 */
	static TOutput ReduceUtilized(Vector3i& position, const VoxelVolume<TVoxel, VoxelBlockHash>& volume);

};

extern template class VolumeReductionEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>;
extern template class VolumeReductionEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>;

} // namespace ITMLib
