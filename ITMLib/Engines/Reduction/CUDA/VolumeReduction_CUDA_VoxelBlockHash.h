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
#include "../Shared/ReductionResult.h"
#include "../../../GlobalTemplateDefines.h"
#include "../../../Utils/Geometry/SpatialIndexConversions.h"
#include "VolumeReduction_CUDA_VoxelBlockHash_Kernels.h"

namespace ITMLib {

template<typename TVoxel>
class VolumeReductionEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> {
public:
	template <typename TRetrieveSingleFunctor, typename TReduceFunctor, typename TOutput>
	/**
	 * \brief Computes reduction on some value from every voxel in the volume, e.g. minimum of some voxel field
	 * \details The position output is not always relevant, i.e. in case of accumulator reductions like the sum of some field.
	 * However, it is extremely useful when trying to either locate a particular value or choose the voxel that best satisfies some criterion,
	 * e.g. minimum or maximum value of some voxel field can be found along with its location.
	 * \tparam TOutput type of the output value, e.g. float when computing minimum of some voxel float field
	 * \tparam TRetrieveSingleFunctor a function object with a static function which accepts a single TVoxel as an argument and returns a TOutput value based on this TVoxel.
	 * \tparam TReduceFunctor a function object with a static function which accepts two ReductionResult<TOutput> objects and returns a single ReductionResult<TOutput> object
	 * \param position the position of the voxel based on the indices produced by TReduceFunctor when comparing each ReductionResult pair.
	 * \param volume the volume to run the reduction over
	 * \param ignored_value this is a sample value-index-hash triplet that will be ignored / won't skew result of the operation when issued
	 * to the reduction algorithm. For instance, when seeking a minimum of a float field, it makes sense to set this to {FLT_MAX, 0, -1}.
	 * This is necessary to normalize input sizes.
	 * \return end result based on all voxels in the volume.
	 */
	static TOutput ReduceUtilized(Vector3i& position, const VoxelVolume<TVoxel, VoxelBlockHash>* volume, ReductionResult<TOutput, VoxelBlockHash> ignored_value = ReductionResult<TOutput, VoxelBlockHash>()){
		const int utilized_entry_count = volume->index.GetUtilizedHashBlockCount();
		const int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();
		const HashEntry* hash_entries = volume->index.GetEntries();
		const TVoxel* voxels = volume->voxels.GetVoxelBlocks();

		const int half_block_voxel_count = VOXEL_BLOCK_SIZE3 / 2;

		auto ceilIntDiv = [](int dividend, int divisor) {
			return dividend / divisor + (dividend % divisor != 0);
		};

		auto getNormalizedCount = [&half_block_voxel_count, &ceilIntDiv](int count) {
			return ceilIntDiv(count, half_block_voxel_count) * half_block_voxel_count;
		};

		const int normalized_entry_count = getNormalizedCount(utilized_entry_count);
		int tail_length = normalized_entry_count - utilized_entry_count;

		ORUtils::MemoryBlock<ReductionResult<TOutput, VoxelBlockHash>> result_buffer1(normalized_entry_count,
		                                                                              MEMORYDEVICE_CUDA);

		dim3 cuda_block_size(half_block_voxel_count);
		dim3 cuda_tail_grid_size(ceilIntDiv(tail_length, half_block_voxel_count));

		setTailToIgnored<TOutput>
		<< < cuda_tail_grid_size, cuda_block_size >> >
		(result_buffer1.GetData(MEMORYDEVICE_CUDA), utilized_entry_count, normalized_entry_count, ignored_value);
		ORcudaKernelCheck;

		dim3 cuda_utilized_grid_size(utilized_entry_count);

		computeVoxelHashReduction_BlockLevel<TVoxel, TRetrieveSingleFunctor, TReduceFunctor, TOutput>
		<< < cuda_utilized_grid_size, cuda_block_size >> >
		(result_buffer1.GetData(
				MEMORYDEVICE_CUDA), voxels, hash_entries, utilized_hash_codes);
		ORcudaKernelCheck;


		int output_count = utilized_entry_count;
		int normalized_output_count = normalized_entry_count;

		ORUtils::MemoryBlock<ReductionResult<TOutput, VoxelBlockHash>> result_buffer2(
				normalized_entry_count / half_block_voxel_count,
				MEMORYDEVICE_CUDA);
		/* input & output are swapped in the beginning of the loop code as opposed to the end,
		 so that output_buffer points to the final buffer after the loop finishes*/
		ORUtils::MemoryBlock<ReductionResult<TOutput, VoxelBlockHash>>* input_buffer = &result_buffer2;
		ORUtils::MemoryBlock<ReductionResult<TOutput, VoxelBlockHash>>* output_buffer = &result_buffer1;

		while (output_count > 1) {
			std::swap(input_buffer, output_buffer);
			output_count = normalized_output_count / half_block_voxel_count;
			normalized_output_count = getNormalizedCount(output_count);
			tail_length = normalized_output_count - output_count;
			cuda_tail_grid_size.x = ceilIntDiv(tail_length, half_block_voxel_count);
			setTailToIgnored<TOutput>
			<< < cuda_tail_grid_size, cuda_block_size >> >
			(output_buffer->GetData(MEMORYDEVICE_CUDA), output_count, normalized_output_count, ignored_value);
			ORcudaKernelCheck;

			dim3 cuda_grid_size(output_count);
			computeVoxelHashReduction_ResultLevel<TReduceFunctor, TOutput>
			<< < cuda_grid_size, cuda_block_size >> >
			(output_buffer->GetData(MEMORYDEVICE_CUDA), input_buffer->GetData(
					MEMORYDEVICE_CUDA));
			ORcudaKernelCheck;
		}

		ReductionResult<TOutput, VoxelBlockHash> final_result;
		ORcudaSafeCall(cudaMemcpyAsync(&final_result, output_buffer->GetData(MEMORYDEVICE_CUDA),
		                               sizeof(ReductionResult<TOutput, VoxelBlockHash>), cudaMemcpyDeviceToHost));
		HashEntry entry_with_result = volume->index.GetHashEntry(final_result.hash_code);
		position = ComputePositionVectorFromLinearIndex_VoxelBlockHash(entry_with_result.pos,
		                                                               static_cast<int> (final_result.index_within_block));
		return final_result.value;
	}
};

} // namespace ITMLib
