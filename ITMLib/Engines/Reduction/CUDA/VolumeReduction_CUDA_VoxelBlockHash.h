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
#include "../../../Utils/Math.h"

namespace ITMLib {

template<typename TVoxel>
class VolumeReductionEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> {
public:

	/**
	 * \brief Computes reduction on some value from every voxel in the volume, e.g. minimum of some voxel field
	 * \details The position output is not always relevant, i.e. in case of accumulator reductions like the sum of some field.
	 * However, it is extremely useful when trying to either locate a particular value or choose the voxel that best satisfies some criterion,
	 * e.g. minimum or maximum value of some voxel field can be found along with its location.
	 * TRetrieveSingleStaticFunctor is used to retrieve initial values from individual voxels.
	 * TReduceBlockLevelStaticFunctor is used to reduce the result on a block level (until there is 1 result per block).
	 * TReduceResultLevelStaticFunctor is used to further reduce the block level results until there is only a single,
	 * \tparam TOutput type of the output value, e.g. float when computing minimum of some voxel float field
	 * \tparam TRetrieveSingleStaticFunctor a function object with a static function which accepts a single TVoxel as an argument and returns a TOutput value based on this TVoxel.
	 * \tparam TReduceBlockLevelStaticFunctor a function object with a static function which accepts two ReductionResult<TOutput> objects and returns a single ReductionResult<TOutput> object
	 * \param position the position of the voxel based on the indices produced by TReduceBlockLevelStaticFunctor when comparing each ReductionResult pair.
	 * \param volume the volume to run the reduction over
	 * \param ignored_value this is a sample value-index-hash triplet that will be ignored / won't skew result of the operation when issued
	 * to the reduction algorithm. For instance, when seeking a minimum of a float field, it makes sense to set this to {FLT_MAX, 0, -1}.
	 * This is necessary to normalize input sizes.
	 * \return end result based on all voxels in the volume.
	 */
	template<typename TRetrieveSingleStaticFunctor, typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor,  typename TOutput>
	static TOutput ReduceUtilized(Vector3i& position, const VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                              ReductionResult<TOutput, VoxelBlockHash> ignored_value = {TOutput(0), 0u, 0}) {
		return ReduceUtilized_Generic<TReduceResultLevelStaticFunctor, TOutput>(
				position, volume, ignored_value,
				[](dim3 cuda_utilized_grid_size, dim3 cuda_block_size,
				   ReductionResult<TOutput, VoxelBlockHash>* result_buffer_device, const TVoxel* voxels,
				   const HashEntry* hash_entries, const int* utilized_hash_codes) {
					computeVoxelHashReduction_BlockLevel_Static<TVoxel, TRetrieveSingleStaticFunctor, TReduceBlockLevelStaticFunctor, TOutput>
							<< < cuda_utilized_grid_size, cuda_block_size >> >
					                                      (result_buffer_device, voxels, hash_entries, utilized_hash_codes);
				}

		);
	}

	/**
	 * \brief Computes reduction on some value from every voxel in the volume, e.g. minimum of some voxel field
	 * \details The position output is not always relevant, i.e. in case of accumulator reductions like the sum of some field.
	 * However, it is extremely useful when trying to either locate a particular value or choose the voxel that best satisfies some criterion,
	 * e.g. minimum or maximum value of some voxel field can be found along with its location.
	 * TRetrieveSingleDynamicFunctor is used to retrieve initial values from individual voxels.
	 * TReduceBlockLevelStaticFunctor is used to reduce the result on a block level (until there is 1 result per block).
	 * TReduceResultLevelStaticFunctor is used to further reduce the block level results until there is only a single,
	 * aggregate result of the whole function.
	 * \tparam TOutput type of the output value, e.g. float when computing minimum of some voxel float field
	 * \tparam TRetrieveSingleDynamicFunctor a function object with a retrieve public member function which accepts a
	 *  single TVoxel as an argument and returns a TOutput value based on this TVoxel.
	 * \tparam TReduceBlockLevelStaticFunctor a function object with a static function which accepts two ReductionResult<TOutput> objects and returns a single ReductionResult<TOutput> object
	 * \tparam TReduceResultLevelStaticFunctor a function object with a static function which accepts two ReductionResult<TOutput> objects and returns a single ReductionResult<TOutput> object
	 * \param position the position of the voxel based on the indices produced by TReduceBlockLevelStaticFunctor when comparing each ReductionResult pair.
	 * \param volume the volume to run the reduction over
	 * \param retrieve_functor functor object to retrieve initial TOutput value from individual voxels
	 * \param ignored_value this is a sample value-index-hash triplet that will be ignored / won't skew result of the operation when issued
	 * to the reduction algorithm. For instance, when seeking a minimum of a float field, it makes sense to set this to {FLT_MAX, UINT_MAX}.
	 * This is necessary to normalize input sizes.
	 * \return end result based on all voxels in the volume.
	 */
	template<typename TRetrieveSingleDynamicFunctor, typename TReduceBlockLevelStaticFunctor,
	        typename TReduceResultLevelStaticFunctor, typename TOutput>
	static TOutput ReduceUtilized(Vector3i& position, const VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                              const TRetrieveSingleDynamicFunctor& retrieve_functor,
	                              ReductionResult<TOutput, VoxelBlockHash> ignored_value = {TOutput(0), 0u, 0}
	) {
		return ReduceUtilized_Generic<TReduceResultLevelStaticFunctor, TOutput>(
				position, volume, ignored_value,
				[&retrieve_functor](dim3 cuda_utilized_grid_size, dim3 cuda_block_size,
				                    ReductionResult<TOutput, VoxelBlockHash>* result_buffer_device,
				                    const TVoxel* voxels,
				                    const HashEntry* hash_entries, const int* utilized_hash_codes) {

					// transfer functor from RAM to VRAM
					TRetrieveSingleDynamicFunctor* retrieve_functor_device = nullptr;
					ORcudaSafeCall(
							cudaMalloc((void**) &retrieve_functor_device, sizeof(TRetrieveSingleDynamicFunctor)));
					ORcudaSafeCall(cudaMemcpy(retrieve_functor_device, &retrieve_functor,
					                          sizeof(TRetrieveSingleDynamicFunctor), cudaMemcpyHostToDevice));

					computeVoxelHashReduction_BlockLevel_Dynamic<TVoxel, TRetrieveSingleDynamicFunctor, TReduceBlockLevelStaticFunctor, TOutput>
							<< < cuda_utilized_grid_size, cuda_block_size >> >
					                                      (result_buffer_device, voxels, hash_entries, utilized_hash_codes, retrieve_functor_device);
				}
		);
	}

private:

	template<typename TReduceResultLevelStaticFunctor, typename TOutput, typename BlockLevelReductionFunction>
	static TOutput ReduceUtilized_Generic(Vector3i& position, const VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                      ReductionResult<TOutput, VoxelBlockHash> ignored_value,
	                                      BlockLevelReductionFunction&& function) {
		const int utilized_entry_count = volume->index.GetUtilizedHashBlockCount();
		const int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();
		const HashEntry* hash_entries = volume->index.GetEntries();
		const TVoxel* voxels = volume->voxels.GetVoxelBlocks();

		const int half_block_voxel_count = (VOXEL_BLOCK_SIZE3 / 2);

		auto getNormalizedCount = [](int count) {
			return ceil_of_integer_quotient(count, VOXEL_BLOCK_SIZE3) * VOXEL_BLOCK_SIZE3;
		};

		const int normalized_entry_count = getNormalizedCount(utilized_entry_count);
		int tail_length = normalized_entry_count - utilized_entry_count;

		ORUtils::MemoryBlock<ReductionResult<TOutput, VoxelBlockHash>> result_buffer1(normalized_entry_count,
		                                                                              MEMORYDEVICE_CUDA);
		dim3 cuda_block_size(half_block_voxel_count);
		dim3 cuda_tail_grid_size(ceil_of_integer_quotient(tail_length, VOXEL_BLOCK_SIZE3));

		if(tail_length > 0){
			setTailToIgnored<TOutput>
					<< < cuda_tail_grid_size, cuda_block_size >> >
			                                  (result_buffer1.GetData(
					                                  MEMORYDEVICE_CUDA), utilized_entry_count, normalized_entry_count, ignored_value);
			ORcudaKernelCheck;
		}

		dim3 cuda_utilized_grid_size(utilized_entry_count);

		std::forward<BlockLevelReductionFunction>(function)(cuda_utilized_grid_size, cuda_block_size,
		                                                    result_buffer1.GetData(MEMORYDEVICE_CUDA), voxels,
		                                                    hash_entries, utilized_hash_codes);


		int output_count = utilized_entry_count;
		int normalized_output_count = normalized_entry_count;



		ORUtils::MemoryBlock<ReductionResult<TOutput, VoxelBlockHash>> result_buffer2(
				getNormalizedCount(normalized_entry_count / half_block_voxel_count),MEMORYDEVICE_CUDA);
		/* input & output are swapped in the beginning of the loop code as opposed to the end,
		 so that output_buffer points to the final buffer after the loop finishes*/
		ORUtils::MemoryBlock<ReductionResult<TOutput, VoxelBlockHash>>* input_buffer = &result_buffer2;
		ORUtils::MemoryBlock<ReductionResult<TOutput, VoxelBlockHash>>* output_buffer = &result_buffer1;

		while (output_count > 1) {
			std::swap(input_buffer, output_buffer);
			output_count = normalized_output_count / VOXEL_BLOCK_SIZE3;
			normalized_output_count = getNormalizedCount(output_count);
			tail_length = normalized_output_count - output_count;
			if(tail_length > 0) {
				cuda_tail_grid_size.x = ceil_of_integer_quotient(tail_length, VOXEL_BLOCK_SIZE3);
				setTailToIgnored<TOutput>
						<< < cuda_tail_grid_size, cuda_block_size >> >
				                                  (output_buffer->GetData(
						                                  MEMORYDEVICE_CUDA), output_count, normalized_output_count, ignored_value);
				ORcudaKernelCheck;
			}

			dim3 cuda_grid_size(output_count);
			computeVoxelHashReduction_ResultLevel<TReduceResultLevelStaticFunctor, TOutput>
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
