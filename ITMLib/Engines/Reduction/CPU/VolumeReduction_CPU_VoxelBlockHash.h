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

#pragma once

#include <thread>

//threadpool
#include <threadpool11/threadpool11.hpp>

//local
#include "../Interface/VolumeReduction.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../Shared/ReductionResult.h"
#include "../../../GlobalTemplateDefines.h"

//TODO: implement public static functions that currently throw the "Not yet implemented" exception

namespace ITMLib {

template<typename TVoxel>
class VolumeReductionEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> {
public:

	/**
	 * \brief Computes reduction on some value from every voxel in the volume, e.g. minimum of some voxel field
	 * \details The position output is not always relevant, i.e. in case of accumulator reductions like the sum of some field.
	 * However, it is extremely useful when trying to either locate a particular value or choose the voxel that best satisfies some criterion,
	 * e.g. minimum or maximum value of some voxel field can be found along with its location.
	 * TRetrieveSingleStaticFunctor is used to retrieve initial values from individual voxels.
	 * TReduceBlockLevelStaticFunctor is used to reduce the result on a block level (until there is 1 result per block).
	 * TReduceResultLevelStaticFunctor is used to further reduce the block level results until there is only a single,
	 * aggregate result of the whole function.
	 *
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
	template<typename TRetrieveSingleStaticFunctor, typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor, typename TOutput>
	static TOutput ReduceUtilized(Vector3i& position, const VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                              ReductionResult<TOutput, VoxelBlockHash> ignored_value = {TOutput(0), 0u, 0}) {
		ReduceUtilized_Generic(
				position, volume,
				[](const TVoxel* block_voxels, int index_within_block) {
					return TReduceBlockLevelStaticFunctor::retrieve(block_voxels[index_within_block]);
				}, ignored_value
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
	 *
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
	template<typename TRetrieveSingleDynamicFunctor, typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor, typename TOutput>
	static TOutput ReduceUtilized(Vector3i& position, const VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                              const TRetrieveSingleDynamicFunctor& retrieve_functor,
	                              ReductionResult<TOutput, VoxelBlockHash> ignored_value = {TOutput(0), 0u, 0}) {
		ReduceUtilized_Generic(
				position, volume,
				[&retrieve_functor](const TVoxel* block_voxels, int index_within_block) {
					return retrieve_functor->retrieve(block_voxels[index_within_block]);
				}, ignored_value
		);
	}

private:

	template<typename TRetrieveFunction, typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor, typename TOutput>
	static TOutput ReduceUtilized_Generic(Vector3i& position, const VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                      TRetrieveFunction&& retrieve_function,
	                                      ReductionResult<TOutput, VoxelBlockHash> ignored_value = {TOutput(0), 0u, 0}) {

		threadpool11::pool pool;

		const auto processor_count = std::thread::hardware_concurrency();
		const int half_block_size = VOXEL_BLOCK_SIZE3 / 2;

		const int utilized_entry_count = volume->index.GetUtilizedBlockCount();
		const int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();
		const HashEntry* hash_entries = volume->index.GetEntries();
		const TVoxel* voxels = volume->GetVoxels();



		// *** we'll need to swap between two buffers for input/output during the reduction
		int first_level_result_count = utilized_entry_count * half_block_size;
		std::vector<std::future<ReductionResult<TOutput, VoxelBlockHash>>> result_buffer1(first_level_result_count);
		std::vector<std::future<ReductionResult<TOutput, VoxelBlockHash>>> result_buffer2(first_level_result_count / 2);

		// *** perform first pass over the voxels, applying the retrieval function
		for (int i_utilized_block = 0; i_utilized_block < utilized_entry_count; i_utilized_block++) {

			int utilized_hash_code = utilized_hash_codes[i_utilized_block];
			const HashEntry& entry = hash_entries[utilized_hash_code];
			const TVoxel* block_voxels = voxels[entry.ptr * VOXEL_BLOCK_SIZE3];

			for (int i_voxel_pair_in_block = 0; i_voxel_pair_in_block < half_block_size; i_voxel_pair_in_block++) {
				result_buffer1.emplace_back(
						pool.post_work<ReductionResult<TOutput, VoxelBlockHash>>(
								[&retrieve_function, &half_block_size, utilized_hash_code, block_voxels, i_voxel_pair_in_block]() {
									return TReduceBlockLevelStaticFunctor::reduce(
											{retrieve_function(block_voxels, i_voxel_pair_in_block), i_voxel_pair_in_block,
											 utilized_hash_code},
											{retrieve_function(block_voxels, i_voxel_pair_in_block + half_block_size),
											 i_voxel_pair_in_block + half_block_size, utilized_hash_code}
									);
								}
						)
				);
			}
		}
		for (int i_result = 0; i_result < first_level_result_count; i_result++) {
			result_buffer1[i_result].get();
		}

		// *** perform subsequent passes on the block level, until there is a unique result per block
		std::reference_wrapper<std::vector<std::future<ReductionResult<TOutput, VoxelBlockHash>>>> input_buffer = result_buffer1;
		std::reference_wrapper<std::vector<std::future<ReductionResult<TOutput, VoxelBlockHash>>>> output_buffer = result_buffer2;
		int previous_step = half_block_size;
		int previous_result_count = first_level_result_count;
		for (int step = previous_step >> 1; step > 0; step >>= 1) {
			const int result_count = previous_result_count >>= 1;
			for (int i_block_offset = 0; i_block_offset < previous_result_count; i_block_offset += previous_step) {
				for (int i_result_in_block = 0; i_result_in_block < step; i_result_in_block++) {
					output_buffer->emplace_back(
							pool.post_work<ReductionResult<TOutput, VoxelBlockHash>>(
									[&step, &input_buffer, i_block_offset, i_result_in_block]() {
										return TReduceBlockLevelStaticFunctor::reduce(
												input_buffer.get()[i_block_offset + i_result_in_block],
												input_buffer.get()[i_block_offset + i_result_in_block + step]
										);
									}
							)
					);
				}
			}
			for (int i_result = 0; i_result < result_count; i_result++) {
				output_buffer.get()[i_result].get();
			}
			std::swap(input_buffer, output_buffer);
			previous_result_count = result_count;
			previous_step = step;
		}

		//*** perform the final reduction on the block-result level
		// for this we have to keep the number of inputs even
		int result_count = previous_result_count;
		while (result_count > 1) {
			if (result_count % 2 == 1) {
				input_buffer.get()[result_count] = ignored_value;
				result_count += 1;
			}
			previous_result_count = result_count;
			result_count >>= 1;
			for (int i_input = 0; i_input < previous_result_count; i_input += 2) {
				output_buffer->emplace_back(
						pool.post_work<ReductionResult<TOutput, VoxelBlockHash>>(
								[&input_buffer, i_input]() {
									return TReduceResultLevelStaticFunctor::reduce(
											input_buffer.get()[i_input],
											input_buffer.get()[i_input + 1]
									);
								}
						)
				);
			}
			for (int i_result = 0; i_result < result_count; i_result++) {
				output_buffer.get()[i_result].get();
			}
			std::swap(input_buffer, output_buffer);
		}
		// undo last swap, for clarity later
		std::swap(input_buffer, output_buffer);

		ReductionResult<TOutput, VoxelBlockHash>& final_result = output_buffer.get()[0];
		HashEntry entry_with_result = volume->index.GetHashEntry(final_result.hash_code);
		position = ComputePositionVectorFromLinearIndex_VoxelBlockHash(entry_with_result.pos, static_cast<int> (final_result.index_within_block));

		return final_result.value;
	}

};

} // namespace ITMLib

