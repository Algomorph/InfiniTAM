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

namespace ITMLib {

namespace internal {
template<typename TVoxel>
class VolumeReductionEngine_IndexSpecialized<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> {
public:
	template<typename TReduceStaticFunctor, typename TOutput, typename TRetrieveFunction>
	static TOutput ReduceUtilized_Generic(Vector3i& position, const VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                      TRetrieveFunction&& retrieve_function,
	                                      ReductionResult<TOutput, VoxelBlockHash> ignored_value) {
		return ReduceUtilizedBlocks_Generic<TReduceStaticFunctor, TReduceStaticFunctor>(
				position, volume, retrieve_function, ignored_value);
	}

	template<typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor, typename TOutput, typename TRetrieveFunction>
	static TOutput ReduceUtilizedBlocks_Generic(Vector3i& position, const VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                            TRetrieveFunction&& retrieve_function,
	                                            ReductionResult<TOutput, VoxelBlockHash> ignored_value) {
		const int half_block_size = VOXEL_BLOCK_SIZE3 / 2;

		const int utilized_entry_count = volume->index.GetUtilizedBlockCount();
		const int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();
		const HashEntry* hash_entries = volume->index.GetEntries();
		const TVoxel* voxels = volume->GetVoxels();


		// *** we'll need to swap between two buffers for input/output during the reduction
		const int first_level_result_count = utilized_entry_count * half_block_size;
		std::vector<ReductionResult<TOutput, VoxelBlockHash>> result_buffer1(first_level_result_count);
		std::vector<ReductionResult<TOutput, VoxelBlockHash>> result_buffer2(first_level_result_count / 2);

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) firstprivate(utilized_entry_count, retrieve_function) shared(utilized_hash_codes, hash_entries, voxels, result_buffer1)
#endif
		for (int i_utilized_block = 0; i_utilized_block < utilized_entry_count; i_utilized_block++) {
			int utilized_hash_code = utilized_hash_codes[i_utilized_block];
			const HashEntry& entry = hash_entries[utilized_hash_code];
			const TVoxel* block_voxels = voxels + (entry.ptr * VOXEL_BLOCK_SIZE3);
			int i_output = i_utilized_block * half_block_size;
			for (int i_voxel_pair_in_block = 0; i_voxel_pair_in_block < half_block_size; i_voxel_pair_in_block++, i_output++) {
				result_buffer1[i_output] = TReduceBlockLevelStaticFunctor::reduce(
						{retrieve_function(block_voxels, i_voxel_pair_in_block),
						 static_cast<unsigned int>(i_voxel_pair_in_block),
						 utilized_hash_code},
						{retrieve_function(block_voxels, i_voxel_pair_in_block + half_block_size),
						 static_cast<unsigned int>(i_voxel_pair_in_block + half_block_size),
						 utilized_hash_code});
			}
		}

		// *** perform subsequent passes on the block level, until there is a unique result per block
		std::reference_wrapper<std::vector<ReductionResult<TOutput, VoxelBlockHash>>> input_buffer = result_buffer1;
		std::reference_wrapper<std::vector<ReductionResult<TOutput, VoxelBlockHash>>> output_buffer = result_buffer2;

		int previous_step = half_block_size;
		int previous_result_count = first_level_result_count;
		for (int step = previous_step >> 1; step > 0; step >>= 1) {
			const int result_count = previous_result_count >> 1;

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) firstprivate(utilized_entry_count, previous_step, step) shared(input_buffer, output_buffer)
#endif
			for (int i_utilized_block = 0; i_utilized_block < utilized_entry_count; i_utilized_block++) {
				const int i_input = i_utilized_block * previous_step;
				int i_output = i_utilized_block * step;
				for (int i_result_in_block = 0; i_result_in_block < step; i_result_in_block++, i_output++) {
					output_buffer.get()[i_output] =
							TReduceBlockLevelStaticFunctor::reduce(
									input_buffer.get()[i_input + i_result_in_block],
									input_buffer.get()[i_input + i_result_in_block + step]
							);
				}
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
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) firstprivate(result_count) shared(input_buffer, output_buffer)
#endif
			for (int i_output = 0; i_output < result_count; i_output++) {
				int i_input = i_output * 2;
				output_buffer.get()[i_output] =
						TReduceResultLevelStaticFunctor::reduce(
								input_buffer.get()[i_input],
								input_buffer.get()[i_input + 1]
						);
			}
			if (result_count > 1) {
				std::swap(input_buffer, output_buffer);
			}
		}

		ReductionResult<TOutput, VoxelBlockHash> final_result = output_buffer.get()[0];
		HashEntry entry_with_result = volume->index.GetHashEntry(final_result.hash_code);
		position = ComputePositionVectorFromLinearIndex_VoxelBlockHash(entry_with_result.pos, static_cast<int> (final_result.index_within_block));

		return final_result.value;


	}

};
} // namespace internal
} // namespace ITMLib

