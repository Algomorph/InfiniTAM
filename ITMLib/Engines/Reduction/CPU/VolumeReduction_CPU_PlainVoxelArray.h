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
#include "../../../Objects/Volume/PlainVoxelArray.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../Shared/ReductionResult.h"
#include "../../../GlobalTemplateDefines.h"
#include "../../../Utils/Geometry/SpatialIndexConversions.h"

//TODO: implement public static functions that currently throw the "Not yet implemented" exception

namespace ITMLib {
namespace internal {
template<typename TVoxel>
class VolumeReductionEngine_IndexSpecialized<TVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> {
public:
	template<typename TReduceStaticFunctor, typename TOutput, typename TRetrieveFunction>
	static TOutput ReduceUtilized_Generic(Vector3i& position, const VoxelVolume<TVoxel, PlainVoxelArray>* volume,
	                                      TRetrieveFunction&& retrieve_function,
	                                      ReductionResult<TOutput, PlainVoxelArray> ignored_value) {
		return ReduceUtilizedBlocks_Generic<TReduceStaticFunctor, TReduceStaticFunctor>(
				position, volume, retrieve_function, ignored_value);
	}

	template<typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor, typename TOutput, typename TRetrieveFunction>
	static TOutput ReduceUtilizedBlocks_Generic(Vector3i& position, const VoxelVolume<TVoxel, PlainVoxelArray>* volume,
	                                            TRetrieveFunction&& retrieve_function,
	                                            ReductionResult<TOutput, PlainVoxelArray> ignored_value) {
		const TVoxel* voxels = volume->GetVoxels();
		const unsigned int voxel_count = volume->index.GetMaxVoxelCount();

		auto final_result = ReduceUtilizedBlocks_Generic_Recursive<TReduceBlockLevelStaticFunctor, TReduceResultLevelStaticFunctor>(
				voxels, voxel_count, retrieve_function, ignored_value);

		GridAlignedBox array_bounds(volume->index.GetVolumeSize(), volume->index.GetVolumeOffset());
		position = ComputePositionVectorFromLinearIndex_PlainVoxelArray(&array_bounds, static_cast<int> (final_result.index_within_array));

		return final_result.value;
	}

private:
	template<typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor, typename TOutput, typename TRetrieveFunction>
	static ReductionResult<TOutput, PlainVoxelArray>
	ReduceUtilizedBlocks_Generic_Recursive(const TVoxel* voxels, const unsigned int voxel_count,
	                                       TRetrieveFunction&& retrieve_function,
	                                       ReductionResult<TOutput, PlainVoxelArray> ignored_value) {
		constexpr int half_block_voxel_count = VOXEL_BLOCK_SIZE3 / 2;
		const int count_of_whole_blocks = voxel_count / VOXEL_BLOCK_SIZE3;

		ReductionResult<TOutput, PlainVoxelArray> extra_result = ignored_value;
		bool use_extra_result = false;
		if (voxel_count % VOXEL_BLOCK_SIZE3 != 0) {
			const unsigned int remainder_voxels = voxel_count % VOXEL_BLOCK_SIZE3;
			ORUtils::MemoryBlock<TVoxel> extra_voxels(VOXEL_BLOCK_SIZE3, MEMORYDEVICE_CPU);
			extra_voxels.Clear();
			memcpy(extra_voxels.GetData(), voxels + (count_of_whole_blocks * VOXEL_BLOCK_SIZE3), sizeof(TVoxel) * remainder_voxels);
			extra_result = ReduceUtilizedBlocks_Generic_Recursive<TReduceBlockLevelStaticFunctor, TReduceResultLevelStaticFunctor>(
					extra_voxels.GetData(MEMORYDEVICE_CPU), VOXEL_BLOCK_SIZE3, retrieve_function, ignored_value);
			extra_result.index_within_array += count_of_whole_blocks * VOXEL_BLOCK_SIZE3;
			use_extra_result = true;
		}

		// *** we'll need to swap between two buffers for input/output during the reduction
		const int first_level_result_count = count_of_whole_blocks * half_block_voxel_count;
		std::vector<ReductionResult<TOutput, PlainVoxelArray>> result_buffer1(first_level_result_count);
		std::vector<ReductionResult<TOutput, PlainVoxelArray>> result_buffer2(first_level_result_count / 2);

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) firstprivate(count_of_whole_blocks, retrieve_function) shared(voxels, result_buffer1)
#endif
		for (int i_block = 0; i_block < count_of_whole_blocks; i_block++) {
			const int first_voxel_index = i_block * VOXEL_BLOCK_SIZE3;
			const TVoxel* block_voxels = voxels + first_voxel_index;
			int i_output = i_block * half_block_voxel_count;

			for (int i_voxel_pair_in_block = 0; i_voxel_pair_in_block < half_block_voxel_count; i_voxel_pair_in_block++, i_output++) {
				const int i_voxel = first_voxel_index + i_voxel_pair_in_block;
				result_buffer1[i_output] = TReduceBlockLevelStaticFunctor::reduce(
						{retrieve_function(block_voxels, i_voxel_pair_in_block),
						 static_cast<unsigned int>(i_voxel)},
						{retrieve_function(block_voxels, i_voxel_pair_in_block + half_block_voxel_count),
						 static_cast<unsigned int>(i_voxel + half_block_voxel_count)}
				);
			}
		}

		// *** perform subsequent passes on the block level, until there is a unique result per block
		std::reference_wrapper<std::vector<ReductionResult<TOutput, PlainVoxelArray>>> input_buffer = result_buffer1;
		std::reference_wrapper<std::vector<ReductionResult<TOutput, PlainVoxelArray>>> output_buffer = result_buffer2;

		int previous_step = half_block_voxel_count;
		int previous_result_count = first_level_result_count;
		for (int step = previous_step >> 1; step > 0; step >>= 1) {
			const int result_count = previous_result_count >> 1;

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) firstprivate(count_of_whole_blocks, previous_step, step) shared(input_buffer, output_buffer)
#endif
			for (int i_utilized_block = 0; i_utilized_block < count_of_whole_blocks; i_utilized_block++) {
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
			result_count >>= 1u;
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
		auto final_result = output_buffer.get()[0];
		if (use_extra_result) {
			final_result = TReduceResultLevelStaticFunctor::reduce(extra_result, final_result);
		}
		return final_result;
	}

}; // class VolumeReductionEngine_IndexSpecialized<TVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>

} // namespace internal
} // namespace ITMLib