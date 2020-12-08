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
#include "VolumeReduction_CUDA_PlainVoxelArray_Kernels.h"
#include "../../../Utils/Geometry/SpatialIndexConversions.h"

namespace ITMLib::internal {
template<typename TVoxel>
class VolumeReductionEngine_IndexSpecialized<TVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> {
public:
	template<typename TReduceStaticFunctor, typename TOutput, typename TRetrieveFunction>
	static TOutput ReduceUtilized_Generic(Vector3i& position, const VoxelVolume<TVoxel, PlainVoxelArray>* volume,
	                                      TRetrieveFunction&& retrieval_function,
	                                      ReductionResult<TOutput, PlainVoxelArray> ignored_value) {

		return ReduceUtilizedBlocks_Generic<TReduceStaticFunctor, TReduceStaticFunctor>(
				position, volume, retrieval_function, ignored_value);
	}

	template<typename TReduceBlockLevelStaticFunctor, typename TReduceResultLevelStaticFunctor, typename TOutput, typename TRetrievalFunction>
	static TOutput ReduceUtilizedBlocks_Generic(Vector3i& position, const VoxelVolume<TVoxel, PlainVoxelArray>* volume,
	                                            TRetrievalFunction&& retrieval_function, ReductionResult<TOutput, PlainVoxelArray> ignored_value) {

		const TVoxel* voxels = volume->GetVoxels();
		const unsigned int voxel_count = volume->index.GetMaxVoxelCount();
		const int count_of_whole_blocks = voxel_count / VOXEL_BLOCK_SIZE3;
		const int half_block_voxel_count = (VOXEL_BLOCK_SIZE3 / 2);
		dim3 cuda_block_size(half_block_voxel_count);

		ReductionResult<TOutput, PlainVoxelArray> extra_result_CPU = ignored_value;
		bool use_extra_result = false;
		if (voxel_count % VOXEL_BLOCK_SIZE3 != 0) {
			const unsigned int remainder_voxels = voxel_count % VOXEL_BLOCK_SIZE3;
			ORUtils::MemoryBlock<TVoxel> extra_voxels(VOXEL_BLOCK_SIZE3, MEMORYDEVICE_CUDA);
			extra_voxels.Clear();
			cudaMemcpy(extra_voxels.GetData(), voxels + (count_of_whole_blocks * VOXEL_BLOCK_SIZE3), sizeof(TVoxel) * remainder_voxels,
			           cudaMemcpyDeviceToDevice);
			ORUtils::MemoryBlock<ReductionResult<TOutput, PlainVoxelArray>> extra_result(1, true, true);
			dim3 cuda_single_grid_size(1);
			computeVoxelHashReduction_BlockLevel<TReduceBlockLevelStaticFunctor, TVoxel, TOutput><<<cuda_single_grid_size, cuda_block_size>>>
					(extra_result.GetData(MEMORYDEVICE_CUDA), extra_voxels.GetData(), retrieval_function);
			ORcudaKernelCheck;
			extra_result.UpdateHostFromDevice();
			extra_result_CPU = *extra_result.GetData(MEMORYDEVICE_CPU);
			extra_result_CPU.index_within_array += count_of_whole_blocks * VOXEL_BLOCK_SIZE3;
			use_extra_result = true;
		}

		auto get_normalized_count = [](int count) {
			return ceil_of_integer_quotient(count, VOXEL_BLOCK_SIZE3) * VOXEL_BLOCK_SIZE3;
		};

		const int normalized_entry_count = get_normalized_count(count_of_whole_blocks);
		int tail_length = normalized_entry_count - count_of_whole_blocks;

		ORUtils::MemoryBlock<ReductionResult<TOutput, PlainVoxelArray>> result_buffer1(normalized_entry_count, MEMORYDEVICE_CUDA);

		dim3 cuda_tail_grid_size(ceil_of_integer_quotient(tail_length, VOXEL_BLOCK_SIZE3));

		if (tail_length > 0) {
			setTailToIgnored<TOutput><<< cuda_tail_grid_size, cuda_block_size >>>
					(result_buffer1.GetData(MEMORYDEVICE_CUDA), count_of_whole_blocks, normalized_entry_count, ignored_value);
			ORcudaKernelCheck;
		}

		dim3 cuda_utilized_grid_size(count_of_whole_blocks);

		computeVoxelHashReduction_BlockLevel<TReduceBlockLevelStaticFunctor, TVoxel, TOutput><<<cuda_utilized_grid_size, cuda_block_size>>>
				(result_buffer1.GetData(MEMORYDEVICE_CUDA), voxels, retrieval_function);
		ORcudaKernelCheck;


		int output_count = count_of_whole_blocks;
		int normalized_output_count = normalized_entry_count;


		ORUtils::MemoryBlock<ReductionResult<TOutput, PlainVoxelArray>> result_buffer2(
				get_normalized_count(normalized_entry_count / half_block_voxel_count), MEMORYDEVICE_CUDA);
		/* input & output are swapped in the beginning of the loop code as opposed to the end,
		 so that output_buffer points to the final buffer after the loop finishes*/
		ORUtils::MemoryBlock<ReductionResult<TOutput, PlainVoxelArray>>* input_buffer = &result_buffer2;
		ORUtils::MemoryBlock<ReductionResult<TOutput, PlainVoxelArray>>* output_buffer = &result_buffer1;

		while (output_count > 1) {
			std::swap(input_buffer, output_buffer);
			output_count = normalized_output_count / VOXEL_BLOCK_SIZE3;
			normalized_output_count = get_normalized_count(output_count);
			tail_length = normalized_output_count - output_count;
			if (tail_length > 0) {
				cuda_tail_grid_size.x = ceil_of_integer_quotient(tail_length, VOXEL_BLOCK_SIZE3);
				setTailToIgnored<TOutput>
				<<< cuda_tail_grid_size, cuda_block_size >>>
						(output_buffer->GetData(
								MEMORYDEVICE_CUDA), output_count, normalized_output_count, ignored_value);
				ORcudaKernelCheck;
			}

			dim3 cuda_grid_size(output_count);
			computeVoxelHashReduction_ResultLevel<TReduceResultLevelStaticFunctor, TOutput>
			<<< cuda_grid_size, cuda_block_size >>>
					(output_buffer->GetData(MEMORYDEVICE_CUDA), input_buffer->GetData(
							MEMORYDEVICE_CUDA));
			ORcudaKernelCheck;
		}

		ReductionResult<TOutput, PlainVoxelArray> final_result;
		ORcudaSafeCall(cudaMemcpyAsync(&final_result, output_buffer->GetData(MEMORYDEVICE_CUDA),
		                               sizeof(ReductionResult<TOutput, PlainVoxelArray>), cudaMemcpyDeviceToHost));
		if (use_extra_result) {
			final_result = TReduceResultLevelStaticFunctor::reduce(final_result, ignored_value);
		}
		GridAlignedBox array_bounds(volume->index.GetVolumeSize(), volume->index.GetVolumeOffset());
		position = ComputePositionVectorFromLinearIndex_PlainVoxelArray(&array_bounds, static_cast<int> (final_result.index_within_array));
		return final_result.value;
	}
};
} // namespace ITMLib
