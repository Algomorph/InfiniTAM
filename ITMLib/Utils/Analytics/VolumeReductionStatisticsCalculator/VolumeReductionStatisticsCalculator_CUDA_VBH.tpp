//  ================================================================
//  Created by Gregory Kramida on 2/21/20.
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

//local

#include "VolumeReductionStatisticsCalculator_CUDA_VBH.h"
#include "VolumeReductionStatisticsCalculator_CUDA_VBH_Kernels.h"
#include "../../../Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"
#include "../../Geometry/SpatialIndexConversions.h"
#include "../../CLionCudaSyntax.h"


using namespace ITMLib;

template<typename TVoxel>
void
VolumeReductionStatisticsCalculator<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::ComputeWarpUpdateMax(
		float& max, Vector3i& position, VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	const int utilized_entry_count = volume->index.GetUtilizedHashBlockCount();
	const int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();
	const HashEntry* hash_entries = volume->index.GetEntries();
	TVoxel* voxels = volume->voxels.GetVoxelBlocks();

	ORUtils::MemoryBlock<ValueAndIndex<float>> block_results(utilized_entry_count, true, true);

	dim3 cuda_block_size(VOXEL_BLOCK_SIZE3 / 2);
	dim3 cuda_grid_size(utilized_entry_count);

	const auto computeMaxWarpUpdateAndItsPositionForEachBlock = computeVoxelHashReduction
			<TVoxel, TRetreiveWarpLengthFunctor<TVoxel, WARP_UPDATE>, TReduceWarpLengthStatisticFunctor<TVoxel, WARP_UPDATE, MAXIMUM>, float>;

	computeMaxWarpUpdateAndItsPositionForEachBlock
			<< < cuda_grid_size, cuda_block_size >> >
	                             (block_results.GetData(MEMORYDEVICE_CUDA), voxels, hash_entries, utilized_hash_codes);

	block_results.UpdateHostFromDevice();
	ValueAndIndex<float>* block_results_CPU = block_results.GetData(MEMORYDEVICE_CPU);

	/* I'm assuming here that the utilized block list is small, so I'm just traversing the results on the CPU serially.
	 * If the assumption doesn't hold, we should add another CUDA-driven reduction step that takes the block results as input.
	 * */
	max = 0.0f;
	int max_utilized_entry_index = 0;
	for (int i_utilized_entry = 0; i_utilized_entry < utilized_entry_count; i_utilized_entry++) {
		ValueAndIndex<float>& block_result = block_results_CPU[i_utilized_entry];
		if (block_result.value > max) {
			max = block_result.value;
			max_utilized_entry_index = i_utilized_entry;
		}
	}
	HashEntry entry_with_max = volume->index.GetUtilizedHashEntryAtIndex(max_utilized_entry_index);

	position = ComputePositionVectorFromLinearIndex_VoxelBlockHash(entry_with_max.pos,
	                                                               static_cast<int> (block_results_CPU[max_utilized_entry_index].index_within_block));

}
