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

using namespace ITMLib;

template<typename TVoxel>
void
VolumeReductionStatisticsCalculator<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::ComputeWarpUpdateLengthMax(
		float& max, Vector3i& position, VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	const int utilized_entry_count = volume->index.GetUtilizedHashBlockCount();
	const int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();
	HashEntry* hash_entries = volume->index.GetEntries();
	TVoxel* voxels = volume->localVBA.GetVoxelBlocks();

	ORUtils::MemoryBlock<ValueAndIndex<float>> block_results(utilized_entry_count,true,true);

	dim3 cuda_block_size(VOXEL_BLOCK_SIZE3/2);
	dim3 cuda_grid_size(utilized_entry_count);

	computeVoxelHashReduction<TVoxel, TRetreiveWarpLengthFunctor<

}
