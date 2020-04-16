//  ================================================================
//  Created by Gregory Kramida on 9/25/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"

namespace ITMLib {
// region ==================================== Offset Warping Functor =====================================================

template<typename TVoxel, typename TIndex, bool hasCumulativeWarp>
struct OffsetWarpsFunctor;

template<typename TVoxel>
struct OffsetWarpsFunctor<TVoxel, VoxelBlockHash, true> {
	static void OffsetWarps(VoxelVolume <TVoxel, VoxelBlockHash>* volume, Vector3f offset) {
		TVoxel* voxels = volume->GetVoxelBlocks();
		const HashEntry* hash_table = volume->index.GetEntries();
		int entry_count = volume->index.hash_entry_count;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash_code = 0; hash_code < entry_count; hash_code++) {
			const HashEntry& hash_entry = hash_table[hash_code];
			if (hash_entry.ptr < 0) continue;
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int linear_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						voxel_block[linear_index_in_block].warp += offset;
					}
				}
			}
		}
	}
};


template<typename TVoxel>
struct OffsetWarpsFunctor<TVoxel, PlainVoxelArray, true> {
	static void OffsetWarps(VoxelVolume <TVoxel, PlainVoxelArray>* volume, Vector3f offset) {
		TVoxel* voxels = volume->GetVoxelBlocks();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linear_index_in_array = 0;
		     linear_index_in_array < volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y *
		                             volume->index.GetVolumeSize().z; ++linear_index_in_array) {
			voxels[linear_index_in_array].warp += offset;
		}
	}
};

template<typename TVoxel>
struct OffsetWarpsFunctor<TVoxel, VoxelBlockHash, false> {
	static void OffsetWarps(VoxelVolume <TVoxel, VoxelBlockHash>* volume, Vector3f offset) {
		DIEWITHEXCEPTION_REPORTLOCATION("Warps not defined for scene of using this voxel type.");
	}
};
template<typename TVoxel>
struct OffsetWarpsFunctor<TVoxel, PlainVoxelArray, false> {
	static void OffsetWarps(VoxelVolume <TVoxel, PlainVoxelArray>* volume, Vector3f offset) {
		DIEWITHEXCEPTION_REPORTLOCATION("Warps not defined for scene of using this voxel type.");
	}
};


// endregion

} // namespace ITMLib