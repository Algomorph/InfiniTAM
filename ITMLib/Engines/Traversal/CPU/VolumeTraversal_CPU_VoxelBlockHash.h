//  ================================================================
//  Created by Gregory Kramida on 5/22/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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
#include "../Interface/VolumeTraversal.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../Shared/VolumeTraversal_Shared.h"
#include "../../../Utils/Analytics/IsAltered.h"

namespace ITMLib {


//======================================================================================================================
//                            CONTAINS TRAVERSAL METHODS FOR VOLUMES USING VoxelBlockHash FOR INDEXING
//======================================================================================================================

template<typename TVoxel, typename TFunctor>
inline void TraverseHashBlock_CPU(TVoxel* voxel_block, TFunctor& functor) {
	for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
		for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
			for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
				int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
				TVoxel& voxel = voxel_block[locId];
				functor(voxel);
			}
		}
	}
}

template<typename TVoxel, typename TFunctor>
inline void TraverseHashBlockWithinBounds_CPU(TVoxel* voxel_block, const Extent3D& local_bounds, TFunctor& functor) {
	for (int z = local_bounds.min_z; z < local_bounds.max_z; z++) {
		for (int y = local_bounds.min_y; y < local_bounds.max_y; y++) {
			for (int x = local_bounds.min_x; x < local_bounds.max_x; x++) {
				int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
				TVoxel& voxel = voxel_block[locId];
				functor(voxel);
			}
		}
	}
}

template<typename TVoxel, typename TFunctor>
inline void
TraverseHashBlockWithPosition_CPU(TVoxel* voxel_block, const Vector3i& block_position_voxels, TFunctor& functor) {
	for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
		for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
			for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
				int voxel_index_within_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
				Vector3i voxel_position = block_position_voxels + Vector3i(x, y, z);
				TVoxel& voxel = voxel_block[voxel_index_within_block];
				functor(voxel, voxel_position);
			}
		}
	}
}

template<typename TVoxel, typename TFunctor>
inline void TraverseHashBlockWithinBoundsWithPosition_CPU(TVoxel* voxel_block, const Vector3i& block_position_voxels,
                                                          const Extent3D& local_bounds, TFunctor& functor) {
	for (int z = local_bounds.min_z; z < local_bounds.max_z; z++) {
		for (int y = local_bounds.min_y; y < local_bounds.max_y; y++) {
			for (int x = local_bounds.min_x; x < local_bounds.max_x; x++) {
				int voxel_index_within_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
				Vector3i voxel_position = block_position_voxels + Vector3i(x, y, z);
				TVoxel& voxel = voxel_block[voxel_index_within_block];
				functor(voxel, voxel_position);
			}
		}
	}
}

//static-member-only classes are used here instead of namespaces to utilize template specialization (and maximize code reuse)
template<typename TVoxel>
class VolumeTraversalEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> {
public:
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================

	template<typename TFunctor>
	inline static void
	TraverseAll(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor) {
		TVoxel* const voxels = volume->localVBA.GetVoxelBlocks();
		const HashEntry* const hash_table = volume->index.GetEntries();
		const int hash_entry_count = volume->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor)
#endif
		for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
			const HashEntry& hash_entry = hash_table[hash_code];
			if (hash_entry.ptr < 0) continue;
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TraverseHashBlock_CPU(voxel_block, functor);
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor) {
		TVoxel* const voxels = volume->localVBA.GetVoxelBlocks();
		const HashEntry* const hash_table = volume->index.GetEntries();
		const int utilized_entry_count = volume->index.GetUtilizedHashBlockCount();
		const int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor, utilized_hash_codes)
#endif
		for (int hash_code_index = 0; hash_code_index < utilized_entry_count; hash_code_index++) {
			const HashEntry& hash_entry = hash_table[utilized_hash_codes[hash_code_index]];
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TraverseHashBlock_CPU(voxel_block, functor);
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseAll_ST(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor) {
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		const HashEntry* hash_table = volume->index.GetEntries();
		int hash_entry_count = volume->index.hashEntryCount;
		for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
			const HashEntry& hash_entry = hash_table[hash_code];
			if (hash_entry.ptr < 0) continue;
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TraverseHashBlock_CPU(voxel_block, functor);
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized_ST(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor) {
		TVoxel* const voxels = volume->localVBA.GetVoxelBlocks();
		const HashEntry* const hash_table = volume->index.GetEntries();
		const int utilized_entry_count = volume->index.GetUtilizedHashBlockCount();
		const int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();
		for (int hash_code_index = 0; hash_code_index < utilized_entry_count; hash_code_index++) {
			const HashEntry& hash_entry = hash_table[utilized_hash_codes[hash_code_index]];
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TraverseHashBlock_CPU(voxel_block, functor);
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor) {
		TVoxel* const voxels = volume->localVBA.GetVoxelBlocks();
		const HashEntry* const hash_table = volume->index.GetEntries();
		const int noTotalEntries = volume->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor)
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& hash_entry = hash_table[entryId];
			if (hash_entry.ptr < 0) continue;
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			//position of the current entry in 3D space (in voxel units)
			Vector3i block_position = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			TraverseHashBlockWithPosition_CPU(voxel_block, block_position, functor);
		}
	}
	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor) {
		TVoxel* const voxels = volume->localVBA.GetVoxelBlocks();
		const HashEntry* const hash_table = volume->index.GetEntries();
		const int utilized_entry_count = volume->index.GetUtilizedHashBlockCount();
		const int* utilized_hash_codes = volume->index.GetUtilizedBlockHashCodes();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor, utilized_hash_codes)
#endif
		for (int hash_code_index = 0; hash_code_index < utilized_entry_count; hash_code_index++) {
			const HashEntry& hash_entry = hash_table[utilized_hash_codes[hash_code_index]];
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			//position of the current entry in 3D space (in voxel units)
			Vector3i block_position = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			TraverseHashBlockWithPosition_CPU(voxel_block, block_position, functor);
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithinBounds(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor,
	                        const Vector6i& bounds) {
		TVoxel* const voxels = volume->localVBA.GetVoxelBlocks();
		const HashEntry* const hash_table = volume->index.GetEntries();
		const int noTotalEntries = volume->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor, bounds)
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& hash_entry = hash_table[entryId];
			if (hash_entry.ptr < 0) continue;
			Vector3i hash_entry_min = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hash_entry_max = hash_entry_min + Vector3i(VOXEL_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(hash_entry_min, hash_entry_max, bounds)) {
				continue;
			}
			Vector6i local_bounds = computeLocalBounds(hash_entry_min, hash_entry_max, bounds);
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TraverseHashBlockWithinBounds_CPU(voxel_block, local_bounds, functor);
		}
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithinBoundsWithPosition(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor,
	                                    Vector6i bounds) {
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		const HashEntry* hash_table = volume->index.GetEntries();
		int noTotalEntries = volume->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& hash_entry = hash_table[entryId];
			if (hash_entry.ptr < 0) continue;
			Vector3i hash_entry_min = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hash_entry_max = hash_entry_min + Vector3i(VOXEL_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(hash_entry_min, hash_entry_max, bounds)) {
				continue;
			}
			Vector6i local_bounds = computeLocalBounds(hash_entry_min, hash_entry_max, bounds);
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			//position of the current entry in 3D space (in voxel units)
			Vector3i block_position_voxels = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			TraverseHashBlockWithinBoundsWithPosition_CPU(voxel_block, block_position_voxels, local_bounds, functor);
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithinBoundsWithPositionAndHashEntry(VoxelVolume<TVoxel, VoxelBlockHash>* volume, TFunctor& functor,
	                                                Vector6i bounds) {
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		const HashEntry* hash_table = volume->index.GetEntries();
		int noTotalEntries = volume->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& hash_entry = hash_table[entryId];
			if (hash_entry.ptr < 0) continue;
			Vector3i hash_entry_min = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hash_entry_max = hash_entry_min + Vector3i(VOXEL_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(hash_entry_min, hash_entry_max, bounds)) {
				continue;
			}
			Vector6i local_bounds = computeLocalBounds(hash_entry_min, hash_entry_max, bounds);
			functor.processHashEntry(hash_entry);
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			Vector3i block_position_voxels = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			TraverseHashBlockWithinBoundsWithPosition_CPU(voxel_block, block_position_voxels, local_bounds, functor);
		}
	}

// endregion ===========================================================================================================
// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void StaticTraverseAll(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		const HashEntry* hash_table = volume->index.GetEntries();
		int noTotalEntries = volume->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& hash_entry = hash_table[entryId];
			if (hash_entry.ptr < 0) continue;
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& voxel = voxel_block[locId];
						TStaticFunctor::run(voxel);
					}
				}
			}
		}
	}

	template<typename TStaticFunctor>
	inline static void StaticTraverseAllWithPositon(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
		const HashEntry* hash_table = volume->index.GetEntries();
		int noTotalEntries = volume->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& hash_entry = hash_table[entryId];
			if (hash_entry.ptr < 0) continue;
			TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
			//position of the current entry in 3D space (in voxel units)
			Vector3i hash_block_position = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						Vector3i voxel_position = hash_block_position + Vector3i(x, y, z);
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& voxel = voxel_block[locId];
						TStaticFunctor::run(voxel, voxel_position);
					}
				}
			}
		}
	}
// endregion
};

}//namespace ITMLib