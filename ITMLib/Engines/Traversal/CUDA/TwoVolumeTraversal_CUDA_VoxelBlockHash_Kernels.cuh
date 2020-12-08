//  ================================================================
//  Created by Gregory Kramida on 1/31/20.
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
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../Shared/VolumeTraversal_Shared.h"
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"


struct HashPair {
	int hash_code1;
	int hash_code2;
};

struct UnmatchedHash {
	int hash_index;
	bool exists_in_hash_table1;
};

struct HashMatchInfo {
	int matched_hash_count;
	int unmatched_hash_count;
};

namespace {
// CUDA kernels


__device__ inline void
checkOtherHashHasMatchingEntry(ITMLib::HashEntry& hash_entry2, const ITMLib::HashEntry* hash_table2,
                               const ITMLib::HashEntry& hash_entry1, const int hash_code1) {
	if (hash_entry2.pos != hash_entry1.pos) {
		int hash_code2 = 0;
		if (!FindHashAtPosition(hash_code2, hash_entry1.pos, hash_table2)) {
			printf("Attempted traversal of volume 1 hash block %d at (%d, %d, %d), but this block is absent from volume 2.\n",
			       hash_code1, hash_entry1.pos.x, hash_entry1.pos.y, hash_entry1.pos.z);
			DIEWITHEXCEPTION_REPORTLOCATION("No hash block with corresponding position found in hash table.");
		}
		hash_entry2 = hash_table2[hash_code2];
	}
}

__device__ inline bool
getVoxelIndexesFromHashBlock(int& voxel1_index, int& voxel2_index,
                             const int hash_code1, const ITMLib::HashEntry* hash_table1,
                             const ITMLib::HashEntry* hash_table2) {
	const ITMLib::HashEntry& hash_entry1 = hash_table1[hash_code1];
	if (hash_entry1.ptr < 0) return false;
	ITMLib::HashEntry hash_entry2 = hash_table2[hash_code1];

	checkOtherHashHasMatchingEntry(hash_entry2, hash_table2, hash_entry1, hash_code1);

	int x = threadIdx.x;
	int y = threadIdx.y;
	int z = threadIdx.z;
	int linear_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	voxel1_index = hash_entry1.ptr * (VOXEL_BLOCK_SIZE3) + linear_index_in_block;
	voxel2_index = hash_entry2.ptr * (VOXEL_BLOCK_SIZE3) + linear_index_in_block;
	return true;
}

__device__ inline bool
getVoxelIndexesAndPositionFromHashBlock(int& voxel1_index, int& voxel2_index, Vector3i& voxel_position,
                                        const int hash_code1, const ITMLib::HashEntry* hash_table1,
                                        const ITMLib::HashEntry* hash_table2) {
	const ITMLib::HashEntry& hash_entry1 = hash_table1[hash_code1];
	if (hash_entry1.ptr < 0) return false;
	ITMLib::HashEntry hash_entry2 = hash_table2[hash_code1];

	checkOtherHashHasMatchingEntry(hash_entry2, hash_table2, hash_entry1, hash_code1);

	int x = threadIdx.x;
	int y = threadIdx.y;
	int z = threadIdx.z;
	int linear_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	voxel_position = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE + Vector3i(x, y, z);
	voxel1_index = hash_entry1.ptr * (VOXEL_BLOCK_SIZE3) + linear_index_in_block;
	voxel2_index = hash_entry2.ptr * (VOXEL_BLOCK_SIZE3) + linear_index_in_block;

	return true;
}

template<typename TFunctor, typename TVoxel1, typename TVoxel2>
__global__ void
traverseAll_device(TVoxel1* voxels1, TVoxel2* voxels2,
                   const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
                   TFunctor* functor) {
	int hash_code1 = blockIdx.x;
	int voxel1_index, voxel2_index;
	if (!getVoxelIndexesFromHashBlock(voxel1_index, voxel2_index, hash_code1, hash_table1, hash_table2)) return;

	TVoxel1& voxel1 = voxels1[voxel1_index];
	TVoxel2& voxel2 = voxels2[voxel2_index];

	(*functor)(voxel1, voxel2);
}

template<typename TFunctor, typename TVoxel1, typename TVoxel2>
__global__ void
traverseUtilized_device(TVoxel1* voxels1, TVoxel2* voxels2,
                        const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
                        const int* utilized_hash_codes, TFunctor* functor) {
	int hash_code1 = utilized_hash_codes[blockIdx.x];
	int voxel1_index, voxel2_index;
	getVoxelIndexesFromHashBlock(voxel1_index, voxel2_index, hash_code1, hash_table1, hash_table2);

	TVoxel1& voxel1 = voxels1[voxel1_index];
	TVoxel2& voxel2 = voxels2[voxel2_index];

	(*functor)(voxel1, voxel2);
}


template<typename TFunctor, typename TVoxel1, typename TVoxel2>
__global__ void
traverseAllWithPosition_device(TVoxel1* voxels1, TVoxel2* voxels2,
                               const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
                               TFunctor* functor) {
	int hash_code1 = blockIdx.x;
	int voxel1_index, voxel2_index;
	Vector3i voxel_position;
	if (!getVoxelIndexesAndPositionFromHashBlock(voxel1_index, voxel2_index, voxel_position, hash_code1, hash_table1,
	                                             hash_table2)) {
		return;
	}
	TVoxel1& voxel1 = voxels1[voxel1_index];
	TVoxel2& voxel2 = voxels2[voxel2_index];

	(*functor)(voxel1, voxel2, voxel_position);
}

template<typename TFunctor, typename TVoxel1, typename TVoxel2>
__global__ void
traverseUtilizedWithPosition_device(TVoxel1* voxels1, TVoxel2* voxels2,
                                    const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
                                    const int* utilized_hash_codes, TFunctor* functor) {
	int hash_code1 = utilized_hash_codes[blockIdx.x];
	int voxel1_index, voxel2_index;
	Vector3i voxel_position;
	getVoxelIndexesAndPositionFromHashBlock(voxel1_index, voxel2_index, voxel_position, hash_code1, hash_table1,
	                                        hash_table2);

	TVoxel1& voxel1 = voxels1[voxel1_index];
	TVoxel2& voxel2 = voxels2[voxel2_index];

	(*functor)(voxel1, voxel2, voxel_position);
}


__global__ void matchUpHashEntriesByPosition(const ITMLib::HashEntry* hash_table1,
                                             const ITMLib::HashEntry* hash_table2,
                                             int hash_entry_count,
                                             HashPair* matched_hash_pairs,
                                             UnmatchedHash* unmatched_hash_codes,
                                             HashMatchInfo* match_info) {

	int hash = blockIdx.x * blockDim.x + threadIdx.x;
	if (hash > hash_entry_count) return;

	const ITMLib::HashEntry& hash_entry1 = hash_table1[hash];
	const ITMLib::HashEntry hash_entry2 = hash_table2[hash];
	int hash_code2 = hash;
	if (hash_entry1.ptr < 0) {
		if (hash_entry2.ptr < 0) {
			// neither hash table 1 nor hash table 2 blocks are allocated for this hash
			return;
		} else {
			int alternative_hash_code1;
			// found no block in table 1 at specified hash index, but did find one in table 2.
			// Ensure we have a matching block in table 1 by position.
			if (!FindHashAtPosition(alternative_hash_code1, hash_entry2.pos, hash_table1)) {
				int unmatched_hash_idx = atomicAdd(&match_info->unmatched_hash_count, 1);
				unmatched_hash_codes[unmatched_hash_idx].hash_index = hash;
				unmatched_hash_codes[unmatched_hash_idx].exists_in_hash_table1 = false;
				return;
			} else {
				// found a matching table 1 block, table 2 will be inspected for this hash index by a different thread
				return;
			}
		}
	}

	// the rare case where we have different positions for table-1 & table-2 voxel block with the same index:
	// we have a hash bucket miss, find the secondary voxel block with the matching coordinates
	if (hash_entry2.pos != hash_entry1.pos) {
		if (hash_entry2.ptr >= 0) {
			int alternative_hash_code1;
			if (!FindHashAtPosition(alternative_hash_code1, hash_entry2.pos, hash_table1)) {
				int unmatched_hash_idx = atomicAdd(&match_info->unmatched_hash_count, 1);
				unmatched_hash_codes[unmatched_hash_idx].hash_index = hash;
				unmatched_hash_codes[unmatched_hash_idx].exists_in_hash_table1 = false;
				return;
			}
		}

		if (!FindHashAtPosition(hash_code2, hash_entry1.pos, hash_table2)) {
			// If we cannot find the matching secondary hash, we will check whether the table-1 voxel block has been altered later
			int unmatched_hash_idx = atomicAdd(&match_info->unmatched_hash_count, 1);
			unmatched_hash_codes[unmatched_hash_idx].hash_index = hash;
			unmatched_hash_codes[unmatched_hash_idx].exists_in_hash_table1 = true;
			return;
		}
	}

	int matched_pair_idx = atomicAdd(&match_info->matched_hash_count, 1);
	matched_hash_pairs[matched_pair_idx].hash_code1 = hash;
	matched_hash_pairs[matched_pair_idx].hash_code2 = hash_code2;
}

// region ========================= ALL-TRUE CHECK KERNELS =============================================================

template<typename TVoxel1, typename TVoxel2,
		typename TVoxelPredicate1Functor,
		typename TVoxelPredicate2Functor>
__global__ void checkIfUnmatchedVoxelBlocksAreAltered(
		const TVoxel1* voxels1, const TVoxel2* voxels2,
		const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
		const UnmatchedHash* unmatched_hash_codes, const HashMatchInfo* match_info,
		bool* altered_voxel_encountered,
		TVoxelPredicate1Functor voxel1_qualifies_for_comparison,
		TVoxelPredicate2Functor voxel2_qualifies_for_comparison) {

	if (*altered_voxel_encountered) return;

	// assume one thread block per hash_index block
	int block_index = blockIdx.x;
	if (block_index > match_info->unmatched_hash_count) return;
	int hash_index = unmatched_hash_codes[block_index].hash_index;

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int voxel_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	bool voxel_altered;
	if (unmatched_hash_codes[block_index].exists_in_hash_table1) {
		auto& voxel1 = voxels1[hash_table1[hash_index].ptr * VOXEL_BLOCK_SIZE3 + voxel_index_in_block];
		voxel_altered = voxel1_qualifies_for_comparison(voxel1) && isAltered(voxel1);
	} else {
		auto& voxel2 = voxels2[hash_table2[hash_index].ptr * VOXEL_BLOCK_SIZE3 + voxel_index_in_block];
		voxel_altered = voxel2_qualifies_for_comparison(voxel2) && isAltered(voxel2);
	}
	if (voxel_altered) *altered_voxel_encountered = true;
}

// ============ super-templated CUDA kernel in struct ====

template<bool PassPositionToFunctor>
struct HashBlockEqualityChecker;

template<>
struct HashBlockEqualityChecker<false> {
	template<typename TBooleanFunctor,
			typename TVoxel1,
			typename TVoxel2,
			typename TVoxelPredicate1Functor,
			typename TVoxelPredicate2Functor>
	__device__ static void check(
			TVoxel1* voxels1, TVoxel2* voxels2,
			const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
			const HashPair* matched_hashes, const HashMatchInfo* match_info, TBooleanFunctor* functor,
			bool* false_encountered,
			TVoxelPredicate1Functor&& voxel1_qualifies_for_comparison,
			TVoxelPredicate2Functor&& voxel2_qualifies_for_comparison) {
		if (*false_encountered) return;

		int hash_pair_idx = blockIdx.x;
		if (hash_pair_idx > match_info->matched_hash_count) return;
		int hash_code1 = matched_hashes[hash_pair_idx].hash_code1;
		int hash_code2 = matched_hashes[hash_pair_idx].hash_code2;

		int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
		int voxel_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

		auto& voxel1 = voxels1[hash_table1[hash_code1].ptr * VOXEL_BLOCK_SIZE3 + voxel_index_in_block];
		auto& voxel2 = voxels2[hash_table2[hash_code2].ptr * VOXEL_BLOCK_SIZE3 + voxel_index_in_block];

		if (voxel1_qualifies_for_comparison(voxel1) &&
		    voxel2_qualifies_for_comparison(voxel2) &&
		    !(*functor)(voxel1, voxel2)) {
			*false_encountered = true;
		}
	}
};

template<>
struct HashBlockEqualityChecker<true> {
	template<typename TBooleanFunctor,
			typename TVoxel1,
			typename TVoxel2,
			typename TVoxelPredicate1Functor,
			typename TVoxelPredicate2Functor>
	__device__ static void check(
			TVoxel1* voxels1, TVoxel2* voxels2,
			const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
			const HashPair* matched_hash_codes, const HashMatchInfo* match_info, TBooleanFunctor* functor,
			bool* false_encountered,
			TVoxelPredicate1Functor&& voxel1_qualifies_for_comparison,
			TVoxelPredicate2Functor&& voxel2_qualifies_for_comparison) {
		if (*false_encountered) return;

		int hashPairIdx = blockIdx.x;
		if (hashPairIdx > match_info->matched_hash_count) return;
		int hash_code1 = matched_hash_codes[hashPairIdx].hash_code1;
		int hash_code2 = matched_hash_codes[hashPairIdx].hash_code2;

		int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
		int voxel_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

		const ITMLib::HashEntry& hash_entry1 = hash_table1[hash_code1];
		// position of the current entry in 3D space in voxel units
		Vector3i hash_block_position = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE;
		Vector3i voxel_position = hash_block_position + Vector3i(x, y, z);

		auto& voxel1 = voxels1[hash_entry1.ptr * VOXEL_BLOCK_SIZE3 + voxel_index_in_block];
		auto& voxel2 = voxels2[hash_table2[hash_code2].ptr * VOXEL_BLOCK_SIZE3 + voxel_index_in_block];

		if (voxel1_qualifies_for_comparison(voxel1) &&
		    voxel2_qualifies_for_comparison(voxel2) &&
		    !(*functor)(voxel1, voxel2, voxel_position)) {
			*false_encountered = true;
		}
	}
};


template<bool PassPositionToFunctor,
		typename TBooleanFunctor, typename TVoxel1, typename TVoxel2,
		typename TVoxelPredicate1Functor, typename TVoxelPredicate2Functor>
__global__ void checkCorrespondingHashBlocks(
		TVoxel1* voxels1, TVoxel2* voxels2,
		const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
		const HashPair* matched_hashes, const HashMatchInfo* match_info, TBooleanFunctor* functor,
		bool* false_encountered,
		TVoxelPredicate1Functor voxel1_qualifies_for_comparison,
		TVoxelPredicate2Functor voxel2_qualifies_for_comparison) {
	HashBlockEqualityChecker<PassPositionToFunctor>::check(voxels1, voxels2, hash_table1, hash_table2, matched_hashes, match_info,
	                                                       functor, false_encountered, voxel1_qualifies_for_comparison,
	                                                       voxel2_qualifies_for_comparison);
}

// endregion ===========================================================================================================

}// end anonymous namespace (CUDA kernels)