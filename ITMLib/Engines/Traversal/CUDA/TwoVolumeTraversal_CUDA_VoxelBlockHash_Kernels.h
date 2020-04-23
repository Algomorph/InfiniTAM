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
	int hash_code;
	bool primary;
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
                                             int hashEntryCount,
                                             HashPair* matchedHashPairs,
                                             UnmatchedHash* unmatchedHashes,
                                             HashMatchInfo* hashMatchInfo) {

	int hash = blockIdx.x * blockDim.x + threadIdx.x;
	if (hash > hashEntryCount) return;

	const ITMLib::HashEntry& hash_entry1 = hash_table1[hash];
	const ITMLib::HashEntry hash_entry2 = hash_table2[hash];
	int secondaryHash = hash;
	if (hash_entry1.ptr < 0) {
		if (hash_entry2.ptr < 0) {
			// neither primary nor secondary hash blocks allocated for this hash
			return;
		} else {
			int alternativePrimaryHash;
			// found no primary at hash index, but did find secondary. Ensure we have a matching primary.
			if (!FindHashAtPosition(alternativePrimaryHash, hash_entry2.pos, hash_table1)) {
				int unmatchedHashIdx = atomicAdd(&hashMatchInfo->unmatched_hash_count, 1);
				unmatchedHashes[unmatchedHashIdx].hash_code = hash;
				unmatchedHashes[unmatchedHashIdx].primary = false;
				return;
			} else {
				// found a matching primary, meaning secondary hash will be processed by a different thread
				return;
			}
		}
	}

	// the rare case where we have different positions for primary & secondary voxel block with the same index:
	// we have a hash bucket miss, find the secondary voxel block with the matching coordinates
	if (hash_entry2.pos != hash_entry1.pos) {
		if (hash_entry2.ptr >= 0) {
			int alternativePrimaryHash;
			if (!FindHashAtPosition(alternativePrimaryHash, hash_entry2.pos, hash_table1)) {
				int unmatchedHashIdx = atomicAdd(&hashMatchInfo->unmatched_hash_count, 1);
				unmatchedHashes[unmatchedHashIdx].hash_code = hash;
				unmatchedHashes[unmatchedHashIdx].primary = false;
				return;
			}
		}

		if (!FindHashAtPosition(secondaryHash, hash_entry1.pos, hash_table2)) {
			// If we cannot find the matching secondary hash, we will check whether the primary voxel block has been altered later
			int unmatchedHashIdx = atomicAdd(&hashMatchInfo->unmatched_hash_count, 1);
			unmatchedHashes[unmatchedHashIdx].hash_code = hash;
			unmatchedHashes[unmatchedHashIdx].primary = true;
			return;
		}
	}

	int matchedPairIdx = atomicAdd(&hashMatchInfo->matched_hash_count, 1);
	matchedHashPairs[matchedPairIdx].hash_code1 = hash;
	matchedHashPairs[matchedPairIdx].hash_code2 = secondaryHash;
}

// region ========================= ALL-TRUE CHECK KERNELS =============================================================

template<typename TVoxel1, typename TVoxel2>
__global__ void checkIfUnmatchedVoxelBlocksAreAltered(
		const TVoxel1* voxels1, const TVoxel2* voxels2,
		const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
		const UnmatchedHash* unmatchedHashes, const HashMatchInfo* hashMatchInfo,
		bool* alteredVoxelEncountered) {

	if (*alteredVoxelEncountered) return;

	// assume one thread block per hash block
	int hashIdx = blockIdx.x;
	if (hashIdx > hashMatchInfo->unmatched_hash_count) return;
	int hash = unmatchedHashes[hashIdx].hash_code;

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	bool voxelAltered;
	if (unmatchedHashes[hashIdx].primary) {
		voxelAltered = isAltered(voxels1[hash_table1[hash].ptr * VOXEL_BLOCK_SIZE3 + locId]);
	} else {
		voxelAltered = isAltered(voxels2[hash_table2[hash].ptr * VOXEL_BLOCK_SIZE3 + locId]);
	}
	if (voxelAltered) *alteredVoxelEncountered = true;
}


template<typename TBooleanFunctor, typename TVoxel1, typename TVoxel2>
__global__ void checkIfMatchingHashBlockVoxelsYieldTrue(
		TVoxel1* voxels1, TVoxel2* voxels2,
		const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
		const HashPair* matchedHashes, const HashMatchInfo* matchInfo, TBooleanFunctor* functor,
		bool* falseEncountered) {
	if (*falseEncountered) return;

	int hashPairIdx = blockIdx.x;
	if (hashPairIdx > matchInfo->matched_hash_count) return;
	int primaryHash = matchedHashes[hashPairIdx].hash_code1;
	int secondaryHash = matchedHashes[hashPairIdx].hash_code2;

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	if (!(*functor)(voxels1[hash_table1[primaryHash].ptr * VOXEL_BLOCK_SIZE3 + locId],
	                voxels2[hash_table2[secondaryHash].ptr * VOXEL_BLOCK_SIZE3 + locId])) {
		*falseEncountered = true;
	}
}


template<typename TBooleanFunctor, typename TVoxel1, typename TVoxel2>
__global__ void checkIfMatchingHashBlockVoxelsYieldTrue_Position(
		TVoxel1* voxels1, TVoxel2* voxels2,
		const ITMLib::HashEntry* hash_table1, const ITMLib::HashEntry* hash_table2,
		const HashPair* matchedHashes, const HashMatchInfo* matchInfo, TBooleanFunctor* functor,
		bool* falseEncountered) {
	if (*falseEncountered) return;

	int hashPairIdx = blockIdx.x;
	if (hashPairIdx > matchInfo->matched_hash_count) return;
	int primaryHash = matchedHashes[hashPairIdx].hash_code1;
	int secondaryHash = matchedHashes[hashPairIdx].hash_code2;

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	const ITMLib::HashEntry& hash_entry1 = hash_table1[primaryHash];
	// position of the current entry in 3D space in voxel units
	Vector3i hashBlockPosition = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE;
	Vector3i voxel_position = hashBlockPosition + Vector3i(x, y, z);

	if (!(*functor)(voxels1[hash_entry1.ptr * VOXEL_BLOCK_SIZE3 + locId],
	                voxels2[hash_table2[secondaryHash].ptr * VOXEL_BLOCK_SIZE3 + locId],
	                voxel_position)) {
		*falseEncountered = true;
	}
}

// endregion ===========================================================================================================

}// end anonymous namespace (CUDA kernels)