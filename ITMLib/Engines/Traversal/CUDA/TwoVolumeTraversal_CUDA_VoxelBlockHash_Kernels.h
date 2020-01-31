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
	int primaryHash;
	int secondaryHash;
};

struct UnmatchedHash {
	int hash;
	bool primary;
};

struct HashMatchInfo {
	int matchedHashCount;
	int unmatchedHashCount;
};

namespace {
// CUDA kernels

template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void
dualVoxelTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                          const ITMHashEntry* primaryHashTable, const ITMHashEntry* secondaryHashTable,
                          TFunctor* functor) {
	int hashCode = blockIdx.x;

	const ITMHashEntry& primaryHashEntry = primaryHashTable[hashCode];
	if (primaryHashEntry.ptr < 0) return;
	ITMHashEntry secondaryHashEntry = secondaryHashTable[hashCode];

	if (secondaryHashEntry.pos != primaryHashEntry.pos) {
		int secondaryHashCode = 0;
		if(!FindHashAtPosition(secondaryHashCode, primaryHashEntry.pos, secondaryHashTable)){
			DIEWITHEXCEPTION_REPORTLOCATION("No hash block with corresponding position found in hash table.");
		}
		secondaryHashEntry = secondaryHashTable[secondaryHashCode];
	}

	int x = threadIdx.x;
	int y = threadIdx.y;
	int z = threadIdx.z;
	int linearIndexInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	TVoxelPrimary& voxelPrimary = primaryVoxels[primaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3) + linearIndexInBlock];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[secondaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3) + linearIndexInBlock];

	(*functor)(voxelPrimary, voxelSecondary);
}

template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void
dualVoxelPositionTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                                  const ITMHashEntry* primaryHashTable, const ITMHashEntry* secondaryHashTable,
                                  TFunctor* functor) {
	int hashCode = blockIdx.x;

	const ITMHashEntry& primaryHashEntry = primaryHashTable[hashCode];
	if (primaryHashEntry.ptr < 0) return;
	ITMHashEntry secondaryHashEntry = secondaryHashTable[hashCode];

	if (secondaryHashEntry.pos != primaryHashEntry.pos) {
		int secondaryHashCode = 0;
		if(!FindHashAtPosition(secondaryHashCode, primaryHashEntry.pos, secondaryHashTable)){
			DIEWITHEXCEPTION_REPORTLOCATION("No hash block with corresponding position found in hash table.");
		}
		secondaryHashEntry = secondaryHashTable[secondaryHashCode];
	}

	int x = threadIdx.x;
	int y = threadIdx.y;
	int z = threadIdx.z;
	int linearIndexInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	// position of the current voxel in 3D space in voxel units
	Vector3i voxelPosition = primaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE + Vector3i(x, y, z);
	TVoxelPrimary& voxelPrimary = primaryVoxels[primaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3) + linearIndexInBlock];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[secondaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3) + linearIndexInBlock];

	(*functor)(voxelPrimary, voxelSecondary, voxelPosition);
}


__global__ void matchUpHashEntriesByPosition(const ITMHashEntry* primaryHashTable,
                                             const ITMHashEntry* secondaryHashTable,
                                             int hashEntryCount,
                                             HashPair* matchedHashPairs,
                                             UnmatchedHash* unmatchedHashes,
                                             HashMatchInfo* hashMatchInfo) {

	int hash = blockIdx.x * blockDim.x + threadIdx.x;
	if (hash > hashEntryCount) return;

	const ITMHashEntry& primaryHashEntry = primaryHashTable[hash];
	const ITMHashEntry secondaryHashEntry = secondaryHashTable[hash];
	int secondaryHash = hash;
	if (primaryHashEntry.ptr < 0) {
		if (secondaryHashEntry.ptr < 0) {
			// neither primary nor secondary hash blocks allocated for this hash
			return;
		} else {
			int alternativePrimaryHash;
			// found no primary at hash index, but did find secondary. Ensure we have a matching primary.
			if (!FindHashAtPosition(alternativePrimaryHash, secondaryHashEntry.pos, primaryHashTable)) {
				int unmatchedHashIdx = atomicAdd(&hashMatchInfo->unmatchedHashCount, 1);
				unmatchedHashes[unmatchedHashIdx].hash = hash;
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
	if (secondaryHashEntry.pos != primaryHashEntry.pos) {
		if (secondaryHashEntry.ptr >= 0) {
			int alternativePrimaryHash;
			if (!FindHashAtPosition(alternativePrimaryHash, secondaryHashEntry.pos, primaryHashTable)) {
				int unmatchedHashIdx = atomicAdd(&hashMatchInfo->unmatchedHashCount, 1);
				unmatchedHashes[unmatchedHashIdx].hash = hash;
				unmatchedHashes[unmatchedHashIdx].primary = false;
				return;
			}
		}

		if (!FindHashAtPosition(secondaryHash, primaryHashEntry.pos, secondaryHashTable)) {
			// If we cannot find the matching secondary hash, we will check whether the primary voxel block has been altered later
			int unmatchedHashIdx = atomicAdd(&hashMatchInfo->unmatchedHashCount, 1);
			unmatchedHashes[unmatchedHashIdx].hash = hash;
			unmatchedHashes[unmatchedHashIdx].primary = true;
			return;
		}
	}

	int matchedPairIdx = atomicAdd(&hashMatchInfo->matchedHashCount, 1);
	matchedHashPairs[matchedPairIdx].primaryHash = hash;
	matchedHashPairs[matchedPairIdx].secondaryHash = secondaryHash;
}

// region ========================= ALL-TRUE CHECK KERNELS =============================================================

template<typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void checkIfUnmatchedVoxelBlocksAreAltered(
		const TVoxelPrimary* primaryVoxels, const TVoxelSecondary* secondaryVoxels,
		const ITMHashEntry* primaryHashTable, const ITMHashEntry* secondaryHashTable,
		const UnmatchedHash* unmatchedHashes, const HashMatchInfo* hashMatchInfo,
		bool* alteredVoxelEncountered) {

	if (*alteredVoxelEncountered) return;

	// assume one thread block per hash block
	int hashIdx = blockIdx.x;
	if (hashIdx > hashMatchInfo->unmatchedHashCount) return;
	int hash = unmatchedHashes[hashIdx].hash;

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	bool voxelAltered;
	if (unmatchedHashes[hashIdx].primary) {
		voxelAltered = isAltered(primaryVoxels[primaryHashTable[hash].ptr * VOXEL_BLOCK_SIZE3 + locId]);
	} else {
		voxelAltered = isAltered(secondaryVoxels[secondaryHashTable[hash].ptr * VOXEL_BLOCK_SIZE3 + locId]);
	}
	if (voxelAltered) *alteredVoxelEncountered = true;
}


template<typename TBooleanFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void checkIfMatchingHashBlockVoxelsYieldTrue(
		TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
		const ITMHashEntry* primaryHashTable, const ITMHashEntry* secondaryHashTable,
		const HashPair* matchedHashes, const HashMatchInfo* matchInfo, TBooleanFunctor* functor,
		bool* falseEncountered) {
	if (*falseEncountered) return;

	int hashPairIdx = blockIdx.x;
	if (hashPairIdx > matchInfo->matchedHashCount) return;
	int primaryHash = matchedHashes[hashPairIdx].primaryHash;
	int secondaryHash = matchedHashes[hashPairIdx].secondaryHash;

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	if (!(*functor)(primaryVoxels[primaryHashTable[primaryHash].ptr * VOXEL_BLOCK_SIZE3 + locId],
	                secondaryVoxels[secondaryHashTable[secondaryHash].ptr * VOXEL_BLOCK_SIZE3 + locId])) {
		*falseEncountered = true;
	}
}


template<typename TBooleanFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void checkIfMatchingHashBlockVoxelsYieldTrue_Position(
		TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
		const ITMHashEntry* primaryHashTable, const ITMHashEntry* secondaryHashTable,
		const HashPair* matchedHashes, const HashMatchInfo* matchInfo, TBooleanFunctor* functor,
		bool* falseEncountered) {
	if (*falseEncountered) return;

	int hashPairIdx = blockIdx.x;
	if (hashPairIdx > matchInfo->matchedHashCount) return;
	int primaryHash = matchedHashes[hashPairIdx].primaryHash;
	int secondaryHash = matchedHashes[hashPairIdx].secondaryHash;

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	const ITMHashEntry& primaryHashEntry = primaryHashTable[primaryHash];
	// position of the current entry in 3D space in voxel units
	Vector3i hashBlockPosition = primaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
	Vector3i voxelPosition = hashBlockPosition + Vector3i(x,y,z);

	if (!(*functor)(primaryVoxels[primaryHashEntry.ptr * VOXEL_BLOCK_SIZE3 + locId],
	                secondaryVoxels[secondaryHashTable[secondaryHash].ptr * VOXEL_BLOCK_SIZE3 + locId],
	                voxelPosition)) {
		*falseEncountered = true;
	}
}

// endregion ===========================================================================================================

}// end anonymous namespace (CUDA kernels)