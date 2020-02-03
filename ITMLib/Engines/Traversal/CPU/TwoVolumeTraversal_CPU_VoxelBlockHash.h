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
#include "../Interface/TwoVolumeTraversal.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../Shared/VolumeTraversal_Shared.h"

//TODO: work on reducing DRY violations

namespace ITMLib{

//======================================================================================================================
//                            CONTAINS TRAVERSAL METHODS FOR VOLUMES USING PlainVoxelArray FOR INDEXING
//                                  (TWO VOLUMES WITH DIFFERING VOXEL TYPES)
//======================================================================================================================

template<typename TVoxelPrimary, typename TVoxelSecondary>
class TwoVolumeTraversalEngine<TVoxelPrimary, TVoxelSecondary, VoxelBlockHash, VoxelBlockHash, MEMORYDEVICE_CPU> {
	/**
	 * \brief Concurrent traversal of two volumes with potentially-differing voxel types
	 * \details The two volumes must have matching hash table size
	 */
public:
// region ================================ STATIC TWO-VOLUME TRAVERSAL =================================================
	template<typename TStaticFunctor>
	inline static void StaticDualVoxelTraversal(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		HashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		HashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {
			const HashEntry& currentLiveHashEntry = primaryHashTable[hash];
			if (currentLiveHashEntry.ptr < 0) continue;
			HashEntry currentCanonicalHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentLiveHashEntry.pos, secondaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentLiveHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentCanonicalHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			TVoxelPrimary* localPrimaryVoxelBlock = &(primaryVoxels[currentLiveHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxelSecondary* localSecondaryVoxelBlock = &(secondaryVoxels[currentCanonicalHashEntry.ptr *
			                                                              (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxelPrimary& primaryVoxel = localPrimaryVoxelBlock[locId];
						TVoxelSecondary& secondaryVoxel = localSecondaryVoxelBlock[locId];
						TStaticFunctor::run(primaryVoxel, secondaryVoxel);
					}
				}
			}
		}
	}

	// endregion
	// region ================================ STATIC TWO-VOLUME TRAVERSAL WITH VOXEL POSITION =========================
	template<typename TStaticFunctor>
	inline static void StaticDualVoxelPositionTraversal(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		HashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		HashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int totalHashEntryCount = primaryScene->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < totalHashEntryCount; hash++) {
			const HashEntry& currentPrimaryHashEntry = primaryHashTable[hash];
			if (currentPrimaryHashEntry.ptr < 0) continue;
			HashEntry currentSecondaryHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentSecondaryHashEntry.pos != currentPrimaryHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentPrimaryHashEntry.pos, secondaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentPrimaryHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentSecondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			// position of the current entry in 3D space in voxel units
			Vector3i hashBlockPosition = currentPrimaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			TVoxelPrimary* localLiveVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxelSecondary* localCanonicalVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr *
			                                                              (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
						TVoxelSecondary& secondaryVoxel = localCanonicalVoxelBlock[locId];
						TStaticFunctor::run(primaryVoxel, secondaryVoxel, voxelPosition);
					}
				}
			}
		}
	}

// endregion
// region ================================ DYNAMIC TWO-VOLUME TRAVERSAL =================================================
	template<typename TFunctor>
	inline static void
	DualVoxelTraversal(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TFunctor& functor) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		HashEntry* secondaryHashTable = secondaryScene->index.GetEntries();


		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		HashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {

			const HashEntry& currentLiveHashEntry = primaryHashTable[hash];
			if (currentLiveHashEntry.ptr < 0) continue;
			HashEntry currentCanonicalHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentLiveHashEntry.pos, secondaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentLiveHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentCanonicalHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			TVoxelPrimary* localLiveVoxelBlock = &(primaryVoxels[currentLiveHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxelSecondary* localCanonicalVoxelBlock = &(secondaryVoxels[currentCanonicalHashEntry.ptr *
			                                                              (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
						TVoxelSecondary& secondaryVoxel = localCanonicalVoxelBlock[locId];
						functor(primaryVoxel, secondaryVoxel);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TFunctor& functor, bool verbose) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		HashEntry* secondaryHashTable = secondaryScene->index.GetEntries();


		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		HashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int hashEntryCount = primaryScene->index.hashEntryCount;

		bool mismatchFound = false;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
			if (mismatchFound) continue;
			const HashEntry& primaryHashEntry = primaryHashTable[hashCode];
			HashEntry secondaryHashEntry = secondaryHashTable[hashCode];

			auto secondaryHashEntryHasMatchingPrimary = [&](int secondaryHashCode) {
				int alternativePrimaryHash;
				if (!FindHashAtPosition(alternativePrimaryHash, secondaryHashEntry.pos, primaryHashTable)) {
					// could not find primary block corresponding to the secondary hash
					TVoxelSecondary* secondaryVoxelBlock = &(secondaryVoxels[secondaryHashEntry.ptr *
					                                                         (VOXEL_BLOCK_SIZE3)]);
					// if the secondary block is unaltered anyway, so no need to match and we're good, so return "true"
					if(verbose){
						return !isVoxelBlockAltered(secondaryVoxelBlock, true, "Second-hash voxel unmatched in first hash: ", secondaryHashEntry.pos, secondaryHashCode);
					}else{
						return !isVoxelBlockAltered(secondaryVoxelBlock);
					}
				} else {
					// alternative primary hash found, skip this primary hash since the corresponding secondary
					// block will be (or has been) processed with the alternative primary hash.
					return true;
				}
			};

			if (primaryHashEntry.ptr < 0) {
				if (secondaryHashEntry.ptr < 0) {
					continue;
				} else {
					if (!secondaryHashEntryHasMatchingPrimary(hashCode)) {
						mismatchFound = true;
						continue;
					} else {
						continue;
					}
				}
			}

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel block with the matching coordinates
			if (secondaryHashEntry.pos != primaryHashEntry.pos) {
				if (secondaryHashEntry.ptr >= 0) {
					if (!secondaryHashEntryHasMatchingPrimary(hashCode)) {
						mismatchFound = true;
						continue;
					}
				}

				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, primaryHashEntry.pos, secondaryHashTable)) {
					// If we cannot find this block, we check whether the primary voxel block has been altered, and
					// return "false" if it is -- i.e. the secondary voxel volume does not have a match.
					// If the primary voxel block has not been altered, we assume the allocation mismatch is benign and
					// continue to the next hash block.
					TVoxelPrimary* primaryVoxelBlock = &(primaryVoxels[primaryHashEntry.ptr *
					                                                   (VOXEL_BLOCK_SIZE3)]);
					if (isVoxelBlockAltered(primaryVoxelBlock)) {
						mismatchFound = true;
						continue;
					} else {
						continue;
					}
				} else {
					secondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			TVoxelPrimary* primaryVoxelBlock = &(primaryVoxels[primaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxelSecondary* secondaryVoxelBlock = &(secondaryVoxels[secondaryHashEntry.ptr *
			                                                         (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxelPrimary& primaryVoxel = primaryVoxelBlock[locId];
						TVoxelSecondary& secondaryVoxel = secondaryVoxelBlock[locId];
						if (!functor(primaryVoxel, secondaryVoxel)) {
							mismatchFound = true;
						}
					}
				}
			}
		}
		return !mismatchFound;
	}

// ========================== DYNAMIC TWO-VOLUME TRAVERSAL WITH VOXEL POSITION =========================================
	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TFunctor& functor) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		HashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		HashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {
			const HashEntry& primaryHashEntry = primaryHashTable[hash];
			if (primaryHashEntry.ptr < 0) continue;
			HashEntry secondaryHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (secondaryHashEntry.pos != primaryHashEntry.pos) {
				int secondaryHash;

				if (!FindHashAtPosition(secondaryHash, primaryHashEntry.pos, secondaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << primaryHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					secondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			// position of the current entry in 3D space in voxel units
			Vector3i hashBlockPosition = primaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

			TVoxelPrimary* primaryVoxelBlock = &(primaryVoxels[primaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxelSecondary* secondaryVoxelBlock = &(secondaryVoxels[secondaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int linearIndexInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						TVoxelPrimary& primaryVoxel = primaryVoxelBlock[linearIndexInBlock];
						TVoxelSecondary& secondaryVoxel = secondaryVoxelBlock[linearIndexInBlock];
						functor(primaryVoxel, secondaryVoxel, voxelPosition);
					}
				}
			}
		}
	}

	template<typename TFunctor>
	inline static bool
	DualVoxelPositionTraversal_AllTrue(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TFunctor& functor, bool verbose) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		HashEntry* secondaryHashTable = secondaryScene->index.GetEntries();


		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		HashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int hashEntryCount = primaryScene->index.hashEntryCount;

		bool mismatchFound = false;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
			if (mismatchFound) continue;
			const HashEntry& primaryHashEntry = primaryHashTable[hashCode];
			HashEntry secondaryHashEntry = secondaryHashTable[hashCode];

			auto secondaryHashEntryHasMatchingPrimary = [&](int secondaryHashCode) {
				int alternativePrimaryHash;
				if (!FindHashAtPosition(alternativePrimaryHash, secondaryHashEntry.pos, primaryHashTable)) {
					// could not find primary block corresponding to the secondary hash
					TVoxelSecondary* secondaryVoxelBlock = &(secondaryVoxels[secondaryHashEntry.ptr *
					                                                         (VOXEL_BLOCK_SIZE3)]);
					// if the secondary block is unaltered anyway, so no need to match and we're good, so return "true"
					if(verbose){
						return !isVoxelBlockAltered(secondaryVoxelBlock, true, "Second-hash voxel unmatched in first hash: ", secondaryHashEntry.pos, secondaryHashCode);
					}else{
						return !isVoxelBlockAltered(secondaryVoxelBlock);
					}
				} else {
					// alternative primary hash found, skip this primary hash since the corresponding secondary
					// block will be (or has been) processed with the alternative primary hash.
					return true;
				}
			};

			if (primaryHashEntry.ptr < 0) {
				if (secondaryHashEntry.ptr < 0) {
					continue;
				} else {
					if (!secondaryHashEntryHasMatchingPrimary(hashCode)) {
						mismatchFound = true;
						continue;
					} else {
						continue;
					}
				}
			}

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel block with the matching coordinates
			if (secondaryHashEntry.pos != primaryHashEntry.pos) {
				if (secondaryHashEntry.ptr >= 0) {
					if (!secondaryHashEntryHasMatchingPrimary(hashCode)) {
						mismatchFound = true;
						continue;
					}
				}

				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, primaryHashEntry.pos, secondaryHashTable)) {
					// If we cannot find this block, we check whether the primary voxel block has been altered, and
					// return "false" if it is -- i.e. the secondary voxel volume does not have a match.
					// If the primary voxel block has not been altered, we assume the allocation mismatch is benign and
					// continue to the next hash block.
					TVoxelPrimary* primaryVoxelBlock = &(primaryVoxels[primaryHashEntry.ptr *
					                                                   (VOXEL_BLOCK_SIZE3)]);
					if (isVoxelBlockAltered(primaryVoxelBlock)) {
						mismatchFound = true;
						continue;
					} else {
						continue;
					}
				} else {
					secondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			TVoxelPrimary* primaryVoxelBlock = &(primaryVoxels[primaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxelSecondary* secondaryVoxelBlock = &(secondaryVoxels[secondaryHashEntry.ptr *
			                                                         (VOXEL_BLOCK_SIZE3)]);
			// position of the current entry in 3D space in voxel units
			Vector3i hashBlockPosition = primaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			for (int z = 0; z < VOXEL_BLOCK_SIZE && !mismatchFound; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						TVoxelPrimary& primaryVoxel = primaryVoxelBlock[locId];
						TVoxelSecondary& secondaryVoxel = secondaryVoxelBlock[locId];
						if (!functor(primaryVoxel, secondaryVoxel, voxelPosition)) {
							mismatchFound = true;
						}
					}
				}
			}
		}
		return !mismatchFound;
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversalWithinBounds(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TFunctor& functor, Vector6i bounds) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		HashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		HashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int totalHashEntryCount = primaryScene->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < totalHashEntryCount; hash++) {
			const HashEntry& primaryHashEntry = primaryHashTable[hash];
			if (primaryHashEntry.ptr < 0) continue;
			HashEntry secondaryHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (secondaryHashEntry.pos != primaryHashEntry.pos) {
				int secondaryHash;

				if (!FindHashAtPosition(secondaryHash, primaryHashEntry.pos, secondaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << primaryHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					secondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			Vector3i hashEntryMinPoint = primaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hashEntryMaxPoint = hashEntryMinPoint + Vector3i(VOXEL_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds)) {
				continue;
			}

			// position of the current entry in 3D space in voxel units
			Vector3i hashBlockPosition = primaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

			TVoxelPrimary* primaryVoxelBlock = &(primaryVoxels[primaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxelSecondary* secondaryVoxelBlock = &(secondaryVoxels[secondaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);

			Vector6i localBounds = computeLocalBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds);

			for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
				for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
					for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						TVoxelPrimary& primaryVoxel = primaryVoxelBlock[locId];
						TVoxelSecondary& secondaryVoxel = secondaryVoxelBlock[locId];
						functor(primaryVoxel, secondaryVoxel, voxelPosition);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal_DefaultForMissingSecondary(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TFunctor& functor) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		HashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		HashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {
			const HashEntry& currentLiveHashEntry = primaryHashTable[hash];
			if (currentLiveHashEntry.ptr < 0) continue;
			HashEntry currentCanonicalHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentLiveHashEntry.pos, secondaryHashTable)) {
					TVoxelPrimary* localLiveVoxelBlock = &(primaryVoxels[currentLiveHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
					Vector3i hashBlockPosition = currentLiveHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
					TVoxelSecondary secondaryVoxel;
					for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
						for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
							for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
								int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
								Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
								TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
								functor(primaryVoxel, secondaryVoxel, voxelPosition);
							}
						}
					}
					continue;
				} else {
					currentCanonicalHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			// position of the current entry in 3D space in voxel units
			Vector3i hashBlockPosition = currentLiveHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

			TVoxelPrimary* localLiveVoxelBlock = &(primaryVoxels[currentLiveHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxelSecondary* localCanonicalVoxelBlock = &(secondaryVoxels[currentCanonicalHashEntry.ptr *
			                                                              (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
						TVoxelSecondary& secondaryVoxel = localCanonicalVoxelBlock[locId];
						functor(primaryVoxel, secondaryVoxel, voxelPosition);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal_SingleThreaded(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TFunctor& functor) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		HashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		HashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.hashEntryCount;

		for (int hash = 0; hash < noTotalEntries; hash++) {
			const HashEntry& currentLiveHashEntry = primaryHashTable[hash];
			if (currentLiveHashEntry.ptr < 0) continue;
			HashEntry currentCanonicalHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentLiveHashEntry.pos, secondaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentLiveHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentCanonicalHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			// position of the current entry in 3D space in voxel units
			Vector3i hashBlockPosition = currentLiveHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

			TVoxelPrimary* localLiveVoxelBlock = &(primaryVoxels[currentLiveHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxelSecondary* localCanonicalVoxelBlock = &(secondaryVoxels[currentCanonicalHashEntry.ptr *
			                                                              (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
						TVoxelSecondary& secondaryVoxel = localCanonicalVoxelBlock[locId];
						functor(primaryVoxel, secondaryVoxel, voxelPosition);
					}
				}
			}
		}
	}
// endregion ===========================================================================================================
};

} // namespace ITMLib
