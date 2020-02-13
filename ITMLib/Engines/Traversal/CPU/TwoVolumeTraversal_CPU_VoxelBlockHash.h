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

template<typename TVoxel1, typename TVoxel2>
class TwoVolumeTraversalEngine<TVoxel1, TVoxel2, VoxelBlockHash, VoxelBlockHash, MEMORYDEVICE_CPU> {
	/**
	 * \brief Concurrent traversal of two volumes with potentially-differing voxel types
	 * \details The two volumes must have matching hash table size
	 */
public:
// region ================================ STATIC TWO-VOLUME TRAVERSAL =================================================
	template<typename TStaticFunctor>
	inline static void StaticTraverseAll(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2) {

// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		HashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		int hash_entry_count = volume1->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
			const HashEntry& hash_entry1 = hash_table1[hash_code];
			if (hash_entry1.ptr < 0) continue;
			HashEntry hash_entry2 = hash_table2[hash_code];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				int hash_code2;
				if (!FindHashAtPosition(hash_code2, hash_entry1.pos, hash_table2)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << hash_entry1.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry2 = hash_table2[hash_code2];
				}
			}
			TVoxel1* localPrimaryVoxelBlock = &(voxels1[hash_entry1.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel2* localSecondaryVoxelBlock = &(voxels2[hash_entry2.ptr *
			                                                              (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel1& voxel1 = localPrimaryVoxelBlock[locId];
						TVoxel2& voxel2 = localSecondaryVoxelBlock[locId];
						TStaticFunctor::run(voxel1, voxel2);
					}
				}
			}
		}
	}

	// endregion
	// region ================================ STATIC TWO-VOLUME TRAVERSAL WITH VOXEL POSITION =========================
	template<typename TStaticFunctor>
	inline static void StaticTraverseAllWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2) {

// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		HashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		int totalHashEntryCount = volume1->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash_code = 0; hash_code < totalHashEntryCount; hash_code++) {
			const HashEntry& hash_entry1 = hash_table1[hash_code];
			if (hash_entry1.ptr < 0) continue;
			HashEntry hash_entry2 = hash_table2[hash_code];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				int hash_code2;
				if (!FindHashAtPosition(hash_code2, hash_entry1.pos, hash_table2)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << hash_entry1.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry2 = hash_table2[hash_code2];
				}
			}
			// position of the current entry in 3D space in voxel units
			Vector3i hash_block_positon = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE;
			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr *
			                                                              (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxel_position = hash_block_positon + Vector3i(x, y, z);
						TVoxel1& voxel1 = voxel_block1[locId];
						TVoxel2& voxel2 = voxel_block2[locId];
						TStaticFunctor::run(voxel1, voxel2, voxel_position);
					}
				}
			}
		}
	}

// endregion
// region ================================ DYNAMIC TWO-VOLUME TRAVERSAL =================================================
	template<typename TFunctor>
	inline static void
	TraverseAll(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor) {

// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		HashEntry* hash_table2 = volume2->index.GetEntries();


		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		int hash_entry_count = volume1->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {

			const HashEntry& hash_entry1 = hash_table1[hash_code];
			if (hash_entry1.ptr < 0) continue;
			HashEntry hash_entry2 = hash_table2[hash_code];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				int hash_code2;
				if (!FindHashAtPosition(hash_code2, hash_entry1.pos, hash_table2)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << hash_entry1.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry2 = hash_table2[hash_code2];
				}
			}
			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr *
			                                                              (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel1& voxel1 = voxel_block1[locId];
						TVoxel2& voxel2 = voxel_block2[locId];
						functor(voxel1, voxel2);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAll(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor, bool verbose) {

// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		HashEntry* hash_table2 = volume2->index.GetEntries();


		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		int hash_entry_count = volume1->index.hashEntryCount;

		bool mismatch_found = false;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
			if (mismatch_found) continue;
			const HashEntry& hash_entry1 = hash_table1[hash_code];
			HashEntry hash_entry2 = hash_table2[hash_code];

			auto block2HasMatchingBlock1 = [&](int hash_code2) {
				int alternative_hash_code1;
				if (!FindHashAtPosition(alternative_hash_code1, hash_entry2.pos, hash_table1)) {
					// could not find primary block corresponding to the secondary hash
					TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr *
					                                  (VOXEL_BLOCK_SIZE3)]);
					// if the secondary block is unaltered anyway, so no need to match and we're good, so return "true"
					if(verbose){
						return !isVoxelBlockAltered(voxel_block2, true, "Second-hash voxel unmatched in first hash: ", hash_entry2.pos, hash_code2);
					}else{
						return !isVoxelBlockAltered(voxel_block2);
					}
				} else {
					// alternative primary hash found, skip this primary hash since the corresponding secondary
					// block will be (or has been) processed with the alternative primary hash.
					return true;
				}
			};

			if (hash_entry1.ptr < 0) {
				if (hash_entry2.ptr < 0) {
					continue;
				} else {
					if (!block2HasMatchingBlock1(hash_code)) {
						mismatch_found = true;
						continue;
					} else {
						continue;
					}
				}
			}

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel block with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				if (hash_entry2.ptr >= 0) {
					if (!block2HasMatchingBlock1(hash_code)) {
						mismatch_found = true;
						continue;
					}
				}

				int hash_code2;
				if (!FindHashAtPosition(hash_code2, hash_entry1.pos, hash_table2)) {
					// If we cannot find this block, we check whether the primary voxel block has been altered, and
					// return "false" if it is -- i.e. the secondary voxel volume does not have a match.
					// If the primary voxel block has not been altered, we assume the allocation mismatch is benign and
					// continue to the next hash block.
					TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr *
					                                  (VOXEL_BLOCK_SIZE3)]);
					if (isVoxelBlockAltered(voxel_block1)) {
						mismatch_found = true;
						continue;
					} else {
						continue;
					}
				} else {
					hash_entry2 = hash_table2[hash_code2];
				}
			}
			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr *
			                                  (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel1& voxel1 = voxel_block1[locId];
						TVoxel2& voxel2 = voxel_block2[locId];
						if (!functor(voxel1, voxel2)) {
							mismatch_found = true;
						}
					}
				}
			}
		}
		return !mismatch_found;
	}

// ========================== DYNAMIC TWO-VOLUME TRAVERSAL WITH VOXEL POSITION =========================================
	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor) {

// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		HashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		int hash_entry_count = volume1->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
			const HashEntry& hash_entry1 = hash_table1[hash_code];
			if (hash_entry1.ptr < 0) continue;
			HashEntry hash_entry2 = hash_table2[hash_code];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				int hash_code2;

				if (!FindHashAtPosition(hash_code2, hash_entry1.pos, hash_table2)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << hash_entry1.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry2 = hash_table2[hash_code2];
				}
			}
			// position of the current entry in 3D space in voxel units
			Vector3i hash_block_positon = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE;

			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr * (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int linearIndexInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxel_position = hash_block_positon + Vector3i(x, y, z);
						TVoxel1& voxel1 = voxel_block1[linearIndexInBlock];
						TVoxel2& voxel2 = voxel_block2[linearIndexInBlock];
						functor(voxel1, voxel2, voxel_position);
					}
				}
			}
		}
	}

	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAllWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor, bool verbose) {

// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		HashEntry* hash_table2 = volume2->index.GetEntries();


		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		int hash_entry_count = volume1->index.hashEntryCount;

		bool mismatch_found = false;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hashCode = 0; hashCode < hash_entry_count; hashCode++) {
			if (mismatch_found) continue;
			const HashEntry& hash_entry1 = hash_table1[hashCode];
			HashEntry hash_entry2 = hash_table2[hashCode];

			auto block2HasMatchingBlock1 = [&](int hash_code2Code) {
				int alternativePrimaryHash;
				if (!FindHashAtPosition(alternativePrimaryHash, hash_entry2.pos, hash_table1)) {
					// could not find primary block corresponding to the secondary hash
					TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr *
					                                  (VOXEL_BLOCK_SIZE3)]);
					// if the secondary block is unaltered anyway, so no need to match and we're good, so return "true"
					if(verbose){
						return !isVoxelBlockAltered(voxel_block2, true, "Second-hash voxel unmatched in first hash: ", hash_entry2.pos, hash_code2Code);
					}else{
						return !isVoxelBlockAltered(voxel_block2);
					}
				} else {
					// alternative primary hash found, skip this primary hash since the corresponding secondary
					// block will be (or has been) processed with the alternative primary hash.
					return true;
				}
			};

			if (hash_entry1.ptr < 0) {
				if (hash_entry2.ptr < 0) {
					continue;
				} else {
					if (!block2HasMatchingBlock1(hashCode)) {
						mismatch_found = true;
						continue;
					} else {
						continue;
					}
				}
			}

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel block with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				if (hash_entry2.ptr >= 0) {
					if (!block2HasMatchingBlock1(hashCode)) {
						mismatch_found = true;
						continue;
					}
				}

				int hash_code2;
				if (!FindHashAtPosition(hash_code2, hash_entry1.pos, hash_table2)) {
					// If we cannot find this block, we check whether the primary voxel block has been altered, and
					// return "false" if it is -- i.e. the secondary voxel volume does not have a match.
					// If the primary voxel block has not been altered, we assume the allocation mismatch is benign and
					// continue to the next hash block.
					TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr *
					                                 (VOXEL_BLOCK_SIZE3)]);
					if (isVoxelBlockAltered(voxel_block1)) {
						mismatch_found = true;
						continue;
					} else {
						continue;
					}
				} else {
					hash_entry2 = hash_table2[hash_code2];
				}
			}
			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr *
			                                 (VOXEL_BLOCK_SIZE3)]);
			// position of the current entry in 3D space in voxel units
			Vector3i hash_block_positon = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE;
			for (int z = 0; z < VOXEL_BLOCK_SIZE && !mismatch_found; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxel_position = hash_block_positon + Vector3i(x, y, z);
						TVoxel1& voxel1 = voxel_block1[locId];
						TVoxel2& voxel2 = voxel_block2[locId];
						if (!functor(voxel1, voxel2, voxel_position)) {
							mismatch_found = true;
						}
					}
				}
			}
		}
		return !mismatch_found;
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithinBoundsWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor, Vector6i bounds) {

// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		HashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		int totalHashEntryCount = volume1->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < totalHashEntryCount; hash++) {
			const HashEntry& primaryHashEntry = hash_table1[hash];
			if (primaryHashEntry.ptr < 0) continue;
			HashEntry hash_code2Entry = hash_table2[hash];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (hash_code2Entry.pos != primaryHashEntry.pos) {
				int hash_code2;

				if (!FindHashAtPosition(hash_code2, primaryHashEntry.pos, hash_table2)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << primaryHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_code2Entry = hash_table2[hash_code2];
				}
			}
			Vector3i hashEntryMinPoint = primaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hashEntryMaxPoint = hashEntryMinPoint + Vector3i(VOXEL_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds)) {
				continue;
			}

			// position of the current entry in 3D space in voxel units
			Vector3i hash_block_positon = primaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

			TVoxel1* voxel_block1 = &(voxels1[primaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel2* voxel_block2 = &(voxels2[hash_code2Entry.ptr * (VOXEL_BLOCK_SIZE3)]);

			Vector6i localBounds = computeLocalBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds);

			for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
				for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
					for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxel_position = hash_block_positon + Vector3i(x, y, z);
						TVoxel1& voxel1 = voxel_block1[locId];
						TVoxel2& voxel2 = voxel_block2[locId];
						functor(voxel1, voxel2, voxel_position);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition_ST(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor) {

// *** traversal vars
		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		HashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		int hash_entry_count = volume1->index.hashEntryCount;

		for (int hash = 0; hash < hash_entry_count; hash++) {
			const HashEntry& hash_entry1 = hash_table1[hash];
			if (hash_entry1.ptr < 0) continue;
			HashEntry hash_entry2 = hash_table2[hash];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				int hash_code2;
				if (!FindHashAtPosition(hash_code2, hash_entry1.pos, hash_table2)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << hash_entry1.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry2 = hash_table2[hash_code2];
				}
			}
			// position of the current entry in 3D space in voxel units
			Vector3i hash_block_positon = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE;

			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr *
			                                                              (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxel_position = hash_block_positon + Vector3i(x, y, z);
						TVoxel1& voxel1 = voxel_block1[locId];
						TVoxel2& voxel2 = voxel_block2[locId];
						functor(voxel1, voxel2, voxel_position);
					}
				}
			}
		}
	}
// endregion ===========================================================================================================
};

} // namespace ITMLib

