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
#include "../../../Utils/Analytics/IsAltered.h"
#include "../../../Utils/Enums/ExecutionMode.h"

//TODO: work on reducing DRY violations

namespace ITMLib {

//======================================================================================================================
//                            CONTAINS TRAVERSAL METHODS FOR VOLUMES USING VoxelBlockHash FOR INDEXING
//                                  (TWO VOLUMES WITH POTENTIALLY DIFFERING VOXEL TYPES)
//======================================================================================================================

/**
 * \brief Concurrent traversal of two volumes with potentially-differing voxel types
 * \details The two volumes must have matching hash table size
 */
template<typename TVoxel1, typename TVoxel2>
class TwoVolumeTraversalEngine<TVoxel1, TVoxel2, VoxelBlockHash, VoxelBlockHash, MEMORYDEVICE_CPU> {
private:


	template<typename TBlockTraversalFunction>
	inline static void TraverseUtilized_Generic(VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
	                                            VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
	                                            TBlockTraversalFunction&& block_traverser) {
		TVoxel1* voxels1 = volume1->GetVoxels();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		TVoxel2* voxels2 = volume2->GetVoxels();
		HashEntry* hash_table2 = volume2->index.GetEntries();
		int utilized_entry_count = volume1->index.GetUtilizedBlockCount();
		const int* utilized_hash_codes = volume1->index.GetUtilizedBlockHashCodes();

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) firstprivate(utilized_entry_count) shared(hash_table1, hash_table2, \
utilized_hash_codes, voxels1, voxels2, block_traverser)
#endif
		for (int hash_code_index = 0; hash_code_index < utilized_entry_count; hash_code_index++) {
			int hash_code = utilized_hash_codes[hash_code_index];
			const HashEntry& hash_entry1 = hash_table1[hash_code];
			if (hash_entry1.ptr < 0) continue;
			HashEntry hash_entry2 = hash_table2[hash_code];

			// the rare case where we have different positions for exists_in_hash_table1 & secondary voxel block with the same index:
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
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr * (VOXEL_BLOCK_SIZE3)]);
			std::forward<TBlockTraversalFunction>(block_traverser)(voxel_block1, voxel_block2, hash_entry1);
		}
	}

	template<typename TBlockTraversalFunction>
	inline static void TraverseAll_Generic(VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
	                                       VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
	                                       TBlockTraversalFunction&& block_traverser) {
		TVoxel1* voxels1 = volume1->GetVoxels();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		TVoxel2* voxels2 = volume2->GetVoxels();
		HashEntry* hash_table2 = volume2->index.GetEntries();
		const int hash_entry_count = volume1->index.hash_entry_count;

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) firstprivate(hash_entry_count) shared(hash_table1, hash_table2, \
voxels1, voxels2, block_traverser)
#endif
		for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
			const HashEntry& hash_entry1 = hash_table1[hash_code];
			if (hash_entry1.ptr < 0) continue;
			HashEntry hash_entry2 = hash_table2[hash_code];

			// the rare case where we have different positions for exists_in_hash_table1 & secondary voxel block with the same index:
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
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr * (VOXEL_BLOCK_SIZE3)]);
			std::forward<TBlockTraversalFunction>(block_traverser)(voxel_block1, voxel_block2, hash_entry1);
		}
	}

	template<typename T2VoxelFunction>
	inline static void
	Traverse2Blocks(TVoxel1* voxel_block1, TVoxel2* voxel_block2, T2VoxelFunction&& process_function) {
		for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
			for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
				for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
					int linear_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
					TVoxel1& voxel1 = voxel_block1[linear_index_in_block];
					TVoxel2& voxel2 = voxel_block2[linear_index_in_block];
					std::forward<T2VoxelFunction>(process_function)(voxel1, voxel2);
				}
			}
		}
	}

	template<typename T2VoxelAndPositionFunction>
	inline static void
	Traverse2BlocksWithPosition(TVoxel1* voxel_block1, TVoxel2* voxel_block2, const HashEntry& hash_entry1,
	                            T2VoxelAndPositionFunction&& process_function) {
		Vector3i hash_block_position = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE;
		for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
			for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
				for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
					int linear_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
					TVoxel1& voxel1 = voxel_block1[linear_index_in_block];
					TVoxel2& voxel2 = voxel_block2[linear_index_in_block];
					Vector3i voxel_position = hash_block_position + Vector3i(x, y, z);
					std::forward<T2VoxelAndPositionFunction>(process_function)(voxel1, voxel2, voxel_position);
				}
			}
		}
	}

	template<typename TProcessMatchedBlocksFunction, typename TCheckIfUnmatchedBlock1IsSignificantFunction, typename TCheckIfUnmatchedBlock2IsSignificantFunction>
	inline static bool
	TraverseAndCompareAll_Generic(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TProcessMatchedBlocksFunction&& traverse_matched_block_function,
			TCheckIfUnmatchedBlock1IsSignificantFunction&& check_if_unmatched_block1_is_significant,
			TCheckIfUnmatchedBlock2IsSignificantFunction&& check_if_unmatched_block2_is_significant,
			bool verbose) {

// *** traversal vars
		TVoxel2* voxels2 = volume2->GetVoxels();
		HashEntry* hash_table2 = volume2->index.GetEntries();


		TVoxel1* voxels1 = volume1->GetVoxels();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		int hash_entry_count = volume1->index.hash_entry_count;

		bool mismatch_found = false;

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) firstprivate(hash_entry_count, verbose) shared(hash_table1, hash_table2, \
voxels1, voxels2, check_if_unmatched_block2_is_significant, check_if_unmatched_block1_is_significant, traverse_matched_block_function, \
mismatch_found)
#endif
		for (int hash_code1 = 0; hash_code1 < hash_entry_count; hash_code1++) {
			if (mismatch_found) continue;
			const HashEntry& hash_entry1 = hash_table1[hash_code1];
			HashEntry hash_entry2 = hash_table2[hash_code1];

			auto block_2_has_matching_block_1 = [&](int hash_code2) {
				int alternative_hash_code1;
				if (!FindHashAtPosition(alternative_hash_code1, hash_entry2.pos, hash_table1)) {
					// could not find exists_in_hash_table1 block corresponding to the secondary hash
					TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr *
					                                  (VOXEL_BLOCK_SIZE3)]);
					// if the secondary block is unaltered anyway, so no need to match and we're good, so return "true"
					if (std::forward<TCheckIfUnmatchedBlock2IsSignificantFunction>(check_if_unmatched_block2_is_significant)
							(voxel_block2, verbose, "Second-hash voxel block unmatched in first hash: ",
							 hash_entry2.pos, hash_code2)) {
						return false;
					}
				}
				// alternative exists_in_hash_table1 hash found, skip this exists_in_hash_table1 hash since the corresponding secondary
				// block will be (or has been) processed with the alternative exists_in_hash_table1 hash.
				return true;
			};

			if (hash_entry1.ptr < 0) {
				if (hash_entry2.ptr < 0) {
					continue;
				} else {
					if (!block_2_has_matching_block_1(hash_code1)) {
						mismatch_found = true;
						continue;
					} else {
						continue;
					}
				}
			}

			// the rare case where we have different positions for exists_in_hash_table1 & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel block with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				if (hash_entry2.ptr >= 0) {
					if (!block_2_has_matching_block_1(hash_code1)) {
						mismatch_found = true;
						continue;
					}
				}

				int hash_code2;
				if (!FindHashAtPosition(hash_code2, hash_entry1.pos, hash_table2)) {
					// If we cannot find this block, we check whether the exists_in_hash_table1 voxel block has been altered, and
					// return "false" if it is -- i.e. the secondary voxel volume does not have a match.
					// If the exists_in_hash_table1 voxel block has not been altered, we assume the allocation mismatch is benign and
					// continue to the next hash block.
					TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr *
					                                  (VOXEL_BLOCK_SIZE3)]);


					if (std::forward<TCheckIfUnmatchedBlock1IsSignificantFunction>(check_if_unmatched_block1_is_significant)
							(voxel_block1, verbose, "First-hash voxel block unmatched in second hash table: ",
							 hash_entry1.pos, hash_code1)) {
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

			std::forward<TProcessMatchedBlocksFunction>(traverse_matched_block_function)
					(voxel_block1, voxel_block2, hash_entry1, mismatch_found);
		}
		return !mismatch_found;
	}


public:
// region ================================ STATIC TWO-VOLUME TRAVERSAL =================================================
	template<typename TStaticFunctor>
	inline static void TraverseAll(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2) {
		TraverseAll_Generic(
				volume1, volume2,
				[](TVoxel1* voxel_block1, TVoxel2* voxel_block2, HashEntry& hash_entry1) {
					Traverse2Blocks(
							voxel_block1,
							voxel_block2,
							[](TVoxel1& voxel1, TVoxel2& voxel2) {
								TStaticFunctor::run(voxel1, voxel2);
							}
					);

				}
		);
	}

	// endregion
	// region ================================ STATIC TWO-VOLUME TRAVERSAL WITH VOXEL POSITION =========================
	template<typename TStaticFunctor>
	inline static void TraverseAllWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2) {
		TraverseAll_Generic(
				volume1, volume2,
				[](TVoxel1* voxel_block1, TVoxel2* voxel_block2, HashEntry& hash_entry1) {
					Traverse2BlocksWithPosition(
							voxel_block1,
							voxel_block2,
							hash_entry1,
							[](TVoxel1& voxel1, TVoxel2& voxel2, const Vector3i& voxel_position) {
								TStaticFunctor::run(voxel1, voxel2, voxel_position);
							}
					);

				}
		);
	}

// endregion
// region ================================ DYNAMIC TWO-VOLUME TRAVERSAL =================================================
	template<typename TFunctor>
	inline static void
	TraverseAll(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor) {
		TraverseAll_Generic(
				volume1, volume2,
				[&functor](TVoxel1* voxel_block1, TVoxel2* voxel_block2, const HashEntry& hash_entry1) {
					Traverse2Blocks(
							voxel_block1,
							voxel_block2,
							[&functor](TVoxel1& voxel1, TVoxel2& voxel2) {
								functor(voxel1, voxel2);
							}
					);

				}
		);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor) {
		TraverseUtilized_Generic(
				volume1, volume2,
				[&functor](TVoxel1* voxel_block1, TVoxel2* voxel_block2, const HashEntry& hash_entry1) {
					Traverse2Blocks(
							voxel_block1,
							voxel_block2,
							[&functor](TVoxel1& voxel1, TVoxel2& voxel2) {
								functor(voxel1, voxel2);
							}
					);

				}
		);
	}


// ========================== DYNAMIC TWO-VOLUME TRAVERSAL WITH VOXEL POSITION =========================================
	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor) {
		TraverseAll_Generic(
				volume1, volume2,
				[&functor](TVoxel1* voxel_block1, TVoxel2* voxel_block2, const HashEntry& hash_entry1) {
					Traverse2BlocksWithPosition(
							voxel_block1, voxel_block2, hash_entry1,
							[&functor](TVoxel1& voxel1, TVoxel2& voxel2, const Vector3i& voxel_position) {
								functor(voxel1, voxel2, voxel_position);
							}
					);

				}
		);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor) {
		TraverseUtilized_Generic(
				volume1, volume2,
				[&functor](TVoxel1* voxel_block1, TVoxel2* voxel_block2, const HashEntry& hash_entry1) {
					Traverse2BlocksWithPosition(
							voxel_block1, voxel_block2, hash_entry1,
							[&functor](TVoxel1& voxel1, TVoxel2& voxel2, const Vector3i& voxel_position) {
								functor(voxel1, voxel2, voxel_position);
							}
					);

				}
		);
	}

// ========================== DYNAMIC TWO-VOLUME TRAVERSAL & COMPARISON ================================================

	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAll(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor, bool verbose) {
		return TraverseAndCompareAll_Generic(
				volume1, volume2,
				[&functor](TVoxel1* voxel_block1, TVoxel2* voxel_block2, const HashEntry& hash_entry1,
				           bool& mismatch_found) {
					Traverse2Blocks(
							voxel_block1, voxel_block2,
							[&functor, &mismatch_found](TVoxel1& voxel1, TVoxel2& voxel2) {
								if (!functor(voxel1, voxel2)) {
									mismatch_found = true;
								}
							}
					);
				},
				isVoxelBlockAltered<TVoxel1>,
				isVoxelBlockAltered<TVoxel2>,
				verbose
		);
	}


	template<ExecutionMode TExecutionMode = ExecutionMode::OPTIMIZED, typename TFunctor>
	inline static bool
	TraverseAndCompareAllWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor, bool verbose) {
		return TraverseAndCompareAll_Generic(
				volume1, volume2,
				[&functor](TVoxel1* voxel_block1, TVoxel2* voxel_block2, const HashEntry& hash_entry1,
				           bool& mismatch_found) {
					Traverse2BlocksWithPosition(
							voxel_block1, voxel_block2, hash_entry1,
							[&functor, &mismatch_found](TVoxel1& voxel1, TVoxel2& voxel2,
							                            const Vector3i& voxel_position) {
								if (!functor(voxel1, voxel2, voxel_position)) {
									mismatch_found = true;
								}
							}
					);
				},
				isVoxelBlockAltered<TVoxel1>,
				isVoxelBlockAltered<TVoxel2>,
				verbose
		);
	}

	template<typename TFunctor>
	inline static bool
	TraverseAndCompareAllWithPosition_Volume1SupersetVolume2(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor, bool verbose) {
		return TraverseAndCompareAll_Generic(
				volume1, volume2,
				[&functor](TVoxel1* voxel_block1, TVoxel2* voxel_block2, const HashEntry& hash_entry1,
				           bool& mismatch_found) {
					Traverse2BlocksWithPosition(
							voxel_block1, voxel_block2, hash_entry1,
							[&functor, &mismatch_found](TVoxel1& voxel1, TVoxel2& voxel2,
							                            const Vector3i& voxel_position) {
								if (!functor(voxel1, voxel2, voxel_position)) {
									mismatch_found = true;
								}
							}
					);
				},
				false,
				isVoxelBlockAltered<TVoxel2>,
				verbose
		);
	}

	template<typename TFunctor>
	inline static bool
	TraverseAndCompareMatchingFlags(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelFlags flags,
			TFunctor& functor, bool verbose) {
		return TraverseAndCompareAll_Generic(
				volume1, volume2,
				[&functor, &flags](TVoxel1* voxel_block1, TVoxel2* voxel_block2, const HashEntry& hash_entry1,
				                   bool& mismatch_found) {
					Traverse2Blocks(
							voxel_block1, voxel_block2,
							[&functor, &mismatch_found, &flags](TVoxel1& voxel1, TVoxel2& voxel2) {
								if ((voxel1.flags == flags || voxel2.flags == flags) &&
								    (!functor(voxel1, voxel2) || voxel2.flags != voxel2.flags)) {
									mismatch_found = true;
								}
							}
					);
				},
				[&flags](TVoxel1* voxel_block, bool verbose = false,
				         std::string message = "",
				         Vector3s block_spatial_position = Vector3s((short) 0),
				         int hash_code = 0) {
					return isVoxelBlockAlteredPredicate(voxel_block,
					                                    [&flags](TVoxel1& voxel) { return voxel.flags == flags; }, verbose,
					                                    message, block_spatial_position, hash_code);
				},
				[&flags](TVoxel2* voxel_block, bool verbose = false,
				         std::string message = "",
				         Vector3s block_spatial_position = Vector3s((short) 0),
				         int hash_code = 0) {
					return isVoxelBlockAlteredPredicate(voxel_block,
					                                    [&flags](TVoxel2& voxel) { return voxel.flags == flags; }, verbose,
					                                    message, block_spatial_position, hash_code);
				},
				verbose
		);
	}

	template<ExecutionMode TExecutionMode = OPTIMIZED, typename TFunctor>
	inline static bool
	TraverseAndCompareMatchingFlagsWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelFlags flags,
			TFunctor& functor, bool verbose) {
		return TraverseAndCompareAll_Generic(
				volume1, volume2,
				[&functor, &flags](TVoxel1* voxel_block1, TVoxel2* voxel_block2, const HashEntry& hash_entry1,
				                   bool& mismatch_found) {
					Traverse2BlocksWithPosition(
							voxel_block1, voxel_block2, hash_entry1,
							[&functor, &mismatch_found, &flags](TVoxel1& voxel1, TVoxel2& voxel2,
							                                    const Vector3i& voxel_position) {
								if ((voxel1.flags == flags || voxel2.flags == flags) &&
								    (!functor(voxel1, voxel2, voxel_position) || voxel2.flags != voxel2.flags)) {
									mismatch_found = true;
								}
							}
					);
				},
				[&flags](TVoxel1* voxel_block, bool verbose = false,
				         std::string message = "",
				         Vector3s block_spatial_position = Vector3s((short) 0),
				         int hash_code = 0) {
					return isVoxelBlockAlteredPredicate(voxel_block,
					                                    [&flags](TVoxel1& voxel) { return voxel.flags == flags; }, verbose,
					                                    message, block_spatial_position, hash_code);
				},
				[&flags](TVoxel2* voxel_block, bool verbose = false,
				         std::string message = "",
				         Vector3s block_spatial_position = Vector3s((short) 0),
				         int hash_code = 0) {
					return isVoxelBlockAlteredPredicate(voxel_block,
					                                    [&flags](TVoxel2& voxel) { return voxel.flags == flags; }, verbose,
					                                    message, block_spatial_position, hash_code);
				},
				verbose
		);
	}

	template<typename TFunctor>
	inline static bool
	TraverseAndCompareMatchingFlagsWithPosition_Volume1SupersetVolume2(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelFlags flags,
			TFunctor& functor, bool verbose) {
		return TraverseAndCompareAll_Generic(
				volume1, volume2,
				[&functor, &flags](TVoxel1* voxel_block1, TVoxel2* voxel_block2, const HashEntry& hash_entry1,
				                   bool& mismatch_found) {
					Traverse2BlocksWithPosition(
							voxel_block1, voxel_block2, hash_entry1,
							[&functor, &mismatch_found, &flags](TVoxel1& voxel1, TVoxel2& voxel2,
							                                    const Vector3i& voxel_position) {
								if ((voxel1.flags == flags || voxel2.flags == flags) &&
								    (!functor(voxel1, voxel2, voxel_position) || voxel2.flags != voxel2.flags)) {
									mismatch_found = true;
								}
							}
					);
				},
				[&flags](TVoxel1* voxel_block, bool verbose = false,
				         std::string message = "",
				         Vector3s block_spatial_position = Vector3s((short) 0),
				         int hash_code = 0) {
					// ignore any blocks present in volume 1 that are absent in volume 2
					return false;
				},
				[&flags](TVoxel2* voxel_block, bool verbose = false,
				         std::string message = "",
				         Vector3s block_spatial_position = Vector3s((short) 0),
				         int hash_code = 0) {
					return isVoxelBlockAlteredPredicate(voxel_block,
					                                    [&flags](TVoxel2& voxel) { return voxel.flags == flags; }, verbose,
					                                    message, block_spatial_position, hash_code);
				},
				verbose
		);
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithinBoundsWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor, Vector6i bounds) {

// *** traversal vars
		TVoxel2* voxels2 = volume2->GetVoxels();
		HashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel1* voxels1 = volume1->GetVoxels();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		int totalHashEntryCount = volume1->index.hash_entry_count;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < totalHashEntryCount; hash++) {
			const HashEntry& primaryHashEntry = hash_table1[hash];
			if (primaryHashEntry.ptr < 0) continue;
			HashEntry hash_code2Entry = hash_table2[hash];

			// the rare case where we have different positions for exists_in_hash_table1 & secondary voxel block with the same index:
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

			Vector6i localBounds = ComputeLocalBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds);

			for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
				for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
					for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
						int linear_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxel_position = hash_block_positon + Vector3i(x, y, z);
						TVoxel1& voxel1 = voxel_block1[linear_index_in_block];
						TVoxel2& voxel2 = voxel_block2[linear_index_in_block];
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
		TVoxel2* voxels2 = volume2->GetVoxels();
		HashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel1* voxels1 = volume1->GetVoxels();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		int hash_entry_count = volume1->index.hash_entry_count;

		for (int hash = 0; hash < hash_entry_count; hash++) {
			const HashEntry& hash_entry1 = hash_table1[hash];
			if (hash_entry1.ptr < 0) continue;
			HashEntry hash_entry2 = hash_table2[hash];

			// the rare case where we have different positions for exists_in_hash_table1 & secondary voxel block with the same index:
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
						int linear_index_in_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxel_position = hash_block_positon + Vector3i(x, y, z);
						TVoxel1& voxel1 = voxel_block1[linear_index_in_block];
						TVoxel2& voxel2 = voxel_block2[linear_index_in_block];
						functor(voxel1, voxel2, voxel_position);
					}
				}
			}
		}
	}
// endregion ===========================================================================================================
};

} // namespace ITMLib

