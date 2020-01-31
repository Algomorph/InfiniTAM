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
#include "../Interface/ThreeVolumeTraversal.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"

namespace ITMLib {

//======================================================================================================================
//                            CONTAINS TRAVERSAL METHODS FOR VOLUMES USING VoxelBlockHash FOR INDEXING
//                                  (THREE VOLUMES WITH DIFFERING VOXEL TYPES)
//======================================================================================================================
template<typename TVoxel1, typename TVoxel2, typename TVoxel3>
class ThreeVolumeTraversalEngine<TVoxel1, TVoxel2, TVoxel3, VoxelBlockHash, MEMORYDEVICE_CPU> {
	/**
	 * \brief Concurrent traversal of three volumes with potentially-differing voxel types
	 * \details All volumes must have matching hash table size
	 */
public:
// region ================================ DYNAMIC THREE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void Traverse(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			VoxelVolume<TVoxel3, PlainVoxelArray>* volume3) {

// *** traversal vars
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table1 = volume1->index.GetEntries();

		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel3* voxels3 = volume3->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table3 = volume3->index.GetEntries();

		int hash_entry_count = volume1->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < hash_entry_count; hash++) {
			const ITMHashEntry& hash_entry1 = hash_table1[hash];
			if (hash_entry1.ptr < 0) continue;

			ITMHashEntry hash_entry2 = hash_table2[hash];
			ITMHashEntry hash_entry3 = hash_table3[hash];

			// the rare case where we have different positions for voxel1 & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				int hash2;
				if (!FindHashAtPosition(hash2, hash_entry1.pos, hash_table2)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << hash_entry1.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry2 = hash_table2[hash2];
				}
			}
			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (hash_entry3.pos != hash_entry2.pos) {
				int hash3;
				if (!FindHashAtPosition(hash3, hash_entry2.pos, hash_table3)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << hash_entry2.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry3 = hash_table3[hash3];
				}
			}
			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel3* voxel_block3 = &(voxels3[hash_entry3.ptr * VOXEL_BLOCK_SIZE3]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int indexWithinBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel2& voxel2 = voxel_block2[indexWithinBlock];
						TVoxel3& voxel3 = voxel_block3[indexWithinBlock];
						TVoxel1& voxel1 = voxel_block1[indexWithinBlock];
						TStaticFunctor::run(voxel2, voxel3, voxel1);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	Traverse(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume2,
			VoxelVolume<TVoxel3, PlainVoxelArray>* volume3,
			TFunctor& functor) {

// *** traversal vars
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table1 = volume1->index.GetEntries();

		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel3* voxels3 = volume3->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table3 = volume3->index.GetEntries();

		int hash_entry_count = volume1->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < hash_entry_count; hash++) {
			const ITMHashEntry& hash_entry2 = hash_table2[hash];
			if (hash_entry2.ptr < 0) continue;
			ITMHashEntry hash_entry1 = hash_table1[hash];
			ITMHashEntry hash_entry3 = hash_table3[hash];

			// the rare case where we have different positions for voxel1 & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				int voxel1Hash;
				if (!FindHashAtPosition(voxel1Hash, hash_entry2.pos, hash_table1)) {
					std::stringstream stream;
					stream << "Could not find corresponding voxel1 scene block at position "
					       << hash_entry1.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry1 = hash_table1[voxel1Hash];
				}
			}
			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (hash_entry3.pos != hash_entry2.pos) {
				int hash3;
				if (!FindHashAtPosition(hash3, hash_entry2.pos, hash_table3)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << hash_entry2.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry3 = hash_table3[hash3];
				}
			}
			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel3* voxel_block3 = &(voxels3[hash_entry3.ptr * VOXEL_BLOCK_SIZE3]);
			
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int indexWithinBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel2& voxel2 = voxel_block2[indexWithinBlock];
						TVoxel2& voxel3 = voxel_block3[indexWithinBlock];
						TVoxel1& voxel1 = voxel_block1[indexWithinBlock];
						functor(voxel2, voxel3, voxel1);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	TraverseWithPosition(
			VoxelVolume<TVoxel1, PlainVoxelArray>* volume2,
			VoxelVolume<TVoxel2, PlainVoxelArray>* volume3,
			VoxelVolume<TVoxel3, PlainVoxelArray>* volume1,
			TFunctor& functor) {
// *** traversal vars
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table1 = volume1->index.GetEntries();

		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel3* voxels3 = volume3->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table3 = volume3->index.GetEntries();

		int hashEntryCount = volume1->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < hashEntryCount; hash++) {
			const ITMHashEntry& hash_entry2 = hash_table2[hash];
			ITMHashEntry hash_entry1 = hash_table1[hash];
			ITMHashEntry hash_entry3 = hash_table3[hash];
			if (hash_entry2.ptr < 0) continue;
			// the rare case where we have different positions for voxel1 & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				int voxel1Hash;
				if (!FindHashAtPosition(voxel1Hash, hash_entry2.pos, hash_table1)) {
					std::stringstream stream;
					stream << "Could not find corresponding voxel1 scene block at position "
					       << hash_entry1.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry1 = hash_table1[voxel1Hash];
				}
			}
			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (hash_entry3.pos != hash_entry2.pos) {
				int hash3;
				if (!FindHashAtPosition(hash3, hash_entry2.pos, hash_table3)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << hash_entry2.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry3 = hash_table3[hash3];
				}
			}
			Vector3i hash_block_position = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE;

			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel3* voxel_block3 = &(voxels3[hash_entry3.ptr * VOXEL_BLOCK_SIZE3]);

			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						Vector3i voxel_position = hash_block_position + Vector3i(x, y, z);
						int index_within_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel1& voxel1 = voxel_block1[index_within_block];
						TVoxel2& voxel2 = voxel_block2[index_within_block];
						TVoxel2& voxel3 = voxel_block3[index_within_block];
						
						functor(voxel1, voxel2, voxel3, voxel_position);
					}
				}
			}
		}
	}

	template<typename TFunctor>
	inline static void
	TraverseWithPosition_DefaultForMissingEntries(VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
	                                              VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
	                                              VoxelVolume<TVoxel2, VoxelBlockHash>* volume3,
	                                              TFunctor& functor) {
// *** traversal vars
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table1 = volume1->index.GetEntries();

		TVoxel2* voxels3 = volume3->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table3 = volume3->index.GetEntries();

		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table2 = volume2->index.GetEntries();

		int hash_entry_count = volume1->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < hash_entry_count; hash++) {
			const ITMHashEntry& hash_entry1 = hash_table1[hash];
			if (hash_entry1.ptr < 0) continue;

			ITMHashEntry hash_entry2 = hash_table2[hash];
			ITMHashEntry hash_entry3 = hash_table3[hash];
			bool primaryFound = true, secondaryFound = true;
// the rare case where we have different positions for voxel1 & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				int hash2;
				if (!FindHashAtPosition(hash2, hash_entry1.pos, hash_table2)) {
					primaryFound = false;
				} else {
					hash_entry2 = hash_table2[hash2];
				}
			}
			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (hash_entry3.pos != hash_entry2.pos) {
				int hash3;
				if (!FindHashAtPosition(hash3, hash_entry2.pos, hash_table3)) {
					secondaryFound = false;
				} else {
					hash_entry3 = hash_table3[hash3];
				}
			}
			Vector3i hash_block_position = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE;

			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel3* voxel_block3 = &(voxels3[hash_entry3.ptr * VOXEL_BLOCK_SIZE3]);

			TVoxel2 defaultVoxel2;
			TVoxel3 defaultVoxel3;

			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxel_position = hash_block_position + Vector3i(x, y, z);
						int indexWithinBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel1& voxel1 = voxel_block1[indexWithinBlock];
						TVoxel2& voxel2 = primaryFound ? voxel_block2[indexWithinBlock] : defaultVoxel2;
						TVoxel2& voxel3 = secondaryFound ? voxel_block3[indexWithinBlock] : defaultVoxel3;
						functor(voxel1, voxel2, voxel3, voxel_position);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	TraversalWithPosition_ST(VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
	                         VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
	                         VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
	                         TFunctor& functor) {
// *** traversal vars
		TVoxel1* voxels1 = volume1->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table1 = volume1->index.GetEntries();

		TVoxel2* voxels2 = volume2->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table2 = volume2->index.GetEntries();

		TVoxel3* voxels3 = volume3->localVBA.GetVoxelBlocks();
		ITMHashEntry* hash_table3 = volume3->index.GetEntries();


		int hash_entry_count = volume1->index.hashEntryCount;

		for (int hash = 0; hash < hash_entry_count; hash++) {
			const ITMHashEntry& hash_entry1 = hash_table1[hash];
			if (hash_entry1.ptr < 0) continue;

			ITMHashEntry hash_entry2 = hash_table2[hash];
			ITMHashEntry hash_entry3 = hash_table3[hash];

			// the rare case where we have different positions for voxel1 & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (hash_entry2.pos != hash_entry1.pos) {
				int hash2;
				if (!FindHashAtPosition(hash2, hash_entry1.pos, hash_table2)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << hash_entry1.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry2 = hash_table2[hash2];
				}
			}
			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (hash_entry3.pos != hash_entry2.pos) {
				int hash3;
				if (!FindHashAtPosition(hash3, hash_entry2.pos, hash_table3)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << hash_entry2.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					hash_entry3 = hash_table3[hash3];
				}
			}

			Vector3i hash_block_position = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE;
			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel3* voxel_block3 = &(voxels3[hash_entry3.ptr * VOXEL_BLOCK_SIZE3]);

			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						Vector3i voxel_position = hash_block_position + Vector3i(x, y, z);
						int indexWithinBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel1& voxel1 = voxel_block1[indexWithinBlock];
						TVoxel2& voxel2 = voxel_block2[indexWithinBlock];
						TVoxel3& voxel3 = voxel_block3[indexWithinBlock];
						functor(voxel1, voxel2, voxel3, voxel_position);
					}
				}
			}
		}
	}
// endregion ===========================================================================================================
};


} // namespace ITMLib

