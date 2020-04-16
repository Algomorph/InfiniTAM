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
private:
// region =================================== HELPER FUNCTIONS =========================================================

	// the rare case where we have different positions for slave volume & master volume voxel blocks with the same index:
	// we have a hash bucket miss, try to find the slave block with the matching coordinates
	inline static void
	CheckSlaveVolumeBlock(HashEntry& slave_hash_entry, const HashEntry& master_hash_entry, HashEntry* slave_hash_table,
	                      const char* slave_volume_name) {
		if (slave_hash_entry.pos != master_hash_entry.pos) {
			int hash_code2;
			if (!FindHashAtPosition(hash_code2, master_hash_entry.pos, slave_hash_table)) {
				std::stringstream stream;
				stream << "Could not find corresponding " << slave_volume_name << " block at position "
				       << master_hash_entry.pos
				       << ". " << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
			} else {
				slave_hash_entry = slave_hash_table[hash_code2];
			}
		}
	}

	template<typename TFunction>
	inline static void TraverseBlocks(TVoxel1* voxel_block1, TVoxel2* voxel_block2, TVoxel3* voxel_block3,
	                                  TFunction&& function) {
		for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
			for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
				for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
					int index_within_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
					TVoxel2& voxel2 = voxel_block2[index_within_block];
					TVoxel3& voxel3 = voxel_block3[index_within_block];
					TVoxel1& voxel1 = voxel_block1[index_within_block];
					std::forward<TFunction>(function)(voxel1, voxel2, voxel3);
				}
			}
		}
	}

	template<typename TFunction>
	inline static void TraverseBlocksWithPosition(TVoxel1* voxel_block1, TVoxel2* voxel_block2, TVoxel3* voxel_block3,
	                                              const HashEntry& hash_entry1, TFunction&& function) {
		Vector3i hash_block_position = hash_entry1.pos.toInt() * VOXEL_BLOCK_SIZE;

		for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
			for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
				for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
					int incex_within_block = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
					Vector3i voxel_position = hash_block_position + Vector3i(x, y, z);
					TVoxel2& voxel2 = voxel_block2[incex_within_block];
					TVoxel3& voxel3 = voxel_block3[incex_within_block];
					TVoxel1& voxel1 = voxel_block1[incex_within_block];
					std::forward<TFunction>(function)(voxel1, voxel2, voxel3, voxel_position);
				}
			}
		}
	}

	template<typename TBlockTraversalFunction>
	inline static void
	TraverseAll_Generic(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
			TBlockTraversalFunction&& block_traverser) {

// *** traversal vars
		TVoxel1* voxels1 = volume1->GetVoxels();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		TVoxel2* voxels2 = volume2->GetVoxels();
		HashEntry* hash_table2 = volume2->index.GetEntries();
		TVoxel3* voxels3 = volume3->GetVoxels();
		HashEntry* hash_table3 = volume3->index.GetEntries();

		const int hash_entry_count = volume1->index.hash_entry_count;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash_code1 = 0; hash_code1 < hash_entry_count; hash_code1++) {
			const HashEntry& hash_entry1 = hash_table1[hash_code1];
			if (hash_entry1.ptr < 0) continue;
			HashEntry hash_entry2 = hash_table2[hash_code1];
			HashEntry hash_entry3 = hash_table3[hash_code1];

			CheckSlaveVolumeBlock(hash_entry2, hash_entry1, hash_table2, "volume 2");
			CheckSlaveVolumeBlock(hash_entry3, hash_entry1, hash_table3, "volume 3");

			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel3* voxel_block3 = &(voxels3[hash_entry3.ptr * VOXEL_BLOCK_SIZE3]);

			std::forward<TBlockTraversalFunction>(block_traverser)(voxel_block1, voxel_block2, voxel_block3, hash_entry1);
		}
	}

	template<typename TBlockTraversalFunction>
	inline static void
	TraverseUtilized_Generic(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
			TBlockTraversalFunction&& block_traverser) {

// *** traversal vars
		TVoxel1* voxels1 = volume1->GetVoxels();
		HashEntry* hash_table1 = volume1->index.GetEntries();
		TVoxel2* voxels2 = volume2->GetVoxels();
		HashEntry* hash_table2 = volume2->index.GetEntries();
		TVoxel3* voxels3 = volume3->GetVoxels();
		HashEntry* hash_table3 = volume3->index.GetEntries();

		const int utilized_entry_count = volume1->index.GetUtilizedBlockCount();
		const int* utilized_hash_codes = volume1->index.GetUtilizedBlockHashCodes();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash_code_index = 0; hash_code_index < utilized_entry_count; hash_code_index++) {
			int hash_code1 = utilized_hash_codes[hash_code_index];
			const HashEntry& hash_entry1 = hash_table1[hash_code1];
			if (hash_entry1.ptr < 0) continue;
			HashEntry hash_entry2 = hash_table2[hash_code1];
			HashEntry hash_entry3 = hash_table3[hash_code1];

			CheckSlaveVolumeBlock(hash_entry2, hash_entry1, hash_table2, "volume 2");
			CheckSlaveVolumeBlock(hash_entry3, hash_entry1, hash_table3, "volume 3");

			TVoxel1* voxel_block1 = &(voxels1[hash_entry1.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel2* voxel_block2 = &(voxels2[hash_entry2.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel3* voxel_block3 = &(voxels3[hash_entry3.ptr * VOXEL_BLOCK_SIZE3]);

			std::forward<TBlockTraversalFunction>(block_traverser)(voxel_block1, voxel_block2, voxel_block3, hash_entry1);
		}
	}

// endregion ===========================================================================================================
public:
// region ================================ STATIC THREE-SCENE TRAVERSAL ================================================

	template<typename TStaticFunctor>
	inline static void TraverseAll(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3) {
		TraverseAll_Generic(
				volume1, volume2, volume3,
				[](TVoxel1* voxel_block1, TVoxel2* voxel_block2, TVoxel3* voxel_block3, const HashEntry& hash_entry1){
					TraverseBlocks(
							voxel_block1, voxel_block2, voxel_block3, hash_entry1,
							[](TVoxel1& voxel1, TVoxel2& voxel2, TVoxel3& voxel3) {
								TStaticFunctor::run(voxel1, voxel2, voxel3);
							}
					);
				}
		);
	}

	template<typename TStaticFunctor>
	inline static void TraverseUtilized(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3) {
		TraverseUtilized_Generic(
				volume1, volume2, volume3,
				[](TVoxel1* voxel_block1, TVoxel2* voxel_block2, TVoxel3* voxel_block3, const HashEntry& hash_entry1){
					TraverseBlocks(
							voxel_block1, voxel_block2, voxel_block3, hash_entry1,
							[](TVoxel1& voxel1, TVoxel2& voxel2, TVoxel3& voxel3) {
								TStaticFunctor::run(voxel1, voxel2, voxel3);
							}
					);
				}
		);
	}

//endregion ============================================================================================================
// region ================================ DYNAMIC THREE-SCENE TRAVERSAL ===============================================

	template<typename TFunctor>
	inline static void
	TraverseAll(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
			TFunctor& functor) {
		TraverseAll_Generic(
				volume1, volume2, volume3,
				[&functor](TVoxel1* voxel_block1, TVoxel2* voxel_block2, TVoxel3* voxel_block3, const HashEntry& hash_entry1){
					TraverseBlocks(
							voxel_block1, voxel_block2, voxel_block3,
							[&functor](TVoxel1& voxel1, TVoxel2& voxel2, TVoxel3& voxel3) {
								functor(voxel1, voxel2, voxel3);
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
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
			TFunctor& functor) {
		TraverseUtilized_Generic(
				volume1, volume2, volume3,
				[&functor](TVoxel1* voxel_block1, TVoxel2* voxel_block2, TVoxel3* voxel_block3, const HashEntry& hash_entry1){
					TraverseBlocks(
							voxel_block1, voxel_block2, voxel_block3,
							[&functor](TVoxel1& voxel1, TVoxel2& voxel2, TVoxel3& voxel3) {
								functor(voxel1, voxel2, voxel3);
							}
					);
				}
		);
	}


	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
			TFunctor& functor) {
		TraverseAll_Generic(
				volume1, volume2, volume3,
				[&functor](TVoxel1* voxel_block1, TVoxel2* voxel_block2, TVoxel3* voxel_block3, const HashEntry& hash_entry1){
					TraverseBlocksWithPosition(
							voxel_block1, voxel_block2, voxel_block3, hash_entry1,
							[&functor](TVoxel1& voxel1, TVoxel2& voxel2, TVoxel3& voxel3, const Vector3i& voxel_position) {
								functor(voxel1, voxel2, voxel3, voxel_position);
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
			VoxelVolume<TVoxel3, VoxelBlockHash>* volume3,
			TFunctor& functor) {
		TraverseUtilized_Generic(
				volume1, volume2, volume3,
				[&functor](TVoxel1* voxel_block1, TVoxel2* voxel_block2, TVoxel3* voxel_block3, const HashEntry& hash_entry1){
					TraverseBlocksWithPosition(
							voxel_block1, voxel_block2, voxel_block3, hash_entry1,
							[&functor](TVoxel1& voxel1, TVoxel2& voxel2, TVoxel3& voxel3, const Vector3i& voxel_position) {
								functor(voxel1, voxel2, voxel3, voxel_position);
							}
					);
				}
		);
	}

// endregion ===========================================================================================================
};


} // namespace ITMLib

