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

namespace ITMLib{

//======================================================================================================================
//                            CONTAINS TRAVERSAL METHODS FOR VOLUMES USING VoxelBlockHash FOR INDEXING
//                                  (THREE VOLUMES WITH DIFFERING VOXEL TYPES)
//======================================================================================================================
template<typename TVoxel, typename TWarp>
class ThreeVolumeTraversalEngine<TVoxel, TWarp, VoxelBlockHash, MEMORYDEVICE_CPU> {
	/**
	 * \brief Concurrent traversal of three volumes with potentially-differing voxel types
	 * \details All volumes must have matching hash table size
	 */
public:
// region ================================ DYNAMIC THREE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void StaticDualVoxelTraversal(
			VoxelVolume<TVoxel, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxel, VoxelBlockHash>* secondaryScene,
			VoxelVolume<TWarp, VoxelBlockHash>* warpField) {

// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int noTotalEntries = warpField->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {
			const ITMHashEntry& currentWarpHashEntry = warpHashTable[hash];
			if (currentWarpHashEntry.ptr < 0) continue;

			ITMHashEntry currentPrimaryHashEntry = primaryHashTable[hash];
			ITMHashEntry currentSecondaryHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for warp & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (currentPrimaryHashEntry.pos != currentWarpHashEntry.pos) {
				int primaryHash;
				if (!FindHashAtPosition(primaryHash, currentWarpHashEntry.pos, primaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentWarpHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentPrimaryHashEntry = primaryHashTable[primaryHash];
				}
			}
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
			TVoxel* localPrimaryVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel* localSecondaryVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			TWarp* localWarpVoxelBlock = &(warpVoxels[currentWarpHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int indexWithinBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& primaryVoxel = localPrimaryVoxelBlock[indexWithinBlock];
						TVoxel& secondaryVoxel = localSecondaryVoxelBlock[indexWithinBlock];
						TWarp& warp = localWarpVoxelBlock[indexWithinBlock];
						TStaticFunctor::run(primaryVoxel, secondaryVoxel, warp);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelTraversal(
			VoxelVolume<TVoxel, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxel, VoxelBlockHash>* secondaryScene,
			VoxelVolume<TWarp, VoxelBlockHash>* warpField,
			TFunctor& functor) {

// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int noTotalEntries = warpField->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {
			const ITMHashEntry& currentPrimaryHashEntry = primaryHashTable[hash];
			if (currentPrimaryHashEntry.ptr < 0) continue;
			ITMHashEntry currentWarpHashEntry = warpHashTable[hash];
			ITMHashEntry currentSecondaryHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for warp & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (currentPrimaryHashEntry.pos != currentWarpHashEntry.pos) {
				int warpHash;
				if (!FindHashAtPosition(warpHash, currentPrimaryHashEntry.pos, warpHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding warp scene block at position "
					       << currentWarpHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentWarpHashEntry = warpHashTable[warpHash];
				}
			}
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
			TVoxel* localPrimaryVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel* localSecondaryVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			TWarp* localWarpVoxelBlock = &(warpVoxels[currentWarpHashEntry.ptr * VOXEL_BLOCK_SIZE3]);

			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int indexWithinBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& primaryVoxel = localPrimaryVoxelBlock[indexWithinBlock];
						TVoxel& secondaryVoxel = localSecondaryVoxelBlock[indexWithinBlock];
						TWarp& warp = localWarpVoxelBlock[indexWithinBlock];
						functor(primaryVoxel, secondaryVoxel, warp);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			VoxelVolume<TVoxel, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxel, VoxelBlockHash>* secondaryScene,
			VoxelVolume<TWarp, VoxelBlockHash>* warpField,
			TFunctor& functor) {
// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int hashEntryCount = warpField->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < hashEntryCount; hash++) {
			const ITMHashEntry& currentPrimaryHashEntry = primaryHashTable[hash];
			ITMHashEntry currentWarpHashEntry = warpHashTable[hash];
			ITMHashEntry currentSecondaryHashEntry = secondaryHashTable[hash];
			if (currentPrimaryHashEntry.ptr < 0) continue;
			// the rare case where we have different positions for warp & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (currentPrimaryHashEntry.pos != currentWarpHashEntry.pos) {
				int warpHash;
				if (!FindHashAtPosition(warpHash, currentPrimaryHashEntry.pos, warpHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding warp scene block at position "
					       << currentWarpHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentWarpHashEntry = warpHashTable[warpHash];
				}
			}
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
			Vector3i hashBlockPosition = currentWarpHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

			TVoxel* localPrimaryVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel* localSecondaryVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			TWarp* localWarpVoxelBlock = &(warpVoxels[currentWarpHashEntry.ptr * VOXEL_BLOCK_SIZE3]);

			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						int indexWithinBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& primaryVoxel = localPrimaryVoxelBlock[indexWithinBlock];
						TVoxel& secondaryVoxel = localSecondaryVoxelBlock[indexWithinBlock];
						TWarp& warp = localWarpVoxelBlock[indexWithinBlock];
						functor(primaryVoxel, secondaryVoxel, warp, voxelPosition);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal_DefaultForMissingEntries(
			VoxelVolume<TVoxel, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxel, VoxelBlockHash>* secondaryScene,
			VoxelVolume<TWarp, VoxelBlockHash>* warpField,
			TFunctor& functor) {

// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int noTotalEntries = warpField->index.hashEntryCount;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {
			const ITMHashEntry& currentWarpHashEntry = warpHashTable[hash];
			if (currentWarpHashEntry.ptr < 0) continue;

			ITMHashEntry currentPrimaryHashEntry = primaryHashTable[hash];
			ITMHashEntry currentSecondaryHashEntry = secondaryHashTable[hash];
			bool primaryFound = true, secondaryFound = true;
// the rare case where we have different positions for warp & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (currentPrimaryHashEntry.pos != currentWarpHashEntry.pos) {
				int primaryHash;
				if (!FindHashAtPosition(primaryHash, currentWarpHashEntry.pos, primaryHashTable)) {
					primaryFound = false;
				} else {
					currentPrimaryHashEntry = primaryHashTable[primaryHash];
				}
			}
			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentSecondaryHashEntry.pos != currentPrimaryHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentPrimaryHashEntry.pos, secondaryHashTable)) {
					secondaryFound = false;
				} else {
					currentSecondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			Vector3i hashBlockPosition = currentWarpHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

			TVoxel* localPrimaryVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel* localSecondaryVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			TWarp* localWarpVoxelBlock = &(warpVoxels[currentWarpHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel defaultVoxel;

			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						int indexWithinBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& primaryVoxel = primaryFound ? localPrimaryVoxelBlock[indexWithinBlock] : defaultVoxel;
						TVoxel& secondaryVoxel = secondaryFound ? localSecondaryVoxelBlock[indexWithinBlock]
						                                        : defaultVoxel;
						TWarp& warp = localWarpVoxelBlock[indexWithinBlock];
						functor(primaryVoxel, secondaryVoxel, warp, voxelPosition);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal_SingleThreaded(
			VoxelVolume<TVoxel, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxel, VoxelBlockHash>* secondaryScene,
			VoxelVolume<TWarp, VoxelBlockHash>* warpField,
			TFunctor& functor) {

// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int noTotalEntries = warpField->index.hashEntryCount;

		for (int hash = 0; hash < noTotalEntries; hash++) {
			const ITMHashEntry& currentWarpHashEntry = warpHashTable[hash];
			if (currentWarpHashEntry.ptr < 0) continue;

			ITMHashEntry currentPrimaryHashEntry = primaryHashTable[hash];
			ITMHashEntry currentSecondaryHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for warp & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (currentPrimaryHashEntry.pos != currentWarpHashEntry.pos) {
				int primaryHash;
				if (!FindHashAtPosition(primaryHash, currentWarpHashEntry.pos, primaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentWarpHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentPrimaryHashEntry = primaryHashTable[primaryHash];
				}
			}
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
			Vector3i hashBlockPosition = currentWarpHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			TVoxel* localPrimaryVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			TVoxel* localSecondaryVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			TWarp* localWarpVoxelBlock = &(warpVoxels[currentWarpHashEntry.ptr * VOXEL_BLOCK_SIZE3]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						int indexWithinBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& primaryVoxel = localPrimaryVoxelBlock[indexWithinBlock];
						TVoxel& secondaryVoxel = localSecondaryVoxelBlock[indexWithinBlock];
						TWarp& warp = localWarpVoxelBlock[indexWithinBlock];
						functor(primaryVoxel, secondaryVoxel, warp, voxelPosition);
					}
				}
			}
		}
	}
// endregion ===========================================================================================================
};


} // namespace ITMLib

