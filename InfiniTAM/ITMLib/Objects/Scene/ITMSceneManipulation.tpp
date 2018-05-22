//  ================================================================
//  Created by Gregory Kramida on 11/5/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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

#include "ITMSceneManipulation.h"
#include "ITMScene.h"
#include "ITMRepresentationAccess.h"
#include "../../Utils/ITMLibSettings.h"
#include "../../Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"


namespace ITMLib {


template<typename TVoxelSource, typename TVoxelDestination, typename TIndex >
void CopySceneSDFandFlagsWithOffset_CPU(ITMScene<TVoxelDestination, TIndex>* destination,
                                        ITMScene<TVoxelSource, TIndex>* source, Vector3i offset) {
	ITMSceneReconstructionEngine<TVoxelDestination, TIndex>* reconstructionEngine =
			ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxelDestination, TIndex>(
					ITMLibSettings::DEVICE_CPU);

	reconstructionEngine->ResetScene(destination);

	TVoxelSource* originalVoxels = source->localVBA.GetVoxelBlocks();
	const ITMHashEntry* originalHashTable = source->index.GetEntries();
	int noTotalEntries = source->index.noTotalEntries;

	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentOriginalHashEntry = originalHashTable[entryId];
		if (currentOriginalHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space (in voxel units)
		Vector3i canonicalHashEntryPosition = currentOriginalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		TVoxelSource* localVoxelBlock = &(originalVoxels[currentOriginalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);
					Vector3i offsetPosition = originalPosition + offset;
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelSource& voxelSource = localVoxelBlock[locId];
					TVoxelDestination voxelDestination;
					voxelDestination.sdf = voxelSource.sdf;
					voxelDestination.flags = voxelSource.flags;
					SetVoxel_CPU(destination, offsetPosition, voxelDestination);
				}
			}
		}
	}

}

template<class TVoxel, class TIndex>
bool SetVoxel_CPU(ITMScene <TVoxel, TIndex>* scene, Vector3i at, TVoxel voxel) {
	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();
	ITMHashEntry* hashTable = scene->index.GetEntries();
	ITMHashEntry* entry = NULL;
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	int* excessAllocationList = scene->index.GetExcessAllocationList();
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(at, blockPos);
	int hash;
	if (AllocateHashEntry_CPU(TO_SHORT_FLOOR3(blockPos), hashTable, entry, lastFreeVoxelBlockId, lastFreeExcessListId,
	                          voxelAllocationList, excessAllocationList, hash)) {
		TVoxel* localVoxelBlock = &(voxels[entry->ptr * (SDF_BLOCK_SIZE3)]);
		localVoxelBlock[linearIdx] = voxel;
	} else {
		return false;
	}
	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);
	return true;
};


template<class TVoxel, class TIndex>
void OffsetWarps(ITMScene<TVoxel, TIndex>& scene, Vector3f offset) {
	TVoxel* voxels = scene.localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = scene.index.GetEntries();
	int noTotalEntries = scene.index.noTotalEntries;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = hashTable[entryId];
		if (currentHashEntry.ptr < 0) continue;
		TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					//localVoxelBlock[locId].gradient = offset;
					localVoxelBlock[locId].warp += offset;
				}
			}
		}
	}
}

template<class TVoxel, class TIndex>
TVoxel ReadVoxel(ITMScene<TVoxel, TIndex>& scene, Vector3i at) {
	TVoxel* voxels = scene.localVBA.GetVoxelBlocks();
	ITMHashEntry* hashTable = scene.index.GetEntries();
	int vmIndex;
	return readVoxel(voxels, hashTable, at, vmIndex);
}




/**
 * \brief Copies the slice (box-like window) specified by points extremum1 and extremum2 from the source scene into a
 * destination scene. Clears the destination scene before copying.
 * \tparam TVoxel type of voxel
 * \tparam TIndex type of voxel index
 * \param destination destination voxel grid (can be uninitialized)
 * \param source source voxel grid
 * \param minPoint minimum point in the desired slice (inclusive), i.e. minimum x, y, and z coordinates
 * \param maxPoint maximum point in the desired slice (inclusive), i.e. maximum x, y, and z coordinates
 * \return true on success (destination scene contains the slice), false on failure (there are no allocated hash blocks
 */
template<class TVoxel, class TIndex>
bool CopySceneSlice_CPU(ITMScene<TVoxel, TIndex>* destination, ITMScene<TVoxel, TIndex>* source,
                        Vector3i minPoint, Vector3i maxPoint) {

	// prep destination scene
	ITMSceneReconstructionEngine<TVoxel, TIndex>* reconstructionEngine =
			ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxel, TIndex>(
					ITMLibSettings::DEVICE_CPU);
	reconstructionEngine->ResetScene(destination);

	//temporary stuff
	ORUtils::MemoryBlock<unsigned char>* entryAllocationTypes
			= new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU);
	ORUtils::MemoryBlock<Vector3s>* blockCoords = new ORUtils::MemoryBlock<Vector3s>(TIndex::noTotalEntries,
	                                                                                 MEMORYDEVICE_CPU);
	uchar* entriesAllocType = entryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	Vector3s* allocationBlockCoords = blockCoords->GetData(MEMORYDEVICE_CPU);

	TVoxel* sourceVoxels = source->localVBA.GetVoxelBlocks();
	const ITMHashEntry* sourceHashTable = source->index.GetEntries();
	int totalHashEntryCount = source->index.noTotalEntries;
	ITMHashEntry* destinationHashTable = destination->index.GetEntries();
	TVoxel* destinationVoxels = destination->localVBA.GetVoxelBlocks();


	for (int sourceHash = 0; sourceHash < totalHashEntryCount; sourceHash++) {
		const ITMHashEntry& currentOriginalHashEntry = sourceHashTable[sourceHash];
		if (currentOriginalHashEntry.ptr < 0) continue;
		Vector3i originalHashBlockPosition = currentOriginalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		if (IsHashBlockFullyInRange(originalHashBlockPosition, minPoint, maxPoint) ||
		    IsHashBlockPartiallyInRange(originalHashBlockPosition, minPoint, maxPoint)) {
			int destinationHash = hashIndex(currentOriginalHashEntry.pos);
			MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, destinationHash,
			                                  currentOriginalHashEntry.pos, destinationHashTable);
		}
	}

	AllocateHashEntriesUsingLists_CPU(destination, entriesAllocType, allocationBlockCoords, ITMLib::STABLE);

	delete blockCoords;
	delete entryAllocationTypes;

	bool newSceneContainsVoxels = false;

	for (int sourceHash = 0; sourceHash < totalHashEntryCount; sourceHash++) {
		const ITMHashEntry& sourceHashEntry = sourceHashTable[sourceHash];

		if (sourceHashEntry.ptr < 0) continue;
		int destinationHash;
		FindHashAtPosition(destinationHash, sourceHashEntry.pos, destinationHashTable);
		const ITMHashEntry& destinationHashEntry = destinationHashTable[destinationHash];

		//position of the current entry in 3D space (in voxel units)
		Vector3i sourceHashBlockPositionVoxels = sourceHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		TVoxel* localSourceVoxelBlock = &(sourceVoxels[sourceHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		TVoxel* localDestinationVoxelBlock = &(destinationVoxels[destinationHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		if (IsHashBlockFullyInRange(sourceHashBlockPositionVoxels, minPoint, maxPoint)) {
			//we can safely copy the whole block
			memcpy(localDestinationVoxelBlock, localSourceVoxelBlock, sizeof(TVoxel) * SDF_BLOCK_SIZE3);
			newSceneContainsVoxels = true;
		} else if (IsHashBlockPartiallyInRange(sourceHashBlockPositionVoxels, minPoint, maxPoint)) {
			int zRangeStart, zRangeEnd, yRangeStart, yRangeEnd, xRangeStart, xRangeEnd;
			ComputeCopyRanges(xRangeStart, xRangeEnd, yRangeStart, yRangeEnd, zRangeStart, zRangeEnd,
			                  sourceHashBlockPositionVoxels, minPoint, maxPoint);
			for (int z = zRangeStart; z < zRangeEnd; z++) {
				for (int y = yRangeStart; y < yRangeEnd; y++) {
					for (int x = xRangeStart; x < xRangeEnd; x++) {
						int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						memcpy(&localDestinationVoxelBlock[locId], &localSourceVoxelBlock[locId], sizeof(TVoxel));
					}
				}
			}
			newSceneContainsVoxels = true;
		}

	}
	return newSceneContainsVoxels;
};

}