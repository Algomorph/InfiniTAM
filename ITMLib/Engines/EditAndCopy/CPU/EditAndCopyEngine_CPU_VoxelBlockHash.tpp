//  ================================================================
//  Created by Gregory Kramida on 9/26/19.
//  Copyright (c) 2019 Gregory Kramida
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


#include "EditAndCopyEngine_CPU.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../DepthFusion/DepthFusionEngineFactory.h"
#include "../../Traversal/Shared/VolumeTraversal_Shared.h"
#include "../Shared/EditAndCopyEngine_Shared.h"
#include "../../Indexing/Interface/IndexingEngine.h"
#include "../../Indexing/Shared/IndexingEngine_Shared.h"

using namespace ITMLib;

// region ==================================== Voxel Hash Volume EditAndCopy Engine ====================================

template<typename TVoxel>
void EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::ResetVolume(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	int numBlocks = volume->index.GetAllocatedBlockCount();
	int blockSize = volume->index.GetVoxelBlockSize();

	TVoxel* voxelBlocks_ptr = volume->localVBA.GetVoxelBlocks();
	for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
	int* vbaAllocationList_ptr = volume->localVBA.GetAllocationList();
	for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
	volume->localVBA.lastFreeBlockId = numBlocks - 1;

	HashEntry tmpEntry;
	memset(&tmpEntry, 0, sizeof(HashEntry));
	tmpEntry.ptr = -2;
	HashEntry* hashEntry_ptr = volume->index.GetEntries();
	for (int i = 0; i < volume->index.hashEntryCount; ++i) hashEntry_ptr[i] = tmpEntry;
	int* excessList_ptr = volume->index.GetExcessAllocationList();
	for (int i = 0; i < volume->index.GetExcessListSize(); ++i) excessList_ptr[i] = i;

	volume->index.SetLastFreeExcessListId(volume->index.GetExcessListSize() - 1);
}

template<typename TVoxel>
bool
EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::SetVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
                                                        Vector3i at, TVoxel voxel) {

	HashEntry* hashTable = volume->index.GetEntries();
	TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
	int hashCode = -1;
	Vector3s blockPos;
	int voxelIndexInBlock = pointToVoxelBlockPos(at, blockPos);
	if (IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
			.AllocateHashBlockAt(volume, blockPos, hashCode)) {
		HashEntry& entry = hashTable[hashCode];
		TVoxel* localVoxelBlock = &(voxels[entry.ptr * (VOXEL_BLOCK_SIZE3)]);
		localVoxelBlock[voxelIndexInBlock] = voxel;
		return true;
	} else {
		return false;
	}
}

template<typename TVoxel>
bool EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::SetVoxelNoAllocation(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume,
		Vector3i at, TVoxel voxel) {
	HashEntry* hashTable = volume->index.GetEntries();
	TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(at, blockPos);
	int hash_index;
	if (FindHashAtPosition(hash_index, blockPos.toShortFloor(), hashTable)) {
		TVoxel* localVoxelBlock = &(voxels[hash_index]);
		localVoxelBlock[linearIdx] = voxel;
	} else {
		return false;
	}
	return true;
}


template<typename TVoxel>
TVoxel
EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::ReadVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
                                                         Vector3i at) {
	TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
	HashEntry* hashTable = volume->index.GetEntries();
	int vmIndex;
	return readVoxel(voxels, hashTable, at, vmIndex);
}

template<typename TVoxel>
TVoxel
EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::ReadVoxel(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
                                                         Vector3i at,
                                                         VoxelBlockHash::IndexCache& cache) {
	TVoxel* voxels = volume->localVBA.GetVoxelBlocks();
	HashEntry* hashTable = volume->index.GetEntries();
	int vmIndex;
	return readVoxel(voxels, hashTable, at, vmIndex, cache);
}

template<typename TVoxel>
void
EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::OffsetWarps(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
                                                           Vector3f offset) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented!");
}

template<typename TVoxel>
bool EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::CopyVolumeSlice(
		VoxelVolume<TVoxel, VoxelBlockHash>* targetVolume, VoxelVolume<TVoxel, VoxelBlockHash>* sourceVolume,
		Vector6i bounds, const Vector3i& offset) {

	assert(targetVolume->index.hashEntryCount == sourceVolume->index.hashEntryCount);

	//temporary stuff
	const int hashEntryCount = sourceVolume->index.hashEntryCount;
	ORUtils::MemoryBlock<HashEntryAllocationState> hashEntryStates(hashEntryCount, MEMORYDEVICE_CPU);
	HashEntryAllocationState* hashEntryStates_device = hashEntryStates.GetData(MEMORYDEVICE_CPU);
	ORUtils::MemoryBlock<Vector3s> blockCoords(hashEntryCount, MEMORYDEVICE_CPU);
	Vector3s* blockCoords_device = blockCoords.GetData(MEMORYDEVICE_CPU);

	TVoxel* sourceVoxels = sourceVolume->localVBA.GetVoxelBlocks();
	const HashEntry* sourceHashTable = sourceVolume->index.GetEntries();

	HashEntry* destinationHashTable = targetVolume->index.GetEntries();
	TVoxel* destinationVoxels = targetVolume->localVBA.GetVoxelBlocks();

	bool voxelsWereCopied = false;

	if (offset == Vector3i(0)) {
		// *** allocate missing entries in target hash table
		// traverse source hash blocks, see which ones are at least partially inside the specified bounds
		//TODO: move this functionality to indexing engine and call that instead
		for (int sourceHash = 0; sourceHash < hashEntryCount; sourceHash++) {
			const HashEntry& currentSourceHashEntry = sourceHashTable[sourceHash];
			if (currentSourceHashEntry.ptr < 0) continue;
			Vector3i originalHashBlockPosition = currentSourceHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			if (IsHashBlockFullyInRange(originalHashBlockPosition, bounds) ||
			    IsHashBlockPartiallyInRange(originalHashBlockPosition, bounds)) {
				int destinationHash = HashCodeFromBlockPosition(currentSourceHashEntry.pos);
				bool collisionDetected = false;
				MarkAsNeedingAllocationIfNotFound(hashEntryStates_device, blockCoords_device, destinationHash,
				                                  currentSourceHashEntry.pos, destinationHashTable, collisionDetected);
			}
		}

		IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().
				AllocateHashEntriesUsingLists(targetVolume);

		//iterate over source hash blocks & fill in the target hash blocks
		for (int sourceHash = 0; sourceHash < hashEntryCount; sourceHash++) {
			const HashEntry& sourceHashEntry = sourceHashTable[sourceHash];

			if (sourceHashEntry.ptr < 0) continue;
			int destinationHash;
			FindHashAtPosition(destinationHash, sourceHashEntry.pos, destinationHashTable);
			const HashEntry& destinationHashEntry = destinationHashTable[destinationHash];

			//position of the current entry in 3D space (in voxel units)
			Vector3i sourceHashBlockPositionVoxels = sourceHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			TVoxel* localSourceVoxelBlock = &(sourceVoxels[sourceHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel* localDestinationVoxelBlock = &(destinationVoxels[destinationHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			if (IsHashBlockFullyInRange(sourceHashBlockPositionVoxels, bounds)) {
				//we can safely copy the whole block
				memcpy(localDestinationVoxelBlock, localSourceVoxelBlock, sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
				voxelsWereCopied = true;
			} else if (IsHashBlockPartiallyInRange(sourceHashBlockPositionVoxels, bounds)) {
				//we have to copy only parts of the scene that are within bounds
				int zRangeStart, zRangeEnd, yRangeStart, yRangeEnd, xRangeStart, xRangeEnd;
				ComputeCopyRanges(xRangeStart, xRangeEnd, yRangeStart, yRangeEnd, zRangeStart, zRangeEnd,
				                  sourceHashBlockPositionVoxels, bounds);
				for (int z = zRangeStart; z < zRangeEnd; z++) {
					for (int y = yRangeStart; y < yRangeEnd; y++) {
						for (int x = xRangeStart; x < xRangeEnd; x++) {
							int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
							memcpy(&localDestinationVoxelBlock[locId], &localSourceVoxelBlock[locId], sizeof(TVoxel));
						}
					}
				}
				voxelsWereCopied = true;
			}

		}
	} else {
		//non-zero-offset case

		//We need to allocated needed blocks in the destination hash
		//This can be done by using the destination bounds
		Vector6i destination_bounds(bounds.min_x + offset.x, bounds.min_y + offset.y, bounds.min_z + offset.z,
		                            bounds.max_x + offset.x, bounds.max_y + offset.y, bounds.max_z + offset.z);

		Vector3i destinationMinPoint(destination_bounds.min_x, destination_bounds.min_y, destination_bounds.min_z);
		Vector3i destinationMaxPoint(destination_bounds.max_x, destination_bounds.max_y, destination_bounds.max_z);
		Vector3i minPointBlock, maxPointBlock;
		pointToVoxelBlockPos(destinationMinPoint, minPointBlock);
		pointToVoxelBlockPos(destinationMaxPoint, maxPointBlock);
		for (int block_z = minPointBlock.z; block_z < maxPointBlock.z; block_z++) {
			for (int block_y = minPointBlock.y; block_y < maxPointBlock.y; block_y++) {
				for (int block_x = minPointBlock.x; block_x < maxPointBlock.x; block_x++) {
					Vector3i blockPos(block_x, block_y, block_x);
					int destinationHash = HashCodeFromBlockPosition(blockPos);
					bool collisionDetected = false;
					MarkAsNeedingAllocationIfNotFound(hashEntryStates_device, blockCoords_device, destinationHash,
					                                  blockPos.toShortFloor(), destinationHashTable, collisionDetected);
				}
			}
		}

		IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().
				AllocateHashEntriesUsingLists(targetVolume);
		VoxelBlockHash::IndexCache source_cache;

		for (int source_z = bounds.min_z; source_z < bounds.max_z; source_z++) {
			for (int source_y = bounds.min_y; source_y < bounds.min_y; source_y++) {
				for (int source_x = bounds.min_x; source_x < bounds.max_x; source_x++) {
					Vector3i source_point(source_x, source_y, source_z);
					Vector3i destination_point = source_point + offset;
					TVoxel source_voxel =
							EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::
							ReadVoxel(sourceVolume, source_point, source_cache);
					EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::
					SetVoxelNoAllocation(targetVolume, destination_point, source_voxel);
					voxelsWereCopied = true;
				}
			}
		}
	}

	return voxelsWereCopied;
}

template<typename TVoxel>
bool EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::CopyVolume(
		VoxelVolume<TVoxel, VoxelBlockHash>* targetVolume, VoxelVolume<TVoxel, VoxelBlockHash>* sourceVolume,
		const Vector3i& offset) {

	assert(targetVolume->index.hashEntryCount == sourceVolume->index.hashEntryCount);

	//reset destination scene
	EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::ResetVolume(targetVolume);

	const int hashEntryCount = sourceVolume->index.hashEntryCount;

	//temporary stuff
	ORUtils::MemoryBlock<HashEntryAllocationState> hashEntryStates(hashEntryCount, MEMORYDEVICE_CPU);
	ORUtils::MemoryBlock<Vector3s> blockCoordinates(hashEntryCount, MEMORYDEVICE_CPU);
	HashEntryAllocationState* hashEntryStates_device = hashEntryStates.GetData(MEMORYDEVICE_CPU);
	Vector3s* blockCoordinates_device = blockCoordinates.GetData(MEMORYDEVICE_CPU);

	TVoxel* sourceVoxels = sourceVolume->localVBA.GetVoxelBlocks();
	const HashEntry* sourceHashTable = sourceVolume->index.GetEntries();

	HashEntry* destinationHashTable = targetVolume->index.GetEntries();
	TVoxel* destinationVoxels = targetVolume->localVBA.GetVoxelBlocks();

	bool voxelsWereCopied = false;

	if (offset == Vector3i(0)) {
		IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().AllocateUsingOtherVolume(targetVolume, sourceVolume);
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(sourceHashTable,destinationHashTable,sourceVoxels,destinationVoxels,voxelsWereCopied)
#endif
		for (int sourceHash = 0; sourceHash < hashEntryCount; sourceHash++) {
			const HashEntry& sourceHashEntry = sourceHashTable[sourceHash];

			if (sourceHashEntry.ptr < 0) continue;
			int destinationHash;
			FindHashAtPosition(destinationHash, sourceHashEntry.pos, destinationHashTable);
			const HashEntry& destinationHashEntry = destinationHashTable[destinationHash];
			//position of the current entry in 3D space (in voxel units)
			TVoxel* localSourceVoxelBlock = &(sourceVoxels[sourceHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			TVoxel* localDestinationVoxelBlock = &(destinationVoxels[destinationHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			//we can safely copy the whole block
			memcpy(localDestinationVoxelBlock, localSourceVoxelBlock, sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
			voxelsWereCopied = true;
		}
	} else {
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(sourceHashTable,sourceVoxels,voxelsWereCopied,offset,targetVolume)
#endif
		// traverse source hash blocks
		for (int sourceHash = 0; sourceHash < hashEntryCount; sourceHash++) {
			const HashEntry& currentSourceHashEntry = sourceHashTable[sourceHash];
			if (currentSourceHashEntry.ptr < 0) continue;

			Vector3i sourceBlockPos;
			sourceBlockPos.x = currentSourceHashEntry.pos.x;
			sourceBlockPos.y = currentSourceHashEntry.pos.y;
			sourceBlockPos.z = currentSourceHashEntry.pos.z;
			sourceBlockPos *= VOXEL_BLOCK_SIZE;

			TVoxel* localSourceVoxelBlock = &(sourceVoxels[currentSourceHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);

			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId;
						Vector3i sourcePoint(sourceBlockPos.x + x, sourceBlockPos.y + y, sourceBlockPos.z + z);
						Vector3i destinationPoint = sourcePoint + offset;
						locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel sourceVoxel = localSourceVoxelBlock[locId];
						EditAndCopyEngine_CPU<TVoxel, VoxelBlockHash>::
						SetVoxel(targetVolume, destinationPoint, sourceVoxel);
						voxelsWereCopied = true;
					}
				}
			}
		}
	}
	return voxelsWereCopied;
}

// endregion ===========================================================================================================

