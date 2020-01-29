// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include <stdlib.h>
#include <fstream>
#include <iostream>

#endif

#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/MemoryBlock.h"
#include "../../../ORUtils/MemoryBlockPersister.h"
#include "../../Utils/ITMHashBlockProperties.h"
#include "../../Utils/Serialization/Serialization.h"

#define VOXEL_BLOCK_SIZE 8
// VOXEL_BLOCK_SIZE3 = VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE
#define VOXEL_BLOCK_SIZE3 512

// Number of Hash Buckets, should be 2^n and bigger than DEFAULT_VOXEL_BLOCK_NUM, VOXEL_HASH_MASK = ORDERED_LIST_SIZE - 1
#define ORDERED_LIST_SIZE 0x100000
// Used for get hashing value of the bucket index,  VOXEL_HASH_MASK = ORDERED_LIST_SIZE - 1
#define VOXEL_HASH_MASK 0xfffff


//// for loop closure
// Number of locally stored blocks, currently 2^12
//#define DEFAULT_VOXEL_BLOCK_NUM 0x10000

// Number of Hash Bucket, should be a power of two and bigger than voxel block count, VOXEL_HASH_MASK = ORDERED_LIST_SIZE - 1
//#define ORDERED_LIST_SIZE 0x40000


// Maximum number of blocks transfered in one swap operation
#define SWAP_OPERATION_BLOCK_COUNT 0x1000

/** \brief
	A single entry in the hash table.
*/
struct ITMHashEntry {
	/** Position of the corner of the 8x8x8 volume, that identifies the entry. */
	Vector3s pos;
	/** Offset in the excess list. */
	int offset;
	/** Pointer to the voxel block array.
		- >= 0 identifies an actual allocated entry in the voxel block array
		- -1 identifies an entry that has been removed (swapped out)
		- <-1 identifies an unallocated block
	*/
	int ptr;
};

namespace ITMLib {
/** \brief
This is the central class for the voxel block hash
implementation. It contains all the data needed on the CPU
and a pointer to the data structure on the GPU.
*/
class VoxelBlockHash {
public:
#define VOXEL_BLOCK_HASH_PARAMETERS_STRUCT_DESCRIPTION \
	VoxelBlockHashParameters, \
	(int, voxel_block_count, 0x40000, PRIMITIVE, "Total count of voxel hash blocks to preallocate."), \
	(int, excess_list_size, 0x20000, PRIMITIVE, \
	"Total count of voxel hash block entries in excess list of the hash table. " \
	"The excess list is used during hash collisions.")

	GENERATE_PATHLESS_SERIALIZABLE_STRUCT(VOXEL_BLOCK_HASH_PARAMETERS_STRUCT_DESCRIPTION);
	typedef VoxelBlockHashParameters InitializationParameters;
	typedef ITMHashEntry IndexData;

	struct IndexCache {
		Vector3i blockPos;
		int blockPtr;
		_CPU_AND_GPU_CODE_ IndexCache() : blockPos(0x7fffffff), blockPtr(-1) {}
	};

	/** Maximum number of total entries. */
	const CONSTPTR(int) voxelBlockCount;
	const CONSTPTR(int) excessListSize;
	const CONSTPTR(int) hashEntryCount;
	const CONSTPTR(int) voxelBlockSize = VOXEL_BLOCK_SIZE3;

private:
	int lastFreeExcessListId;
	int utilizedHashBlockCount;

	/** The actual hash entries in the hash table, ordered by their hash codes. */
	ORUtils::MemoryBlock<ITMHashEntry> hashEntries;
	/** States of hash entries during allocation procedures */
	ORUtils::MemoryBlock<HashEntryAllocationState> hashEntryAllocationStates;
	/** Voxel coordinates assigned to new hash blocks during allocation procedures */
	ORUtils::MemoryBlock<Vector3s> allocationBlockCoordinates;
	/** Identifies which entries of the overflow
	list are allocated. This is used if too
	many hash collisions caused the buckets to
	overflow. */
	ORUtils::MemoryBlock<int> excessAllocationList;
	/** A list of hash codes for "visible entries" */
	ORUtils::MemoryBlock<int> visibleBlockHashCodes;
	/** Visibility types of "visible entries", ordered by hashCode */
	ORUtils::MemoryBlock<HashBlockVisibility> blockVisibilityTypes;

public:
	const MemoryDeviceType memoryType;


	VoxelBlockHash(VoxelBlockHashParameters parameters, MemoryDeviceType memoryType);

	explicit VoxelBlockHash(MemoryDeviceType memoryType) : VoxelBlockHash(VoxelBlockHashParameters(),
	                                                                            memoryType) {}

	VoxelBlockHash(const VoxelBlockHash& other, MemoryDeviceType memoryType) :
			VoxelBlockHash({other.voxelBlockCount, other.excessListSize}, memoryType) {
		this->SetFrom(other);
	}

	void SetFrom(const VoxelBlockHash& other) {
		MemoryCopyDirection memoryCopyDirection = determineMemoryCopyDirection(this->memoryType, other.memoryType);
		this->hashEntryAllocationStates.SetFrom(&other.hashEntryAllocationStates, memoryCopyDirection);
		this->hashEntries.SetFrom(&other.hashEntries, memoryCopyDirection);
		this->excessAllocationList.SetFrom(&other.excessAllocationList, memoryCopyDirection);
		this->lastFreeExcessListId = other.lastFreeExcessListId;
		this->utilizedHashBlockCount = other.utilizedHashBlockCount;
	}

	~VoxelBlockHash() = default;

	/** Get the list of actual entries in the hash table. */
	const ITMHashEntry* GetEntries() const { return hashEntries.GetData(memoryType); }

	ITMHashEntry* GetEntries() { return hashEntries.GetData(memoryType); }

	/** Get the list of actual entries in the hash table (alternative to GetEntries). */
	const IndexData* GetIndexData() const { return hashEntries.GetData(memoryType); }

	IndexData* GetIndexData() { return hashEntries.GetData(memoryType); }

	ITMHashEntry GetHashEntry(int hashCode) const {
		return hashEntries.GetElement(hashCode, memoryType);
	}

	ITMHashEntry GetHashEntryAt(const Vector3s& pos) const;
	ITMHashEntry GetHashEntryAt(const Vector3s& pos, int& hashCode) const;

	ITMHashEntry GetHashEntryAt(int x, int y, int z) const {
		Vector3s coord(x, y, z);
		return GetHashEntryAt(coord);
	}

	ITMHashEntry GetHashEntryAt(int x, int y, int z, int& hashCode) const {
		Vector3s coord(x, y, z);
		return GetHashEntryAt(coord, hashCode);
	}

	/** Get a list of temporary hash entry state flags**/
	const HashEntryAllocationState* GetHashEntryAllocationStates() const {
		return hashEntryAllocationStates.GetData(memoryType);
	}

	HashEntryAllocationState* GetHashEntryAllocationStates() { return hashEntryAllocationStates.GetData(memoryType); }

	void ClearHashEntryAllocationStates() { hashEntryAllocationStates.Clear(NEEDS_NO_CHANGE); }

	const Vector3s* GetAllocationBlockCoordinates() const { return allocationBlockCoordinates.GetData(memoryType); }

	/** Get a temporary list for coordinates of voxel blocks to be soon allocated**/
	Vector3s* GetAllocationBlockCoordinates() { return allocationBlockCoordinates.GetData(memoryType); }

	const int* GetUtilizedBlockHashCodes() const { return visibleBlockHashCodes.GetData(memoryType); }

	int* GetUtilizedBlockHashCodes() { return visibleBlockHashCodes.GetData(memoryType); }

	HashBlockVisibility* GetBlockVisibilityTypes() { return blockVisibilityTypes.GetData(memoryType); }

	const HashBlockVisibility* GetBlockVisibilityTypes() const { return blockVisibilityTypes.GetData(memoryType); }

	/** Get the list that identifies which entries of the
	overflow list are allocated. This is used if too
	many hash collisions caused the buckets to overflow.
	*/
	const int* GetExcessAllocationList() const { return excessAllocationList.GetData(memoryType); }

	int* GetExcessAllocationList() { return excessAllocationList.GetData(memoryType); }

	int GetLastFreeExcessListId() const { return lastFreeExcessListId; }

	void SetLastFreeExcessListId(int newLastFreeExcessListId) { this->lastFreeExcessListId = newLastFreeExcessListId; }

	int GetUtilizedHashBlockCount() const { return this->utilizedHashBlockCount; }

	void SetUtilizedHashBlockCount(int utilizedHashBlockCount) { this->utilizedHashBlockCount = utilizedHashBlockCount; }

	/*VBH-specific*/
	int GetExcessListSize() const { return this->excessListSize; }

	/** Number of allocated blocks. */
	int GetAllocatedBlockCount() const { return this->voxelBlockCount; }

	int GetVoxelBlockSize() const { return this->voxelBlockSize; }

	unsigned int GetMaxVoxelCount() const {
		return static_cast<unsigned int>(this->voxelBlockCount)
		       * static_cast<unsigned int>(this->voxelBlockSize);
	}

	void SaveToDirectory(const std::string& outputDirectory) const;

	void LoadFromDirectory(const std::string& inputDirectory);


	// Suppress the default copy constructor and assignment operator
	VoxelBlockHash(const VoxelBlockHash&) = delete;
	VoxelBlockHash& operator=(const VoxelBlockHash&) = delete;
};

} //namespace ITMLib