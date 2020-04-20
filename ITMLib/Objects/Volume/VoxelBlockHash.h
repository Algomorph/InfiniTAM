// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include <stdlib.h>
#include <fstream>
#include <iostream>

#endif

#include "../../Utils/Math.h"
#include "../../../ORUtils/MemoryBlock.h"
#include "../../../ORUtils/MemoryBlockPersister.h"
#include "../../Utils/HashBlockProperties.h"
#include "../../Utils/Serialization/Serialization.h"

#define VOXEL_BLOCK_SIZE 8
// VOXEL_BLOCK_SIZE3 = VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE
#define VOXEL_BLOCK_SIZE3 512

// Number of Hash Buckets, should be 2^n and bigger than DEFAULT_VOXEL_BLOCK_NUM, VOXEL_HASH_MASK = ORDERED_LIST_SIZE - 1
#define ORDERED_LIST_SIZE 0x100000
// Used to compute_allocated the hash of a bucket from a 3D location,  VOXEL_HASH_MASK = ORDERED_LIST_SIZE - 1
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
struct HashEntry {
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

	/** Times the hash block has been modified (only used by some algorithms) **/
	uchar modification_count;

	/** The frame index when the target block has been last modified. **/
	unsigned short last_modified_timestamp;

	friend std::ostream& operator<<(std::ostream& os, const HashEntry& entry) {
		os << entry.pos << " | " << entry.offset << " [" << entry.ptr << "]";
		return os;
	}
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
	typedef HashEntry IndexData;

	struct IndexCache {
		Vector3i blockPos;
		int blockPtr;
		_CPU_AND_GPU_CODE_ IndexCache() : blockPos(0x7fffffff), blockPtr(-1) {}
	};

	/** Maximum count of blocks in volume / entries in hash table. */
	const CONSTPTR(int) voxel_block_count;
	const CONSTPTR(int) excess_list_size;
	const CONSTPTR(int) hash_entry_count;
	const CONSTPTR(int) voxel_block_size = VOXEL_BLOCK_SIZE3;

private:
	int last_free_block_list_id;
	int last_free_excess_list_id;
	int utilized_block_count;
	int visible_block_count;

	/** The actual hash entries in the hash table, ordered by their hash codes. */
	ORUtils::MemoryBlock<HashEntry> hash_entries;
	/** States of hash entries during allocation procedures */
	ORUtils::MemoryBlock<HashEntryAllocationState> hash_entry_allocation_states;
	/** Voxel coordinates assigned to new hash blocks during allocation procedures */
	ORUtils::MemoryBlock<Vector3s> allocation_block_coordinates;
	/**
	 * Identifies which voxel blocks within the
	 * voxel storage can be allocated.
	 * **/
	ORUtils::MemoryBlock<int> block_allocation_list;
	/** Identifies which entries of the overflow
	 / excess list can be allocated. Any hash collisions
	 during allocation will cause the losing entry to be
	 allocated in the excess list. */
	ORUtils::MemoryBlock<int> excess_entry_list;
	/** A list of hash codes for "visible entries" */
	ORUtils::MemoryBlock<int> visible_block_hash_codes;
	/** A list of hash codes for utilized / allocated entries */
	ORUtils::MemoryBlock<int> utilized_block_hash_codes;
	/** Visibility types of "visible entries", ordered by hashCode */
	ORUtils::MemoryBlock<HashBlockVisibility> block_visibility_types;

public:
	const MemoryDeviceType memory_type;

	VoxelBlockHash();

	VoxelBlockHash(VoxelBlockHashParameters parameters, MemoryDeviceType memory_type);

	explicit VoxelBlockHash(MemoryDeviceType memory_type) : VoxelBlockHash(VoxelBlockHashParameters(),
	                                                                       memory_type) {}

	VoxelBlockHash(const VoxelBlockHash& other, MemoryDeviceType memory_type) :
			VoxelBlockHash({other.voxel_block_count, other.excess_list_size}, memory_type) {
		this->SetFrom(other);
	}

	void SetFrom(const VoxelBlockHash& other) {
		MemoryCopyDirection memory_copy_direction = determineMemoryCopyDirection(this->memory_type, other.memory_type);
		this->hash_entry_allocation_states.SetFrom(other.hash_entry_allocation_states, memory_copy_direction);
		this->hash_entries.SetFrom(other.hash_entries, memory_copy_direction);
		this->block_allocation_list.SetFrom(other.block_allocation_list, memory_copy_direction);
		this->excess_entry_list.SetFrom(other.excess_entry_list, memory_copy_direction);
		this->utilized_block_hash_codes.SetFrom(other.utilized_block_hash_codes, memory_copy_direction);
		this->last_free_block_list_id = other.last_free_block_list_id;
		this->last_free_excess_list_id = other.last_free_excess_list_id;
		this->utilized_block_count = other.utilized_block_count;
	}

	~VoxelBlockHash() = default;

	/** Get the list of actual entries in the hash table. */
	const HashEntry* GetEntries() const { return hash_entries.GetData(memory_type); }

	HashEntry* GetEntries() { return hash_entries.GetData(memory_type); }

	/** Get the list of actual entries in the hash table (alternative to GetEntries). */
	const IndexData* GetIndexData() const { return hash_entries.GetData(memory_type); }

	IndexData* GetIndexData() { return hash_entries.GetData(memory_type); }

	HashEntry GetHashEntry(int hash_code) const {
		return hash_entries.GetElement(hash_code, memory_type);
	}

	HashEntry GetHashEntryAt(const Vector3s& pos) const;
	HashEntry GetHashEntryAt(const Vector3s& pos, int& hash_code) const;
	HashEntry GetHashEntryAt(int x, int y, int z) const;
	HashEntry GetHashEntryAt(int x, int y, int z, int& hash_code) const;
	HashEntry GetUtilizedHashEntryAtIndex(int index, int& hash_code) const;
	HashEntry GetUtilizedHashEntryAtIndex(int index) const;

	/** Get a list of temporary hash entry state flags**/
	const HashEntryAllocationState* GetHashEntryAllocationStates() const {
		return hash_entry_allocation_states.GetData(memory_type);
	}

	HashEntryAllocationState* GetHashEntryAllocationStates() {
		return hash_entry_allocation_states.GetData(memory_type);
	}

	void ClearHashEntryAllocationStates() { hash_entry_allocation_states.Clear(NEEDS_NO_CHANGE); }

	const Vector3s* GetAllocationBlockCoordinates() const { return allocation_block_coordinates.GetData(memory_type); }

	/** Get a temporary list for coordinates of voxel blocks to be soon allocated**/
	Vector3s* GetAllocationBlockCoordinates() { return allocation_block_coordinates.GetData(memory_type); }

	const int* GetUtilizedBlockHashCodes() const { return utilized_block_hash_codes.GetData(memory_type); }

	int* GetUtilizedBlockHashCodes() { return utilized_block_hash_codes.GetData(memory_type); }

	const int* GetVisibleBlockHashCodes() const { return visible_block_hash_codes.GetData(memory_type); }

	int* GetVisibleBlockHashCodes() { return visible_block_hash_codes.GetData(memory_type); };

	HashBlockVisibility* GetBlockVisibilityTypes() { return block_visibility_types.GetData(memory_type); }

	const HashBlockVisibility* GetBlockVisibilityTypes() const { return block_visibility_types.GetData(memory_type); }

	/**
	 * \brief This list contains indices of blocks that are to be
	 *  allocated within the voxel store.
	 */
	const int* GetBlockAllocationList() const { return block_allocation_list.GetData(memory_type); }
	int* GetBlockAllocationList() { return block_allocation_list.GetData(memory_type); }

	/**
	 * Get the list that identifies which entries of the
	 * excess list are allocated. The excess list is only
	 * used when there is a hash collision during hash
	 * allocations, i.e. two hash positions map to the same
	 * hash code / bucket. In this case, the second block's
	 * entry will be put in the excess list and mapped to
	 * be accessable from the entry at the original bucket
	 * hash code.
	 * */
	const int* GetExcessEntryList() const { return excess_entry_list.GetData(memory_type); }
	int* GetExcessEntryList() { return excess_entry_list.GetData(memory_type); }

	int GetLastFreeBlockListId() const { return last_free_block_list_id; }
	int GetLastFreeExcessListId() const { return last_free_excess_list_id; }

	void
	SetLastFreeBlockListId(int last_free_block_list_id) { this->last_free_block_list_id = last_free_block_list_id; }
	void
	SetLastFreeExcessListId(int last_free_excess_list_id) { this->last_free_excess_list_id = last_free_excess_list_id; }


	int GetUtilizedBlockCount() const { return this->utilized_block_count; }

	void SetUtilizedBlockCount(
			int utilized_hash_block_count) { this->utilized_block_count = utilized_hash_block_count; }

	int GetVisibleBlockCount() const { return this->visible_block_count; }

	void SetVisibleBlockCount(
			int visible_hash_block_count) { this->visible_block_count = visible_hash_block_count; }

	/*VBH-specific*/
	int GetExcessListSize() const { return this->excess_list_size; }

	/** Number of allocated blocks. */
	int GetMaximumBlockCount() const { return this->voxel_block_count; }

	int GetVoxelBlockSize() const { return this->voxel_block_size; }

	unsigned int GetMaxVoxelCount() const {
		return static_cast<unsigned int>(this->voxel_block_count)
		       * static_cast<unsigned int>(this->voxel_block_size);
	}

	// Suppress the default copy constructor and assignment operator
	VoxelBlockHash(const VoxelBlockHash&) = delete;
	VoxelBlockHash& operator=(const VoxelBlockHash&) = delete;
};

} //namespace ITMLib