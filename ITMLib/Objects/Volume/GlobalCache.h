// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifdef COMPILE_WITHOUT_CUDA
#include <cstring>
#endif

#include <cstdio>


#include "VoxelBlockHash.h"
#include "PlainVoxelArray.h"
#include "../../../ORUtils/CUDADefines.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class GlobalCache;

struct ITMHashSwapState {
	/// 0 - most recent data is on host, data not currently in active
	///     memory
	/// 1 - data both on host and in active memory, information has not
	///     yet been combined
	/// 2 - most recent data is in active memory, should save this data
	///     back to host at some point
	uchar state;
};

template<typename TVoxel>
class GlobalCache<TVoxel, PlainVoxelArray> {
public:
	explicit GlobalCache(const PlainVoxelArray& index);
};

template<typename TVoxel>
class GlobalCache<TVoxel, VoxelBlockHash> {
private:
	//TODO: rename vars to something more descriptive &/ document them
	int hash_entry_count;

	bool* has_stored_data;
	TVoxel* stored_voxel_blocks;
	ITMHashSwapState* swap_states_host;
	ITMHashSwapState* swap_states_device;

	bool* has_synced_data_host;
	bool* has_synced_data_device;
	TVoxel* synced_voxel_blocks_host;
	TVoxel* synced_voxel_blocks_device;

	int* needed_hash_codes_host;
	int* needed_hash_codes_device;
public:
	GlobalCache();
	explicit GlobalCache(const int hash_entry_count);

	explicit GlobalCache(const VoxelBlockHash& index) : GlobalCache(index.hash_entry_count) {}

	explicit GlobalCache(const GlobalCache& other);
	GlobalCache(GlobalCache&& other);

	GlobalCache& operator=(GlobalCache other) {
		swap(*this, other);
		return *this;
	}

	~GlobalCache();

	void Swap(GlobalCache<TVoxel, VoxelBlockHash>& rhs);

	friend void swap(GlobalCache<TVoxel, VoxelBlockHash>& lhs, GlobalCache<TVoxel, VoxelBlockHash>& rhs) { // nothrow
		lhs.Swap(rhs);
	}

	inline void SetStoredData(int address, TVoxel* data) {
		has_stored_data[address] = true;
		memcpy(stored_voxel_blocks + address * VOXEL_BLOCK_SIZE3, data, sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
	}

	inline bool HasStoredData(int address) const { return has_stored_data[address]; }

	inline TVoxel* GetStoredVoxelBlock(int address) { return stored_voxel_blocks + address * VOXEL_BLOCK_SIZE3; }

	bool* GetHasSyncedData(bool useGPU) const { return useGPU ? has_synced_data_device : has_synced_data_host; }

	TVoxel* GetSyncedVoxelBlocks(bool useGPU) const {
		return useGPU ? synced_voxel_blocks_device : synced_voxel_blocks_host;
	}

	ITMHashSwapState* GetSwapStates(bool useGPU) { return useGPU ? swap_states_device : swap_states_host; }

	int* GetNeededEntryIDs(bool useGPU) { return useGPU ? needed_hash_codes_device : needed_hash_codes_host; }

	int GetHashEntryCount() const;

	void SaveToFile(char* path) const {
		TVoxel* stored_data = stored_voxel_blocks;

		FILE* f = fopen(path, "wb");

		fwrite(has_stored_data, sizeof(bool), hash_entry_count, f);
		for (int i = 0; i < hash_entry_count; i++) {
			fwrite(stored_data, sizeof(TVoxel) * VOXEL_BLOCK_SIZE3, 1, f);
			stored_data += VOXEL_BLOCK_SIZE3;
		}

		fclose(f);
	}

	void ReadFromFile(char* path) {
		TVoxel* stored_data = stored_voxel_blocks;
		FILE* f = fopen(path, "rb");

		size_t tmp = fread(has_stored_data, sizeof(bool), hash_entry_count, f);
		if (tmp == (size_t) hash_entry_count) {
			for (int i = 0; i < hash_entry_count; i++) {
				fread(stored_data, sizeof(TVoxel) * VOXEL_BLOCK_SIZE3, 1, f);
				stored_data += VOXEL_BLOCK_SIZE3;
			}
		}

		fclose(f);
	}
};
} // namespace ITMLib
