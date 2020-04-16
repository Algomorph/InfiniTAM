// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "SwappingEngine_CPU.h"

#include "../Shared/SwappingEngine_Shared.h"
using namespace ITMLib;


template<class TVoxel>
int SwappingEngine_CPU<TVoxel, VoxelBlockHash>::LoadFromGlobalMemory(VoxelVolume<TVoxel, VoxelBlockHash> *scene)
{
	GlobalCache<TVoxel, VoxelBlockHash> *globalCache = scene->global_cache;

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);

	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

	TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);

	int hashEntryCount = globalCache->GetHashEntryCount();

	int neededHashEntryCount = 0;
	for (int entryId = 0; entryId < hashEntryCount; entryId++)
	{
		if (neededHashEntryCount >= SWAP_OPERATION_BLOCK_COUNT) break;
		if (swapStates[entryId].state == 1)
		{
			neededEntryIDs_local[neededHashEntryCount] = entryId;
			neededHashEntryCount++;
		}
	}

	// would copy neededEntryIDs_local into neededEntryIDs_global here

	if (neededHashEntryCount > 0)
	{
		memset(syncedVoxelBlocks_global, 0, neededHashEntryCount * VOXEL_BLOCK_SIZE3 * sizeof(TVoxel));
		memset(hasSyncedData_global, 0, neededHashEntryCount * sizeof(bool));
		for (int i = 0; i < neededHashEntryCount; i++)
		{
			int entryId = neededEntryIDs_global[i];

			if (globalCache->HasStoredData(entryId))
			{
				hasSyncedData_global[i] = true;
				memcpy(syncedVoxelBlocks_global + i * VOXEL_BLOCK_SIZE3, globalCache->GetStoredVoxelBlock(entryId), VOXEL_BLOCK_SIZE3 * sizeof(TVoxel));
			}
		}
	}

	// would copy syncedVoxelBlocks_global and hasSyncedData_global and syncedVoxelBlocks_local and hasSyncedData_local here

	return neededHashEntryCount;
}

template<class TVoxel>
void SwappingEngine_CPU<TVoxel, VoxelBlockHash>::IntegrateGlobalIntoLocal(VoxelVolume<TVoxel, VoxelBlockHash> *scene, RenderState *renderState)
{
	GlobalCache<TVoxel, VoxelBlockHash> *globalCache = scene->global_cache;

	HashEntry *hashTable = scene->index.GetEntries();

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);

	TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_local = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

	TVoxel *localVBA = scene->GetVoxelBlocks();

	int noNeededEntries = this->LoadFromGlobalMemory(scene);

	int maxW = scene->parameters->max_integration_weight;

	for (int i = 0; i < noNeededEntries; i++)
	{
		int entryDestId = neededEntryIDs_local[i];

		if (hasSyncedData_local[i])
		{
			TVoxel *srcVB = syncedVoxelBlocks_local + i * VOXEL_BLOCK_SIZE3;
			TVoxel *dstVB = localVBA + hashTable[entryDestId].ptr * VOXEL_BLOCK_SIZE3;

			for (int vIdx = 0; vIdx < VOXEL_BLOCK_SIZE3; vIdx++)
			{
				CombineVoxelInformation<TVoxel::hasColorInformation, TVoxel>::compute(srcVB[vIdx], dstVB[vIdx], maxW);
			}
		}

		swapStates[entryDestId].state = 2;
	}
}

template<class TVoxel>
void SwappingEngine_CPU<TVoxel, VoxelBlockHash>::SaveToGlobalMemory(VoxelVolume<TVoxel, VoxelBlockHash> *scene, RenderState *renderState)
{
	GlobalCache<TVoxel, VoxelBlockHash>* global_cache = scene->global_cache;

	ITMHashSwapState* swap_states = global_cache->GetSwapStates(false);

	HashEntry* hash_table = scene->index.GetEntries();
	HashBlockVisibility* block_visibility_types = scene->index.GetBlockVisibilityTypes();

	TVoxel *syncedVoxelBlocks_local = global_cache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_local = global_cache->GetHasSyncedData(false);
	int *neededEntryIDs_local = global_cache->GetNeededEntryIDs(false);

	TVoxel *syncedVoxelBlocks_global = global_cache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_global = global_cache->GetHasSyncedData(false);
	int *neededEntryIDs_global = global_cache->GetNeededEntryIDs(false);

	TVoxel* voxels = scene->GetVoxelBlocks();
	int* block_allocation_list = scene->index.GetBlockAllocationList();

	const int hash_entry_count = global_cache->GetHashEntryCount();
	
	int needed_entry_count = 0; // needed for what?
	int allocated_block_count = scene->index.GetLastFreeBlockListId();

	for (int destination_hash_code = 0; destination_hash_code < hash_entry_count; destination_hash_code++)
	{
		if (needed_entry_count >= SWAP_OPERATION_BLOCK_COUNT) break;

		int localPtr = hash_table[destination_hash_code].ptr;
		ITMHashSwapState &swapState = swap_states[destination_hash_code];

		if (swapState.state == 2 && localPtr >= 0 && block_visibility_types[destination_hash_code] == 0)
		{
			TVoxel *localVBALocation = voxels + localPtr * VOXEL_BLOCK_SIZE3;

			neededEntryIDs_local[needed_entry_count] = destination_hash_code;

			hasSyncedData_local[needed_entry_count] = true;
			memcpy(syncedVoxelBlocks_local + needed_entry_count * VOXEL_BLOCK_SIZE3, localVBALocation, VOXEL_BLOCK_SIZE3 * sizeof(TVoxel));

			swap_states[destination_hash_code].state = 0;

			int vbaIdx = allocated_block_count;
			if (vbaIdx < ORDERED_LIST_SIZE - 1)
			{
				allocated_block_count++;
				block_allocation_list[vbaIdx + 1] = localPtr;
				hash_table[destination_hash_code].ptr = -1;

				for (int i = 0; i < VOXEL_BLOCK_SIZE3; i++) localVBALocation[i] = TVoxel();
			}

			needed_entry_count++;
		}
	}

	scene->index.SetLastFreeBlockListId(allocated_block_count);

	// would copy neededEntryIDs_local, hasSyncedData_local and syncedVoxelBlocks_local into *_global here

	if (needed_entry_count > 0)
	{
		for (int entryId = 0; entryId < needed_entry_count; entryId++)
		{
			if (hasSyncedData_global[entryId])
				global_cache->SetStoredData(neededEntryIDs_global[entryId], syncedVoxelBlocks_global + entryId * VOXEL_BLOCK_SIZE3);
		}
	}
}

template<class TVoxel>
void SwappingEngine_CPU<TVoxel, VoxelBlockHash>::CleanLocalMemory(VoxelVolume<TVoxel, VoxelBlockHash> *scene, RenderState *renderState)
{
	HashEntry *hashTable = scene->index.GetEntries();
	HashBlockVisibility *blockVisibilityTypes = scene->index.GetBlockVisibilityTypes();

	TVoxel *localVBA = scene->GetVoxelBlocks();
	int *voxelAllocationList = scene->index.GetBlockAllocationList();

	int noTotalEntries = scene->index.hash_entry_count;

	int noNeededEntries = 0;
	int noAllocatedVoxelEntries = scene->index.GetLastFreeBlockListId();

	for (int entryDestId = 0; entryDestId < noTotalEntries; entryDestId++)
	{
		if (noNeededEntries >= SWAP_OPERATION_BLOCK_COUNT) break;

		int localPtr = hashTable[entryDestId].ptr;

		if (localPtr >= 0 && blockVisibilityTypes[entryDestId] == 0)
		{
			TVoxel *localVBALocation = localVBA + localPtr * VOXEL_BLOCK_SIZE3;

			int vbaIdx = noAllocatedVoxelEntries;
			if (vbaIdx < ORDERED_LIST_SIZE - 1)
			{
				noAllocatedVoxelEntries++;
				voxelAllocationList[vbaIdx + 1] = localPtr;
				hashTable[entryDestId].ptr = -1;

				for (int i = 0; i < VOXEL_BLOCK_SIZE3; i++) localVBALocation[i] = TVoxel();
			}

			noNeededEntries++;
		}
	}

	scene->index.SetLastFreeBlockListId(noAllocatedVoxelEntries);
}