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

	int hashEntryCount = globalCache->hashEntryCount;

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
	GlobalCache<TVoxel, VoxelBlockHash> *globalCache = scene->global_cache;

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);

	HashEntry *hashTable = scene->index.GetEntries();
	HashBlockVisibility *blockVisibilityTypes = scene->index.GetBlockVisibilityTypes();

	TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_local = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

	TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);

	TVoxel *localVBA = scene->GetVoxelBlocks();
	int *voxelAllocationList = scene->index.GetBlockAllocationList();

	int hashEntryCount = globalCache->hashEntryCount;
	
	int neededEntryCount = 0; // needed for what?
	int allocatedEntryCount = scene->index.GetLastFreeBlockListId();

	for (int entryDestId = 0; entryDestId < hashEntryCount; entryDestId++)
	{
		if (neededEntryCount >= SWAP_OPERATION_BLOCK_COUNT) break;

		int localPtr = hashTable[entryDestId].ptr;
		ITMHashSwapState &swapState = swapStates[entryDestId];

		if (swapState.state == 2 && localPtr >= 0 && blockVisibilityTypes[entryDestId] == 0)
		{
			TVoxel *localVBALocation = localVBA + localPtr * VOXEL_BLOCK_SIZE3;

			neededEntryIDs_local[neededEntryCount] = entryDestId;

			hasSyncedData_local[neededEntryCount] = true;
			memcpy(syncedVoxelBlocks_local + neededEntryCount * VOXEL_BLOCK_SIZE3, localVBALocation, VOXEL_BLOCK_SIZE3 * sizeof(TVoxel));

			swapStates[entryDestId].state = 0;

			int vbaIdx = allocatedEntryCount;
			if (vbaIdx < ORDERED_LIST_SIZE - 1)
			{
				allocatedEntryCount++;
				voxelAllocationList[vbaIdx + 1] = localPtr;
				hashTable[entryDestId].ptr = -1;

				for (int i = 0; i < VOXEL_BLOCK_SIZE3; i++) localVBALocation[i] = TVoxel();
			}

			neededEntryCount++;
		}
	}

	scene->index.SetLastFreeBlockListId(allocatedEntryCount);

	// would copy neededEntryIDs_local, hasSyncedData_local and syncedVoxelBlocks_local into *_global here

	if (neededEntryCount > 0)
	{
		for (int entryId = 0; entryId < neededEntryCount; entryId++)
		{
			if (hasSyncedData_global[entryId])
				globalCache->SetStoredData(neededEntryIDs_global[entryId], syncedVoxelBlocks_global + entryId * VOXEL_BLOCK_SIZE3);
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