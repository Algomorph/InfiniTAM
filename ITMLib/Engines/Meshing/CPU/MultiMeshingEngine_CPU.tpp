// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "MultiMeshingEngine_CPU.h"

#include "../Shared/MultiMeshingEngine_Shared.h"

using namespace ITMLib;

template<class TVoxel>
inline void MultiMeshingEngine_CPU<TVoxel, VoxelBlockHash>::MeshScene(Mesh * mesh, const MultiSceneManager & sceneManager)
{
	int numLocalMaps = (int)sceneManager.numLocalMaps();
	if (numLocalMaps > MAX_NUM_LOCALMAPS) numLocalMaps = MAX_NUM_LOCALMAPS;

	MultiIndexData hashTables;
	MultiVoxelData localVBAs;

	const VoxelVolumeParameters & sceneParams = *(sceneManager.getLocalMap(0)->scene->parameters);
	hashTables.numLocalMaps = numLocalMaps;
	for (int localMapId = 0; localMapId < numLocalMaps; ++localMapId)
	{
		hashTables.poses_vs[localMapId] = sceneManager.getEstimatedGlobalPose(localMapId).GetM();
		hashTables.poses_vs[localMapId].m30 /= sceneParams.voxel_size;
		hashTables.poses_vs[localMapId].m31 /= sceneParams.voxel_size;
		hashTables.poses_vs[localMapId].m32 /= sceneParams.voxel_size;
		
		hashTables.posesInv[localMapId] = sceneManager.getEstimatedGlobalPose(localMapId).GetInvM();
		hashTables.posesInv[localMapId].m30 /= sceneParams.voxel_size;
		hashTables.posesInv[localMapId].m31 /= sceneParams.voxel_size;
		hashTables.posesInv[localMapId].m32 /= sceneParams.voxel_size;

		hashTables.index[localMapId] = sceneManager.getLocalMap(localMapId)->scene->index.GetIndexData();
		localVBAs.voxels[localMapId] = sceneManager.getLocalMap(localMapId)->scene->GetVoxelBlocks();
	}

	Mesh::Triangle *triangles = mesh->triangles.GetData(MEMORYDEVICE_CPU);
	mesh->triangles.Clear();

	int triangle_count = 0, max_triangle_count = mesh->max_triangle_count;
	int noTotalEntriesPerLocalMap = sceneManager.getLocalMap(0)->scene->index.hash_entry_count;
	float factor = sceneParams.voxel_size;

	// very dumb rendering -- likely to generate lots of duplicates
	for (int localMapId = 0; localMapId < numLocalMaps; ++localMapId)
	{
		HashEntry *hashTable = hashTables.index[localMapId];

		for (int entryId = 0; entryId < noTotalEntriesPerLocalMap; entryId++)
		{
			Vector3i globalPos;
			const HashEntry &currentHashEntry = hashTable[entryId];

			if (currentHashEntry.ptr < 0) continue;

			globalPos = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) for (int x = 0; x < VOXEL_BLOCK_SIZE; x++)
			{
				Vector3f vertList[12];
				int cubeIndex = buildVertListMulti(vertList, globalPos, Vector3i(x, y, z), &localVBAs, &hashTables, localMapId);

				if (cubeIndex < 0) continue;

				for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3)
				{
					triangles[triangle_count].p0 = vertList[triangleTable[cubeIndex][i]] * factor;
					triangles[triangle_count].p1 = vertList[triangleTable[cubeIndex][i + 1]] * factor;
					triangles[triangle_count].p2 = vertList[triangleTable[cubeIndex][i + 2]] * factor;

					if (triangle_count < max_triangle_count - 1) triangle_count++;
				}
			}
		}
	}

	mesh->triangle_count = triangle_count;
}