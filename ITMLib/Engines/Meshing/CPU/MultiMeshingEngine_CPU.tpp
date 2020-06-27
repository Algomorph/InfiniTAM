// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "MultiMeshingEngine_CPU.h"

#include "../Shared/MultiMeshingEngine_Shared.h"

using namespace ITMLib;

template<class TVoxel>
inline Mesh MultiMeshingEngine_CPU<TVoxel, VoxelBlockHash>::MeshVolume(const MultiSceneManager & manager)
{
	int numLocalMaps = (int)manager.numLocalMaps();
	if (numLocalMaps > MAX_NUM_LOCALMAPS) numLocalMaps = MAX_NUM_LOCALMAPS;

	MultiIndexData hashTables;
	MultiVoxelData localVBAs;

	const VoxelVolumeParameters& sceneParams = manager.getLocalMap(0)->volume->GetParameters();
	hashTables.numLocalMaps = numLocalMaps;
	for (int localMapId = 0; localMapId < numLocalMaps; ++localMapId)
	{
		hashTables.poses_vs[localMapId] = manager.getEstimatedGlobalPose(localMapId).GetM();
		hashTables.poses_vs[localMapId].m30 /= sceneParams.voxel_size;
		hashTables.poses_vs[localMapId].m31 /= sceneParams.voxel_size;
		hashTables.poses_vs[localMapId].m32 /= sceneParams.voxel_size;
		
		hashTables.posesInv[localMapId] = manager.getEstimatedGlobalPose(localMapId).GetInvM();
		hashTables.posesInv[localMapId].m30 /= sceneParams.voxel_size;
		hashTables.posesInv[localMapId].m31 /= sceneParams.voxel_size;
		hashTables.posesInv[localMapId].m32 /= sceneParams.voxel_size;

		hashTables.index[localMapId] = manager.getLocalMap(localMapId)->volume->index.GetIndexData();
		localVBAs.voxels[localMapId] = manager.getLocalMap(localMapId)->volume->GetVoxels();
	}
	const unsigned int max_triangle_count = manager.getLocalMap(0)->volume->index.GetMaxVoxelCount();
	ORUtils::MemoryBlock<Mesh::Triangle> triangles(max_triangle_count, MEMORYDEVICE_CUDA);
	Mesh::Triangle *triangles_device = triangles.GetData(MEMORYDEVICE_CPU);

	unsigned int triangle_count = 0;
	int noTotalEntriesPerLocalMap = manager.getLocalMap(0)->volume->index.hash_entry_count;
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

				for (int i = 0; triangle_table[cubeIndex][i] != -1; i += 3)
				{
					triangles_device[triangle_count].p0 = vertList[triangle_table[cubeIndex][i]] * factor;
					triangles_device[triangle_count].p1 = vertList[triangle_table[cubeIndex][i + 1]] * factor;
					triangles_device[triangle_count].p2 = vertList[triangle_table[cubeIndex][i + 2]] * factor;

					if (triangle_count < max_triangle_count - 1) triangle_count++;
				}
			}
		}
	}

	return Mesh(triangles, triangle_count);
}