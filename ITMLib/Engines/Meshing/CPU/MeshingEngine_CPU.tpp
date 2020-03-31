// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "MeshingEngine_CPU.h"
#include "../Shared/MeshingEngine_Shared.h"

using namespace ITMLib;

template<class TVoxel>
void MeshingEngine_CPU<TVoxel, VoxelBlockHash>::MeshScene(Mesh *mesh, const VoxelVolume<TVoxel, VoxelBlockHash> *scene)
{
	Mesh::Triangle *triangles = mesh->triangles->GetData(MEMORYDEVICE_CPU);
	const TVoxel *localVBA = scene->voxels.GetVoxelBlocks();
	const HashEntry *hashTable = scene->index.GetEntries();

	int noTriangles = 0, noMaxTriangles = mesh->noMaxTriangles, noTotalEntries = scene->index.hash_entry_count;
	float factor = scene->parameters->voxel_size;

	mesh->triangles->Clear();

	for (int entryId = 0; entryId < noTotalEntries; entryId++)
	{
		Vector3i hashBlockCornerPosition;
		const HashEntry &currentHashEntry = hashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		//position of the voxel at the current hash block corner with minimum coordinates
		hashBlockCornerPosition = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

		for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) for (int x = 0; x < VOXEL_BLOCK_SIZE; x++)
		{
			Vector3f vertexList[12];
			// build vertices by interpolating edges of cubes that intersect with the isosurface
			// based on positive/negative SDF values
			int cubeIndex = buildVertexList(vertexList, hashBlockCornerPosition, Vector3i(x, y, z), localVBA, hashTable);

			// cube does not intersect with the isosurface
			if (cubeIndex < 0) continue;

			for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3)
			{
				// cube index also tells us how the vertices are grouped into triangles,
				// use it to look up the vertex indices composing each triangle from the vertex list
				triangles[noTriangles].p0 = vertexList[triangleTable[cubeIndex][i]] * factor;
				triangles[noTriangles].p1 = vertexList[triangleTable[cubeIndex][i + 1]] * factor;
				triangles[noTriangles].p2 = vertexList[triangleTable[cubeIndex][i + 2]] * factor;

				if (noTriangles < noMaxTriangles - 1) noTriangles++;
			}
		}
	}

	mesh->noTotalTriangles = noTriangles;
}