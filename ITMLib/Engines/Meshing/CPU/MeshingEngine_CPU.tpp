//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/27/20.
//  Copyright (c) 2020 Gregory Kramida
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

#include "MeshingEngine_CPU.h"
#include "../Shared/MeshingEngine_Shared.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"

using namespace ITMLib;

template<class TVoxel>
Mesh MeshingEngine_CPU<TVoxel, VoxelBlockHash>::MeshVolume(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
	bool temporary_volume_used = false;

	const VoxelVolume<TVoxel, VoxelBlockHash>* meshing_target_volume;
	VoxelVolume<TVoxel, VoxelBlockHash>* temporary_volume;

	if (volume->GetMemoryType() == MEMORYDEVICE_CUDA) {
		temporary_volume_used = true;
		temporary_volume = new VoxelVolume<TVoxel, VoxelBlockHash>(*volume, MEMORYDEVICE_CPU);
		meshing_target_volume = temporary_volume;
	} else {
		meshing_target_volume = volume;
	}

	const int utilized_block_count = meshing_target_volume->index.GetUtilizedBlockCount();
	const int max_triangle_count = utilized_block_count * VOXEL_BLOCK_SIZE3;
	const float voxel_size = meshing_target_volume->GetParameters().voxel_size;

	ORUtils::MemoryBlock<Mesh::Triangle> triangles(max_triangle_count, MEMORYDEVICE_CPU);
	Mesh::Triangle* triangles_device = triangles.GetData(MEMORYDEVICE_CPU);
	const TVoxel* voxels = meshing_target_volume->GetVoxels();
	const HashEntry* hash_table = meshing_target_volume->index.GetEntries();

	const int* utilized_block_indices = meshing_target_volume->index.GetUtilizedBlockHashCodes();

	std::atomic<unsigned int> triangle_count(0);


#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(hash_table, voxels, utilized_block_indices, triangle_count, triangle_table, triangles_device)
#endif
	for (int utilized_entry_index = 0; utilized_entry_index < utilized_block_count; utilized_entry_index++) {
		const HashEntry& hash_entry = hash_table[utilized_block_indices[utilized_entry_index]];

		//position of the voxel at the current hash block corner with minimum coordinates
		Vector3i block_corner_position = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;

		for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
			for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
				for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
					Vector3f vertex_list[12];
					// build vertices by interpolating edges of cubes that intersect with the isosurface
					// based on positive/negative SDF values
					int cube_index = buildVertexList(vertex_list, block_corner_position, Vector3i(x, y, z), voxels,
					                                 hash_table);

					// cube does not intersect with the isosurface
					if (cube_index < 0) continue;

					for (int i_vertex = 0; triangle_table[cube_index][i_vertex] != -1; i_vertex += 3) {
						unsigned int current_triangle_count = atomicAdd_CPU(triangle_count, 1u);
						// cube index also tells us how the vertices are grouped into triangles,
						// use it to look up the vertex indices composing each triangle from the vertex list
						triangles_device[current_triangle_count].p0 =
								vertex_list[triangle_table[cube_index][i_vertex]] * voxel_size;
						triangles_device[current_triangle_count].p1 =
								vertex_list[triangle_table[cube_index][i_vertex + 1]] * voxel_size;
						triangles_device[current_triangle_count].p2 =
								vertex_list[triangle_table[cube_index][i_vertex + 2]] * voxel_size;
					}
				}
			}
		}
	}

	if (temporary_volume_used) {
		delete temporary_volume;
	}
	return Mesh(triangles, triangle_count.load());
}