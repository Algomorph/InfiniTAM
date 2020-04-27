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

#pragma once

namespace { // Anonymous namespace (CUDA Kernels)

//triangles_device, triangle_count.GetData(MEMORYDEVICE_CUDA), voxel_size,
//		utilized_block_count, voxels, hash_table, volume->GetUtilizedBlockHashCodes()

template<class TVoxel>
__global__ void
meshScene_device(ITMLib::Mesh::Triangle* triangles, unsigned int* triangle_count, const float voxel_size,
                 const int utilized_block_count, const TVoxel* voxels, const ITMLib::HashEntry* hash_table,
                 const int* utilized_block_hash_codes) {
	int block_idx = blockIdx.x + gridDim.x * blockIdx.y;
	if (block_idx > utilized_block_count) return;

	const ITMLib::HashEntry& hash_entry = hash_table[utilized_block_hash_codes[block_idx]];

	//position of the voxel at the current hash block corner with minimum coordinates
	Vector3i block_corner_position = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;

	Vector3f vertex_list[12];
	int cube_index = buildVertexList(vertex_list, block_corner_position, Vector3i(threadIdx.x, threadIdx.y, threadIdx.z),
	                                 voxels, hash_table);

	// cube does not intersect with the isosurface
	if (cube_index < 0) return;

	for (int i_vertex = 0; triangle_table[cube_index][i_vertex] != -1; i_vertex += 3) {
		// cube index also tells us how the vertices are grouped into triangles,
		// use it to look up the vertex indices composing each triangle from the vertex list
		int triangle_index = atomicAdd(triangle_count, 1);
		triangles[triangle_index].p0 = vertex_list[triangle_table[cube_index][i_vertex]] * voxel_size;
		triangles[triangle_index].p1 = vertex_list[triangle_table[cube_index][i_vertex + 1]] * voxel_size;
		triangles[triangle_index].p2 = vertex_list[triangle_table[cube_index][i_vertex + 2]] * voxel_size;
	}
}

}// end anonymous namespace
