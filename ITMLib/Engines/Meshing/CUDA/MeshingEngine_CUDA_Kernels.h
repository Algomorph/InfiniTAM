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

template<class TVoxel>
__global__ void
meshScene_device(ITMLib::Mesh::Triangle* triangles, unsigned int* noTriangles_device, float factor, int noTotalEntries,
                 int noMaxTriangles, const Vector4s* visibleBlockGlobalPos, const TVoxel* localVBA,
                 const ITMLib::HashEntry* hashTable) {
	const Vector4s globalPos_4s = visibleBlockGlobalPos[blockIdx.x + gridDim.x * blockIdx.y];

	if (globalPos_4s.w == 0) return;

	Vector3i globalPos = Vector3i(globalPos_4s.x, globalPos_4s.y, globalPos_4s.z) * VOXEL_BLOCK_SIZE;

	Vector3f vertList[12];
	int cubeIndex = buildVertexList(vertList, globalPos, Vector3i(threadIdx.x, threadIdx.y, threadIdx.z), localVBA,
	                                hashTable);

	if (cubeIndex < 0) return;

	for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3) {
		int triangleId = atomicAdd(noTriangles_device, 1);

		if (triangleId < noMaxTriangles - 1) {
			triangles[triangleId].p0 = vertList[triangleTable[cubeIndex][i]] * factor;
			triangles[triangleId].p1 = vertList[triangleTable[cubeIndex][i + 1]] * factor;
			triangles[triangleId].p2 = vertList[triangleTable[cubeIndex][i + 2]] * factor;
		}
	}
}

}// end anonymous namespace
