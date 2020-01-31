//  ================================================================
//  Created by Gregory Kramida on 10/17/19.
//  Copyright (c) 2019 Gregory Kramida
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


#include "../../../ORUtils/PlatformIndependence.h"
#include "../../Objects/Volume/PlainVoxelArray.h"

_CPU_AND_GPU_CODE_
inline static int
ComputeLinearIndexFromPosition_PlainVoxelArray(const ITMLib::PlainVoxelArray::IndexData* data, const Vector3i& position) {
	Vector3i positionIn3DArray = position - data->offset;
	return positionIn3DArray.z * (data->size.y * data->size.x)
	       + positionIn3DArray.y * data->size.x + positionIn3DArray.x;
};

_CPU_AND_GPU_CODE_
inline static void
ComputePositionFromLinearIndex_PlainVoxelArray(int& x, int& y, int& z,
                                               const ITMLib::PlainVoxelArray::IndexData* indexData,
                                               int linearIndex) {

	z = linearIndex / (indexData->size.x * indexData->size.y);
	int tmp = linearIndex - z * indexData->size.x * indexData->size.y;
	y = tmp / indexData->size.x;
	x = tmp - y * indexData->size.x;
	x += indexData->offset.x;
	y += indexData->offset.y;
	z += indexData->offset.z;
}

_CPU_AND_GPU_CODE_
inline static Vector3i
ComputePositionVectorFromLinearIndex_PlainVoxelArray(const ITMLib::PlainVoxelArray::IndexData* indexData,
                                                     int linearIndex) {
	int z = linearIndex / (indexData->size.x * indexData->size.y);
	int tmp = linearIndex - z * indexData->size.x * indexData->size.y;
	int y = tmp / indexData->size.x;
	int x = tmp - y * indexData->size.x;
	return {x + indexData->offset.x, y + indexData->offset.y, z + indexData->offset.z};
}

_CPU_AND_GPU_CODE_
inline static Vector3i
ComputePositionVectorFromLinearIndex_VoxelBlockHash( Vector3s blockPosition_Blocks,
                                                     int linearIndex) {
	int z = linearIndex / (VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE);
	int tmp = linearIndex - z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	int y = tmp / VOXEL_BLOCK_SIZE;
	int x = tmp - y * VOXEL_BLOCK_SIZE;
	return {x + blockPosition_Blocks.x * VOXEL_BLOCK_SIZE,
		 y + blockPosition_Blocks.y * VOXEL_BLOCK_SIZE,
		 z + blockPosition_Blocks.z * VOXEL_BLOCK_SIZE};
}

