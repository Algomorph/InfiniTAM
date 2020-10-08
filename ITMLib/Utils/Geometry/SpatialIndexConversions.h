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
#include "../../Objects/Volume/RepresentationAccess.h"

_CPU_AND_GPU_CODE_
inline static int
ComputeLinearIndexFromPosition_PlainVoxelArray(const ITMLib::PlainVoxelArray::IndexData* data, const Vector3i& position) {
	Vector3i position_in_3d_array = position - data->offset;
	return position_in_3d_array.z * (data->size.y * data->size.x)
	       + position_in_3d_array.y * data->size.x + position_in_3d_array.x;
};

_CPU_AND_GPU_CODE_
inline static int
ComputeLinearIndexFromPosition_VoxelBlockHash(const ITMLib::VoxelBlockHash::IndexData* data, const Vector3i& position) {
	Vector3i block_position;
	pointToVoxelBlockPos(position, block_position);
	Vector3i position_in_voxel_hash_block(position.x - block_position.x * VOXEL_BLOCK_SIZE,
	                                      position.y - block_position.y * VOXEL_BLOCK_SIZE,
	                                      position.z - block_position.z * VOXEL_BLOCK_SIZE);
	return position_in_voxel_hash_block.z * (VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE)
	       + position_in_voxel_hash_block.y * VOXEL_BLOCK_SIZE + position_in_voxel_hash_block.x;
}

_CPU_AND_GPU_CODE_
inline static void
ComputePositionFromLinearIndex_PlainVoxelArray(int& x, int& y, int& z,
                                               const ITMLib::PlainVoxelArray::IndexData* array_parameters,
                                               int linear_index_in_array) {

	z = linear_index_in_array / (array_parameters->size.x * array_parameters->size.y);
	int tmp = linear_index_in_array - z * array_parameters->size.x * array_parameters->size.y;
	y = tmp / array_parameters->size.x;
	x = tmp - y * array_parameters->size.x;
	x += array_parameters->offset.x;
	y += array_parameters->offset.y;
	z += array_parameters->offset.z;
}

_CPU_AND_GPU_CODE_
inline static Vector3i
ComputePositionVectorFromLinearIndex_PlainVoxelArray(const ITMLib::PlainVoxelArray::IndexData* array_parameters,
                                                     int linear_index_in_array) {
	int z = linear_index_in_array / (array_parameters->size.x * array_parameters->size.y);
	int tmp = linear_index_in_array - z * array_parameters->size.x * array_parameters->size.y;
	int y = tmp / array_parameters->size.x;
	int x = tmp - y * array_parameters->size.x;
	return {x + array_parameters->offset.x, y + array_parameters->offset.y, z + array_parameters->offset.z};
}

_CPU_AND_GPU_CODE_
inline static Vector3i
ComputePositionVectorFromLinearIndex_VoxelBlockHash(Vector3s block_position_in_blocks,
                                                    int linear_index_in_block) {
	int z = linear_index_in_block / (VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE);
	int tmp = linear_index_in_block - z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	int y = tmp / VOXEL_BLOCK_SIZE;
	int x = tmp - y * VOXEL_BLOCK_SIZE;
	return {x + block_position_in_blocks.x * VOXEL_BLOCK_SIZE,
	        y + block_position_in_blocks.y * VOXEL_BLOCK_SIZE,
	        z + block_position_in_blocks.z * VOXEL_BLOCK_SIZE};
}

