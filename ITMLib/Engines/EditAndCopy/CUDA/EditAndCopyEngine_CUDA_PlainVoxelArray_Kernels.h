//  ================================================================
//  Created by Gregory Kramida on 7/24/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "EditAndCopyEngine_CUDA.h"
#include "../Shared/EditAndCopyEngine_Shared.h"
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../Utils/CUDAUtils.h"
#include "../../../Objects/Volume/RepresentationAccess.h"

#include <cstring>
#include <iostream>

using namespace ITMLib;

namespace {

// device functions

template<class TVoxel>
__global__ void setVoxel_device(TVoxel* voxelArray, const PlainVoxelArray::GridAlignedBox* arrayInfo,
                                const Vector3i location, TVoxel value, bool* success) {
	int vmIndex;
	int linearIndex = findVoxel(arrayInfo, location, vmIndex);

	if (linearIndex > -1) {
		voxelArray[linearIndex] = value;
		*success = true;
	} else {
		*success = false;
	}
}

template<class TVoxel>
__global__ void readVoxel_device(TVoxel* voxelArray, const PlainVoxelArray::GridAlignedBox* arrayInfo,
                                 const Vector3i at, ReadVoxelResult<TVoxel>* result) {
	int vmIndex = 0;
	int arrayIndex = findVoxel(arrayInfo, at, vmIndex);
	if (arrayIndex < 0) {
		result->found = false;
	} else {
		result->found = true;
		result->voxel = voxelArray[arrayIndex];
	}
}

__global__ void isPointInBounds_device(
		const PlainVoxelArray::GridAlignedBox* index_bounds,
		const Vector3i at, bool* answer) {
	Vector3i point2 = at - index_bounds->offset;
	*answer = !((point2.x < 0) || (point2.x >= index_bounds->size.x) ||
	            (point2.y < 0) || (point2.y >= index_bounds->size.y) ||
	            (point2.z < 0) || (point2.z >= index_bounds->size.z));

}

template<class TVoxel>
__global__ void directCopy_device(TVoxel* destinationArray, const TVoxel* sourceArray,
                                  const PlainVoxelArray::GridAlignedBox* arrayInfo) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;
	int locId;
	locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;
	destinationArray[locId] = sourceArray[locId];
}

template<class TVoxel>
__global__ void offsetCopy_device(TVoxel* destinationArray, const TVoxel* sourceArray,
                                  const PlainVoxelArray::GridAlignedBox* destinationIndexData,
                                  const PlainVoxelArray::GridAlignedBox* sourceIndexData,
                                  const Vector3i offset, const Vector3i minPointSourceSansOffset,
                                  const Vector3i extent) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;
	if(x >= extent.x || y >= extent.y || z >= extent.z ) return;
	x += minPointSourceSansOffset.x;
	y += minPointSourceSansOffset.y;
	z += minPointSourceSansOffset.z;
	int linearSourceIndex = x + y * sourceIndexData->size.x + z * sourceIndexData->size.x * sourceIndexData->size.y;
	int destination_z, destination_y, destination_x;
	destination_z = z + sourceIndexData->offset.z - destinationIndexData->offset.z + offset.z;
	destination_y = y + sourceIndexData->offset.y - destinationIndexData->offset.y + offset.y;
	destination_x = x + sourceIndexData->offset.x - destinationIndexData->offset.x + offset.x;

	int linearDestinationIndex = destination_x + destination_y * destinationIndexData->size.x +
	                             destination_z * destinationIndexData->size.x * destinationIndexData->size.y;
	destinationArray[linearDestinationIndex] = sourceArray[linearSourceIndex];
}

} // anonymous namespace (device functions)