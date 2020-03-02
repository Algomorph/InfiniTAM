//  ================================================================
//  Created by Gregory Kramida on 9/27/19.
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
#include "../../../ORUtils/MathUtils.h"
#include "../Math.h"
#include "GeometryConversionOperations.h"

_CPU_AND_GPU_CODE_
inline
bool IsPointInBounds(const Vector3i& point, const ITMLib::GridAlignedBox& box) {
	return point.x >= box.offset.x &&
	       point.y >= box.offset.y &&
	       point.z >= box.offset.z &&
	       point.x < box.offset.x + box.size.x &&
	       point.y < box.offset.y + box.size.y &&
	       point.z < box.offset.z + box.size.z;
}

_CPU_AND_GPU_CODE_
inline
bool IsPointInBounds(const Vector3i& point, const Extent3Di& bounds) {
	return point.x >= bounds.min_x &&
	       point.y >= bounds.min_y &&
	       point.z >= bounds.min_z &&
	       point.x < bounds.max_x &&
	       point.y < bounds.max_y &&
	       point.z < bounds.max_z;
}


_CPU_AND_GPU_CODE_
inline
bool IsHashBlockFullyInBounds(const Vector3i& hashBlockPositionVoxels, const Extent3Di& bounds) {
	return hashBlockPositionVoxels.x + VOXEL_BLOCK_SIZE - 1 <= bounds.max_x &&
	       hashBlockPositionVoxels.x >= bounds.min_x &&
	       hashBlockPositionVoxels.y + VOXEL_BLOCK_SIZE - 1 <= bounds.max_y &&
	       hashBlockPositionVoxels.y >= bounds.min_y &&
	       hashBlockPositionVoxels.z + VOXEL_BLOCK_SIZE - 1 <= bounds.max_z &&
	       hashBlockPositionVoxels.z >= bounds.min_z;
}

_CPU_AND_GPU_CODE_
inline
bool IsHashBlockPartiallyInBounds(const Vector3i& hashBlockPositionVoxels, const Extent3Di& bounds) {
	//@formatter:off
	return ((hashBlockPositionVoxels.x + VOXEL_BLOCK_SIZE - 1 >= bounds.max_x && hashBlockPositionVoxels.x <= bounds.max_x)
	        || (hashBlockPositionVoxels.x + VOXEL_BLOCK_SIZE - 1 >= bounds.min_x && hashBlockPositionVoxels.x <= bounds.min_x)) &&
	       ((hashBlockPositionVoxels.y + VOXEL_BLOCK_SIZE - 1 >= bounds.max_y && hashBlockPositionVoxels.y <= bounds.max_y)
	        || (hashBlockPositionVoxels.y + VOXEL_BLOCK_SIZE - 1 >= bounds.min_y && hashBlockPositionVoxels.y <= bounds.min_y)) &&
	       ((hashBlockPositionVoxels.z + VOXEL_BLOCK_SIZE - 1 >= bounds.max_z && hashBlockPositionVoxels.z <= bounds.max_z)
	        || (hashBlockPositionVoxels.z + VOXEL_BLOCK_SIZE - 1 >= bounds.min_z && hashBlockPositionVoxels.z <= bounds.min_z));
	//@formatter:on
}

_CPU_AND_GPU_CODE_
inline
Extent3Di UnionExtent(const Extent3Di& extent1, const Extent3Di& extent2) {
	return {ORUTILS_MIN(extent1.min_x, extent2.min_x),
	        ORUTILS_MIN(extent1.min_y, extent2.min_y),
	        ORUTILS_MIN(extent1.min_z, extent2.min_z),
	        ORUTILS_MAX(extent1.max_x, extent2.max_x),
	        ORUTILS_MAX(extent1.max_y, extent2.max_y),
	        ORUTILS_MAX(extent1.max_z, extent2.max_z)};
}

_CPU_AND_GPU_CODE_
inline
Extent3Di UnionExtent(const ITMLib::GridAlignedBox& extent1,
                      const ITMLib::GridAlignedBox& extent2) {
	return UnionExtent(PVA_InfoToExtent(extent1), PVA_InfoToExtent(extent2));
}

_CPU_AND_GPU_CODE_
inline
Extent3Di IntersectionExtent(const Extent3Di& extent1, const Extent3Di& extent2) {
	Extent3Di extent(ORUTILS_MAX(extent1.min_x, extent2.min_x),
	                 ORUTILS_MAX(extent1.min_y, extent2.min_y),
	                 ORUTILS_MAX(extent1.min_z, extent2.min_z),
	                 ORUTILS_MIN(extent1.max_x, extent2.max_x),
	                 ORUTILS_MIN(extent1.max_y, extent2.max_y),
	                 ORUTILS_MIN(extent1.max_z, extent2.max_z));
	if (extent.max_x - extent.min_x < 0 ||
	    extent.max_y - extent.min_y < 0 ||
	    extent.max_z - extent.min_z < 0) {
		return Extent3Di();
	}
	return extent;
}

_CPU_AND_GPU_CODE_
inline
Extent3Di IntersectionExtent(const ITMLib::GridAlignedBox& extent1,
                             const ITMLib::GridAlignedBox& extent2) {
	return IntersectionExtent(PVA_InfoToExtent(extent1), PVA_InfoToExtent(extent2));
}

