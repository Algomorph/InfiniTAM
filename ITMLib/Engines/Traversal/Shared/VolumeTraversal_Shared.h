//  ================================================================
//  Created by Gregory Kramida on 8/29/19.
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

//local
#include "../../../Objects/Volume/PlainVoxelArray.h"
#include "../../../Objects/Volume/RepresentationAccess.h"

namespace ITMLib{

// region ======================================= AUXILIARY FUNCTIONS (VOXEL HASH BLOCKS) ==============================
_CPU_AND_GPU_CODE_
inline bool HashBlockDoesNotIntersectBounds(const Vector3i& hash_entry_min_point, const Vector3i& hash_entry_max_point,
                                            const Vector6i& bounds) {
	return hash_entry_max_point.x < bounds.min_x ||
	       hash_entry_max_point.y < bounds.min_y ||
	       hash_entry_max_point.z < bounds.min_z ||
	       hash_entry_min_point.x >= bounds.max_x ||
	       hash_entry_min_point.y >= bounds.max_y ||
	       hash_entry_min_point.z >= bounds.max_z;
}

_CPU_AND_GPU_CODE_
inline
Vector6i ComputeLocalBounds(const Vector3i& hash_entry_min_point, const Vector3i& hash_entry_max_point,
                            const Vector6i& bounds) {
	return Vector6i(ORUTILS_MAX(0, bounds.min_x - hash_entry_min_point.x),
	                ORUTILS_MAX(0, bounds.min_y - hash_entry_min_point.y),
	                ORUTILS_MAX(0, bounds.min_z - hash_entry_min_point.z),
	                ORUTILS_MIN(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE - (hash_entry_max_point.x - bounds.max_x)),
	                ORUTILS_MIN(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE - (hash_entry_max_point.y - bounds.max_y)),
	                ORUTILS_MIN(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE - (hash_entry_max_point.z - bounds.max_z)));
}


// endregion ===========================================================================================================


template<typename TVoxelPrimary, typename TVoxelSecondary, typename TFunctor>
struct FlipArgumentBooleanFunctor {
	FlipArgumentBooleanFunctor(TFunctor functor) : functor(functor) {}
	_CPU_AND_GPU_CODE_
	bool operator()(TVoxelPrimary& voxelPrimary, TVoxelSecondary& voxelSecondary) {
		return functor(voxelSecondary, voxelPrimary);
	}

	TFunctor functor;
};


template<typename TVoxel, typename TPredicateFunctor>
inline static bool
VoxelBlockSatisfiesPredicate(TVoxel* block_voxels,
                             TPredicateFunctor& one_voxel_predicate_functor) {
	for (int linear_index_in_block = 0; linear_index_in_block < VOXEL_BLOCK_SIZE3; linear_index_in_block++) {
		TVoxel& voxel = block_voxels[linear_index_in_block];
		if (!one_voxel_predicate_functor(voxel)) {
			return false;
		}
	}
	return true;
}

} // namespace ITMLib