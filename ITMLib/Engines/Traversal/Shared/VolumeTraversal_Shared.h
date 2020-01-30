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
#include "../../../Objects/Scene/PlainVoxelArray.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"


// region ======================================= AUXILIARY FUNCTIONS (VOXEL HASH BLOCKS) ==============================
_CPU_AND_GPU_CODE_
inline bool HashBlockDoesNotIntersectBounds(const Vector3i& hashEntryMinPoint, const Vector3i& hashEntryMaxPoint,
                                            const Vector6i& bounds) {
	return hashEntryMaxPoint.x < bounds.min_x ||
	       hashEntryMaxPoint.y < bounds.min_y ||
	       hashEntryMaxPoint.z < bounds.min_z ||
	       hashEntryMinPoint.x >= bounds.max_x ||
	       hashEntryMinPoint.y >= bounds.max_y ||
	       hashEntryMinPoint.z >= bounds.max_z;
}

_CPU_AND_GPU_CODE_
inline
Vector6i computeLocalBounds(const Vector3i& hashEntryMinPoint, const Vector3i& hashEntryMaxPoint,
                            const Vector6i& bounds) {
	return Vector6i(ORUTILS_MAX(0, bounds.min_x - hashEntryMinPoint.x),
	                ORUTILS_MAX(0, bounds.min_y - hashEntryMinPoint.y),
	                ORUTILS_MAX(0, bounds.min_z - hashEntryMinPoint.z),
	                ORUTILS_MIN(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE - (hashEntryMaxPoint.x - bounds.max_x)),
	                ORUTILS_MIN(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE - (hashEntryMaxPoint.y - bounds.max_y)),
	                ORUTILS_MIN(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE - (hashEntryMaxPoint.z - bounds.max_z)));
}

/**
 * \brief Look for the hash index of the hash entry with the specified position
 * \param hashIdx [out] the index of the hash entry corresponding to the specified position
 * \param hashBlockPosition [in] spacial position of the sough-after hash entry (in hash blocks)
 * \param hashTable [in] the hash table to search
 * \return true if hash block is allocated, false otherwise
 */
_CPU_AND_GPU_CODE_
inline bool FindHashAtPosition(THREADPTR(int)& hashIdx,
                               const CONSTPTR(Vector3s)& hashBlockPosition,
                               const CONSTPTR(ITMHashEntry)* hashTable) {
	hashIdx = HashCodeFromBlockPosition(hashBlockPosition);
	ITMHashEntry hashEntry = hashTable[hashIdx];

	if (!(IS_EQUAL3(hashEntry.pos, hashBlockPosition) && hashEntry.ptr >= -1)) {
		if (hashEntry.ptr >= -1) {
			//search excess list only if there is no room in ordered part
			while (hashEntry.offset >= 1) {
				hashIdx = ORDERED_LIST_SIZE + hashEntry.offset - 1;
				hashEntry = hashTable[hashIdx];

				if (IS_EQUAL3(hashEntry.pos, hashBlockPosition) && hashEntry.ptr >= -1) {
					return true;
				}
			}
			return false;
		}
		return false;
	}
	return true;
}

// endregion ===========================================================================================================


template<typename TVoxelPrimary, typename TVoxelSecondary, typename TFunctor>
struct ITMFlipArgumentBooleanFunctor {
	ITMFlipArgumentBooleanFunctor(TFunctor functor) : functor(functor) {}
	_CPU_AND_GPU_CODE_
	bool operator()(TVoxelPrimary& voxelPrimary, TVoxelSecondary& voxelSecondary) {
		return functor(voxelSecondary, voxelPrimary);
	}

	TFunctor functor;
};


template<typename TVoxel, typename TPredicateFunctor>
inline static bool
voxelBlockSatisfiesPredicate(TVoxel* voxelBlock,
                             TPredicateFunctor& oneVoxelPredicateFunctor) {
	for (int locId = 0; locId < VOXEL_BLOCK_SIZE3; locId++) {
		TVoxel& voxel = voxelBlock[locId];
		if (!oneVoxelPredicateFunctor(voxel)) {
			return false;
		}
	}
	return true;
}