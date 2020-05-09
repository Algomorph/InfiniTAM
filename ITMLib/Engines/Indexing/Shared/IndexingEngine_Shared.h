//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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

//ORUtils
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"

//local
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../../Objects/Views/View.h"
#include "../../../Objects/Tracking/CameraTrackingState.h"
#include "../../Common/WarpAccessFunctors.h"
#include "../../Common/AllocationTempData.h"
#include "../../../Utils/Math.h"
#include "../../../Utils/PixelUtils.h"
#include "../../../Utils/VoxelFlags.h"
#include "../../../Utils/HashBlockProperties.h"



using namespace ITMLib;

_CPU_AND_GPU_CODE_
inline bool
HashBlockAllocatedAtOffset(const CONSTPTR(ITMLib::VoxelBlockHash::IndexData)* target_index,
                           const THREADPTR(Vector3s)& block_position,
                           const CONSTPTR(Vector6i)& offset_block_range) {

	for (int z = block_position.z + offset_block_range.min_z; z < block_position.z + offset_block_range.max_z; z++) {
		for (int y = block_position.y + offset_block_range.min_y; y < block_position.y + offset_block_range.max_y; y++) {
			for (int x = block_position.x + offset_block_range.min_x; x < block_position.x + offset_block_range.max_x; x++) {
				if (FindHashCodeAt(target_index, Vector3s(x, y, z)) != -1) {
					return true;
				}
			}
		}
	}
	return false;
}



_CPU_AND_GPU_CODE_
inline
bool FindOrAllocateHashEntry(const Vector3s& hash_entry_position,
                             HashEntry* hash_table,
                             HashEntry*& result_entry,
                             int& last_free_voxel_block_id,
                             int& last_free_excess_list_id,
                             int& utilized_hash_code_count,
                             const int* voxel_allocation_list,
                             const int* excess_allocation_list,
                             int* utilized_hash_codes,
                             int& hash_code) {
	hash_code = HashCodeFromBlockPosition(hash_entry_position);
	HashEntry hash_entry = hash_table[hash_code];
	if (!IS_EQUAL3(hash_entry.pos, hash_entry_position) || hash_entry.ptr < -1) {
		bool add_to_excess_list = false;
		//search excess list only if there is no room in ordered part
		if (hash_entry.ptr >= -1) {
			while (hash_entry.offset >= 1) {
				hash_code = ORDERED_LIST_SIZE + hash_entry.offset - 1;
				hash_entry = hash_table[hash_code];
				if (IS_EQUAL3(hash_entry.pos, hash_entry_position) && hash_entry.ptr >= -1) {
					result_entry = &hash_table[hash_code];
					return true;
				}
			}
			add_to_excess_list = true;

		}
		//still not found, allocate
		if (add_to_excess_list && last_free_voxel_block_id >= 0 && last_free_excess_list_id >= 0) {
			//there is room in the voxel block array and excess list
			HashEntry new_hash_entry;
			new_hash_entry.pos = hash_entry_position;
			new_hash_entry.ptr = voxel_allocation_list[last_free_voxel_block_id];
			new_hash_entry.offset = 0;
			int excess_list_offset = excess_allocation_list[last_free_excess_list_id];
			hash_table[hash_code].offset = excess_list_offset + 1; //connect to child
			hash_code = ORDERED_LIST_SIZE + excess_list_offset;
			hash_table[hash_code] = new_hash_entry; //add child to the excess list
			result_entry = &hash_table[hash_code];
			last_free_voxel_block_id--;
			last_free_excess_list_id--;
			utilized_hash_codes[utilized_hash_code_count] = hash_code;
			utilized_hash_code_count++;
			return true;
		} else if (last_free_voxel_block_id >= 0) {
			//there is room in the voxel block array
			HashEntry new_hash_entry;
			new_hash_entry.pos = hash_entry_position;
			new_hash_entry.ptr = voxel_allocation_list[last_free_voxel_block_id];
			new_hash_entry.offset = 0;
			hash_table[hash_code] = new_hash_entry;
			result_entry = &hash_table[hash_code];
			last_free_voxel_block_id--;
			utilized_hash_codes[utilized_hash_code_count] = hash_code;
			utilized_hash_code_count++;
			return true;
		} else {
			return false;
		}
	} else {
		//HashEntry already exists, return the pointer to it
		result_entry = &hash_table[hash_code];
		return true;
	}
}

// region ================================= OFFSET COPYING =============================================================

_CPU_AND_GPU_CODE_
inline void
ComputeVoxelBlockOffsetRange(const CONSTPTR(Vector3i)& offset,
                             THREADPTR(Vector6i)& offsetRange) {
#define  ITM_CMP(a, b)    (((a) > (b)) - ((a) < (b)))
#define  ITM_SIGN(a)     ITM_CMP((a),0)

	int xA = offset.x / VOXEL_BLOCK_SIZE;
	int xB = xA + ITM_SIGN(offset.x % VOXEL_BLOCK_SIZE);
	int yA = offset.y / VOXEL_BLOCK_SIZE;
	int yB = yA + ITM_SIGN(offset.y % VOXEL_BLOCK_SIZE);
	int zA = offset.z / VOXEL_BLOCK_SIZE;
	int zB = zA + ITM_SIGN(offset.z % VOXEL_BLOCK_SIZE);
	int tmp;
	if (xA > xB) {
		tmp = xA;
		xA = xB;
		xB = tmp;
	}
	if (yA > yB) {
		tmp = yA;
		yA = yB;
		yB = tmp;
	}
	if (zA > zB) {
		tmp = zA;
		zA = zB;
		zB = tmp;
	}
	offsetRange.min_x = xA;
	offsetRange.min_y = yA;
	offsetRange.min_z = zA;
	offsetRange.max_x = xB + 1;
	offsetRange.max_y = yB + 1;
	offsetRange.max_z = zB + 1;
#undef ITM_CMP
#undef ITM_SIGN
}

//endregion