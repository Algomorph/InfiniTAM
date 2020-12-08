//  ================================================================
//  Created by Gregory Kramida on 11/8/19.
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

#include "VoxelBlockHash.h"
#include "../../GlobalTemplateDefines.h"
#include "../../Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
#include "../../Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"

namespace ITMLib {

DEFINE_PATHLESS_SERIALIZABLE_STRUCT(VOXEL_BLOCK_HASH_PARAMETERS_STRUCT_DESCRIPTION);

VoxelBlockHash::VoxelBlockHash()
		: memory_type(MEMORYDEVICE_NONE),
		  voxel_block_count(0),
		  excess_list_size(0),
		  hash_entry_count(0),
		  voxel_block_size(VOXEL_BLOCK_SIZE3),

		  last_free_block_list_id(0),
		  last_free_excess_list_id(0),
		  utilized_block_count(0),
		  visible_block_count(0),

		  hash_entries(), hash_entry_allocation_states(), allocation_block_coordinates(), block_allocation_list(),
		  excess_entry_list(), visible_block_hash_codes(), utilized_block_hash_codes(), block_visibility_types(){}

HashEntry VoxelBlockHash::GetHashEntryAt(const Vector3s& pos, int& hash_code) const {
	const HashEntry* entries = this->GetEntries();
	switch (memory_type) {
		case MEMORYDEVICE_CPU:
			return IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
					.FindHashEntry(*this, pos, hash_code);
#ifndef COMPILE_WITHOUT_CUDA
		case MEMORYDEVICE_CUDA:
			return IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance()
					.FindHashEntry(*this, pos, hash_code);
#endif
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
			return HashEntry();
	}
}

HashEntry VoxelBlockHash::GetHashEntryAt(const Vector3s& pos) const {
	int hashCode = 0;
	return GetHashEntryAt(pos, hashCode);
}

HashEntry VoxelBlockHash::GetHashEntryAt(int x, int y, int z, int& hash_code) const {
	Vector3s coord(x, y, z);
	return GetHashEntryAt(coord, hash_code);
}

HashEntry VoxelBlockHash::GetHashEntryAt(int x, int y, int z) const {
	Vector3s coord(x, y, z);
	return GetHashEntryAt(coord);
}

HashEntry VoxelBlockHash::GetUtilizedHashEntryAtIndex(int index, int& hash_code) const {
	hash_code = utilized_block_hash_codes.GetElement(index, memory_type);
	return GetHashEntry(hash_code);
}

HashEntry VoxelBlockHash::GetUtilizedHashEntryAtIndex(int index) const {
	int hash_code = 0;
	return GetUtilizedHashEntryAtIndex(index, hash_code);
}

VoxelBlockHash::VoxelBlockHash(VoxelBlockHashParameters parameters, MemoryDeviceType memory_type) :
		voxel_block_count(parameters.voxel_block_count),
		excess_list_size(parameters.excess_list_size),
		hash_entry_count(ORDERED_LIST_SIZE + parameters.excess_list_size),
		last_free_block_list_id(parameters.voxel_block_count - 1),
		last_free_excess_list_id(parameters.excess_list_size - 1),
		hash_entry_allocation_states(ORDERED_LIST_SIZE + parameters.excess_list_size, memory_type),
		allocation_block_coordinates(ORDERED_LIST_SIZE + parameters.excess_list_size, memory_type),
		utilized_block_hash_codes(parameters.voxel_block_count, memory_type),
		visible_block_hash_codes(parameters.voxel_block_count, memory_type),
		block_visibility_types(ORDERED_LIST_SIZE + parameters.excess_list_size, memory_type),
		memory_type(memory_type),
		hash_entries(hash_entry_count, memory_type),
		block_allocation_list(voxel_block_count, memory_type),
		excess_entry_list(excess_list_size, memory_type),
		utilized_block_count(0),
		visible_block_count(0) {
	hash_entry_allocation_states.Clear(NEEDS_NO_CHANGE);

}

void VoxelBlockHash::SetFrom(const VoxelBlockHash& other) {
	MemoryCopyDirection memory_copy_direction = DetermineMemoryCopyDirection(this->memory_type, other.memory_type);
	this->hash_entry_allocation_states.SetFrom(other.hash_entry_allocation_states, memory_copy_direction);
	this->hash_entries.SetFrom(other.hash_entries, memory_copy_direction);
	this->block_allocation_list.SetFrom(other.block_allocation_list, memory_copy_direction);
	this->excess_entry_list.SetFrom(other.excess_entry_list, memory_copy_direction);
	this->utilized_block_hash_codes.SetFrom(other.utilized_block_hash_codes, memory_copy_direction);
	this->visible_block_hash_codes.SetFrom(other.visible_block_hash_codes, memory_copy_direction);
	this->last_free_block_list_id = other.last_free_block_list_id;
	this->last_free_excess_list_id = other.last_free_excess_list_id;
	this->utilized_block_count = other.utilized_block_count;
}

HashEntry VoxelBlockHash::GetHashEntry(int hash_code) const {
	if(hash_code < 0 || hash_code >= hash_entries.size()){
		return {Vector3s(0, 0, 0), 0, -2};
	}
	return hash_entries.GetElement(hash_code, memory_type);
}



}// namespace ITMLib
