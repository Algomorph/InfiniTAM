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
#include "../../Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
#include "../../Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"

namespace ITMLib {

HashEntry VoxelBlockHash::GetHashEntryAt(const Vector3s& pos, int& hash_code) const {
	const HashEntry* entries = this->GetEntries();
	switch (memory_type) {
		case MEMORYDEVICE_CPU:
			return IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
					.FindHashEntry(*this, pos, hash_code);
#ifndef COMPILE_WITHOUT_CUDA
		case MEMORYDEVICE_CUDA:
			return IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance()
					.FindHashEntry(*this,pos, hash_code);
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
		voxelBlockCount(parameters.voxel_block_count),
		excessListSize(parameters.excess_list_size),
		hash_entry_count(ORDERED_LIST_SIZE + parameters.excess_list_size),
		last_free_excess_list_id(parameters.excess_list_size - 1),
		hash_entry_allocation_states(ORDERED_LIST_SIZE + parameters.excess_list_size, memory_type),
		allocationBlockCoordinates(ORDERED_LIST_SIZE + parameters.excess_list_size, memory_type),
		utilized_block_hash_codes(parameters.voxel_block_count, memory_type),
		visible_block_hash_codes(parameters.voxel_block_count, memory_type),
		block_visibility_types(ORDERED_LIST_SIZE + parameters.excess_list_size, memory_type),
		memory_type(memory_type),
		hash_entries(hash_entry_count, memory_type),
		excess_allocation_list(excessListSize, memory_type),
		utilized_hash_block_count(0),
		visible_hash_block_count(0)
		{
	hash_entry_allocation_states.Clear(NEEDS_NO_CHANGE);

}

void VoxelBlockHash::SaveToDirectory(const std::string& outputDirectory) const {
	std::string hashEntriesFileName = outputDirectory + "hash.dat";
	std::string excessAllocationListFileName = outputDirectory + "excess.dat";
	std::string lastFreeExcessListIdFileName = outputDirectory + "last.txt";

	std::ofstream ofs(lastFreeExcessListIdFileName.c_str());
	if (!ofs) throw std::runtime_error("Could not open " + lastFreeExcessListIdFileName + " for writing");

	ofs << last_free_excess_list_id;
	ORUtils::MemoryBlockPersister::SaveMemoryBlock(hashEntriesFileName, hash_entries, memory_type);
	ORUtils::MemoryBlockPersister::SaveMemoryBlock(excessAllocationListFileName, excess_allocation_list, memory_type);
}

void VoxelBlockHash::LoadFromDirectory(const std::string& inputDirectory) {
	std::string hashEntriesFileName = inputDirectory + "hash.dat";
	std::string excessAllocationListFileName = inputDirectory + "excess.dat";
	std::string lastFreeExcessListIdFileName = inputDirectory + "last.txt";

	std::ifstream ifs(lastFreeExcessListIdFileName.c_str());
	if (!ifs) throw std::runtime_error("Count not open " + lastFreeExcessListIdFileName + " for reading");

	ifs >> this->last_free_excess_list_id;
	ORUtils::MemoryBlockPersister::LoadMemoryBlock(hashEntriesFileName, hash_entries, memory_type);
	ORUtils::MemoryBlockPersister::LoadMemoryBlock(excessAllocationListFileName, excess_allocation_list,
	                                               memory_type);
}



}// namespace ITMLib
