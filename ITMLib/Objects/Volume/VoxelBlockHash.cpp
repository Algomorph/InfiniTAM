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

HashEntry VoxelBlockHash::GetHashEntryAt(const Vector3s& pos, int& hashCode) const {
	const HashEntry* entries = this->GetEntries();
	switch (memoryType) {
		case MEMORYDEVICE_CPU:
			return IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
					.FindHashEntry(*this,pos,hashCode);
#ifndef COMPILE_WITHOUT_CUDA
		case MEMORYDEVICE_CUDA:
			return IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance()
					.FindHashEntry(*this,pos, hashCode);
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

VoxelBlockHash::VoxelBlockHash(VoxelBlockHashParameters parameters, MemoryDeviceType memoryType) :
		voxelBlockCount(parameters.voxel_block_count),
		excessListSize(parameters.excess_list_size),
		hashEntryCount(ORDERED_LIST_SIZE + parameters.excess_list_size),
		lastFreeExcessListId(parameters.excess_list_size - 1),
		hashEntryAllocationStates(ORDERED_LIST_SIZE + parameters.excess_list_size, memoryType),
		allocationBlockCoordinates(ORDERED_LIST_SIZE + parameters.excess_list_size, memoryType),
		utilized_block_hash_codes(parameters.voxel_block_count, memoryType),
		blockVisibilityTypes(ORDERED_LIST_SIZE + parameters.excess_list_size, memoryType),
		memoryType(memoryType),
		hashEntries(hashEntryCount, memoryType),
		excessAllocationList(excessListSize, memoryType),
		utilizedHashBlockCount(0)
		{
	hashEntryAllocationStates.Clear(NEEDS_NO_CHANGE);

}

void VoxelBlockHash::SaveToDirectory(const std::string& outputDirectory) const {
	std::string hashEntriesFileName = outputDirectory + "hash.dat";
	std::string excessAllocationListFileName = outputDirectory + "excess.dat";
	std::string lastFreeExcessListIdFileName = outputDirectory + "last.txt";

	std::ofstream ofs(lastFreeExcessListIdFileName.c_str());
	if (!ofs) throw std::runtime_error("Could not open " + lastFreeExcessListIdFileName + " for writing");

	ofs << lastFreeExcessListId;
	ORUtils::MemoryBlockPersister::SaveMemoryBlock(hashEntriesFileName, hashEntries, memoryType);
	ORUtils::MemoryBlockPersister::SaveMemoryBlock(excessAllocationListFileName, excessAllocationList, memoryType);
}

void VoxelBlockHash::LoadFromDirectory(const std::string& inputDirectory) {
	std::string hashEntriesFileName = inputDirectory + "hash.dat";
	std::string excessAllocationListFileName = inputDirectory + "excess.dat";
	std::string lastFreeExcessListIdFileName = inputDirectory + "last.txt";

	std::ifstream ifs(lastFreeExcessListIdFileName.c_str());
	if (!ifs) throw std::runtime_error("Count not open " + lastFreeExcessListIdFileName + " for reading");

	ifs >> this->lastFreeExcessListId;
	ORUtils::MemoryBlockPersister::LoadMemoryBlock(hashEntriesFileName, hashEntries, memoryType);
	ORUtils::MemoryBlockPersister::LoadMemoryBlock(excessAllocationListFileName, excessAllocationList,
	                                               memoryType);
}

}// namespace ITMLib
