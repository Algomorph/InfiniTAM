//  ================================================================
//  Created by Gregory Kramida on 7/10/18.
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

//boost
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>

//local
#include "VolumeFileIOEngine.h"

using namespace ITMLib;
namespace b_ios = boost::iostreams;

//TODO: revise member functions & their usages to accept the full path as argument instead of the directory


template<typename TVoxel>
void VolumeFileIOEngine<TVoxel, VoxelBlockHash>::SaveToDirectoryCompact(
		const VoxelVolume<TVoxel, VoxelBlockHash>* scene,
		const std::string& outputDirectory) {


	std::string path = outputDirectory + compactFilePostfixAndExtension;
	std::ofstream ofStream = std::ofstream(path.c_str(), std::ios_base::binary | std::ios_base::out);
	if (!ofStream) throw std::runtime_error("Could not open '" + path + "' for writing.");

	b_ios::filtering_ostream outFilter;
	outFilter.push(b_ios::zlib_compressor());
	outFilter.push(ofStream);

	bool tempSceneUsed = false;
	if (scene->localVBA.GetMemoryType() == MEMORYDEVICE_CUDA) {
		// we cannot access CUDA blocks directly, so the easiest solution here is to make a local main-memory copy first
		VoxelVolume<TVoxel, VoxelBlockHash>* scene_cpu_copy = new VoxelVolume<TVoxel, VoxelBlockHash>(
				*scene, MEMORYDEVICE_CPU);
		scene = scene_cpu_copy;
		tempSceneUsed = true;
	}

	const TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.hashEntryCount;

	int lastExcessListId = scene->index.GetLastFreeExcessListId();
	int visibleHashBlockCount = scene->index.GetUtilizedHashBlockCount();
	outFilter.write(reinterpret_cast<const char* >(&scene->localVBA.lastFreeBlockId), sizeof(int));
	outFilter.write(reinterpret_cast<const char* >(&scene->localVBA.allocatedSize), sizeof(int));
	outFilter.write(reinterpret_cast<const char* >(&lastExcessListId), sizeof(int));
	outFilter.write(reinterpret_cast<const char* >(&visibleHashBlockCount), sizeof(int));
	//count filled entries
	int allocatedHashEntryCount = 0;
#ifdef WITH_OPENMP
#pragma omp parallel for reduction(+:allocatedHashEntryCount)
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = hashTable[entryId];
		//skip unfilled hash
		if (currentHashEntry.ptr < 0) continue;
		allocatedHashEntryCount++;
	}

	outFilter.write(reinterpret_cast<const char* >(&allocatedHashEntryCount), sizeof(int));
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = hashTable[entryId];
		//skip unfilled hash
		if (currentHashEntry.ptr < 0) continue;
		outFilter.write(reinterpret_cast<const char* >(&entryId), sizeof(int));
		outFilter.write(reinterpret_cast<const char* >(&currentHashEntry), sizeof(ITMHashEntry));
		const TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
		outFilter.write(reinterpret_cast<const char* >(localVoxelBlock), sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
	}

	if (tempSceneUsed) {
		delete scene;
	}
}

template<typename TVoxel>
void
VolumeFileIOEngine<TVoxel, VoxelBlockHash>::LoadFromDirectoryCompact(
		VoxelVolume<TVoxel, VoxelBlockHash>* scene,
		const std::string& outputDirectory) {
	std::string path = outputDirectory + "compact.dat";
	std::ifstream ifStream = std::ifstream(path.c_str(), std::ios_base::binary | std::ios_base::in);
	if (!ifStream) throw std::runtime_error("Could not open '" + path + "' for reading.");
	b_ios::filtering_istream inFilter;
	inFilter.push(b_ios::zlib_decompressor());
	inFilter.push(ifStream);

	VoxelVolume<TVoxel, VoxelBlockHash>* targetScene = scene;
	bool copyToCUDA = false;
	if (scene->localVBA.GetMemoryType() == MEMORYDEVICE_CUDA) {
		// we cannot access CUDA blocks directly, so the easiest solution here is to make a local main-memory copy
		// first, read it in from disk, and then copy it over into the target
		auto scene_cpu_copy = new VoxelVolume<TVoxel, VoxelBlockHash>(*scene, MEMORYDEVICE_CPU);
		scene = scene_cpu_copy;
		copyToCUDA = true;
	}

	TVoxel* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry* hashTable = scene->index.GetEntries();
	int lastExcessListId;
	int lastOrderedListId;
	int visibleHashBlockCount;
	inFilter.read(reinterpret_cast<char* >(&lastOrderedListId), sizeof(int));
	inFilter.read(reinterpret_cast<char* >(&scene->localVBA.allocatedSize), sizeof(int));
	inFilter.read(reinterpret_cast<char* >(&lastExcessListId), sizeof(int));
	inFilter.read(reinterpret_cast<char* >(&visibleHashBlockCount), sizeof(int));
	scene->index.SetLastFreeExcessListId(lastExcessListId);
	scene->index.SetUtilizedHashBlockCount(visibleHashBlockCount);
	scene->localVBA.lastFreeBlockId = lastOrderedListId;


	//count filled entries
	int allocatedHashEntryCount;
	inFilter.read(reinterpret_cast<char* >(&allocatedHashEntryCount), sizeof(int));
	for (int iEntry = 0; iEntry < allocatedHashEntryCount; iEntry++) {
		int entryId;
		inFilter.read(reinterpret_cast<char* >(&entryId), sizeof(int));
		ITMHashEntry& currentHashEntry = hashTable[entryId];
		inFilter.read(reinterpret_cast<char* >(&currentHashEntry), sizeof(ITMHashEntry));
		TVoxel* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
		inFilter.read(reinterpret_cast<char* >(localVoxelBlock), sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
	}

	if (copyToCUDA) {
		targetScene->SetFrom(*scene);
		delete scene;
	}
}


template<typename TVoxel>
void
VolumeFileIOEngine<TVoxel, PlainVoxelArray>::SaveToDirectoryCompact(
		const VoxelVolume<TVoxel, PlainVoxelArray>* scene,
		const std::string& outputDirectory) {
	scene->localVBA.SaveToDirectory(outputDirectory);
	scene->index.SaveToDirectory(outputDirectory);
}


template<typename TVoxel>
void
VolumeFileIOEngine<TVoxel, PlainVoxelArray>::LoadFromDirectoryCompact(
		VoxelVolume<TVoxel, PlainVoxelArray>* scene,
		const std::string& outputDirectory) {
	scene->localVBA.LoadFromDirectory(outputDirectory);
	scene->index.LoadFromDirectory(outputDirectory);
}