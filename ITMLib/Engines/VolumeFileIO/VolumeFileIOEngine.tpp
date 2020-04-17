//  ================================================================
//  Created by Gregory Kramida on 7/10/18.
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

//boost
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>

//local
#include "VolumeFileIOEngine.h"

using namespace ITMLib;
namespace b_ios = boost::iostreams;


// region ==================================== VOXEL BLOCK HASH ========================================================

template<typename TVoxel>
void VolumeFileIOEngine<TVoxel, VoxelBlockHash>::SaveVolumeCompact(
		const VoxelVolume<TVoxel, VoxelBlockHash>& volume,
		const std::string& path) {

	std::ofstream volume_output_stream = std::ofstream(path.c_str(), std::ios_base::binary | std::ios_base::out);
	if (!volume_output_stream) throw std::runtime_error("Could not open '" + path + "' for writing.");

	b_ios::filtering_ostream out_filter;
	out_filter.push(b_ios::zlib_compressor());
	out_filter.push(volume_output_stream);

	bool temporary_volume_used = false;

	const VoxelVolume<TVoxel, VoxelBlockHash>* volume_to_write = &volume;

	if (volume.index.memory_type == MEMORYDEVICE_CUDA) {
		// we cannot access CUDA blocks directly, so the easiest solution here is to make a local main-memory copy first
		auto volume_cpu_copy = new VoxelVolume<TVoxel, VoxelBlockHash>(volume, MEMORYDEVICE_CPU);
		volume_to_write = volume_cpu_copy;
		temporary_volume_used = true;
	}

	const TVoxel* voxels = volume_to_write->GetVoxels();
	const HashEntry* hash_table = volume_to_write->index.GetEntries();
	const int hash_entry_count = volume_to_write->index.hash_entry_count;

	int last_free_block_id = volume_to_write->index.GetLastFreeBlockListId();
	int last_excess_list_id = volume_to_write->index.GetLastFreeExcessListId();
	int utilized_block_count = volume_to_write->index.GetUtilizedBlockCount();
	out_filter.write(reinterpret_cast<const char* >(&last_free_block_id), sizeof(int));
	out_filter.write(reinterpret_cast<const char* >(&last_excess_list_id), sizeof(int));
	out_filter.write(reinterpret_cast<const char* >(&utilized_block_count), sizeof(int));
	//count filled entries
	int allocated_hash_entry_count = 0;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none)  shared(out_filter, hash_table) reduction(+:allocated_hash_entry_count)
#endif
	for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
		const HashEntry& hash_entry = hash_table[hash_code];
		//skip unfilled hash
		if (hash_entry.ptr < 0) continue;
		allocated_hash_entry_count++;
	}

	out_filter.write(reinterpret_cast<const char* >(&allocated_hash_entry_count), sizeof(int));
	for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
		const HashEntry& hash_entry = hash_table[hash_code];
		//skip unfilled hash
		if (hash_entry.ptr < 0) continue;
		out_filter.write(reinterpret_cast<const char* >(&hash_code), sizeof(int));
		out_filter.write(reinterpret_cast<const char* >(&hash_entry), sizeof(HashEntry));
		const TVoxel* voxel_block = &(voxels[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
		out_filter.write(reinterpret_cast<const char* >(voxel_block), sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
	}
	const int* utilized_hash_codes = volume_to_write->index.GetUtilizedBlockHashCodes();
	for (int i_utilized_hash_code = 0; i_utilized_hash_code < utilized_block_count; i_utilized_hash_code++) {
		out_filter.write(reinterpret_cast<const char* >(utilized_hash_codes + i_utilized_hash_code), sizeof(int));
	}


	if (temporary_volume_used) {
		delete volume_to_write;
	}
}

template<typename TVoxel>
void
VolumeFileIOEngine<TVoxel, VoxelBlockHash>::LoadVolumeCompact(VoxelVolume<TVoxel, VoxelBlockHash>& volume,
                                                              const std::string& path) {
	std::ifstream if_stream = std::ifstream(path.c_str(), std::ios_base::binary | std::ios_base::in);
	if (!if_stream) throw std::runtime_error("Could not open '" + path + "' for reading.");
	b_ios::filtering_istream in_filter;
	in_filter.push(b_ios::zlib_decompressor());
	in_filter.push(if_stream);

	VoxelVolume<TVoxel, VoxelBlockHash>* volume_to_load = &volume;
	bool temporary_volume_used = false;
	if (volume.index.memory_type == MEMORYDEVICE_CUDA) {
		// we cannot access CUDA blocks directly, so the easiest solution here is to make a local main-memory copy
		// first, read it in from disk, and then copy it over into the target
		auto scene_cpu_copy = new VoxelVolume<TVoxel, VoxelBlockHash>(*volume_to_load, MEMORYDEVICE_CPU);
		volume_to_load = scene_cpu_copy;
		temporary_volume_used = true;
	}

	TVoxel* voxel_blocks = volume_to_load->GetVoxels();
	HashEntry* hash_table = volume_to_load->index.GetEntries();
	int last_free_excess_list_id;
	int last_free_voxel_block_id;
	int utilized_block_count;
	in_filter.read(reinterpret_cast<char* >(&last_free_voxel_block_id), sizeof(int));
	in_filter.read(reinterpret_cast<char* >(&last_free_excess_list_id), sizeof(int));
	in_filter.read(reinterpret_cast<char* >(&utilized_block_count), sizeof(int));
	volume_to_load->index.SetLastFreeExcessListId(last_free_excess_list_id);
	volume_to_load->index.SetUtilizedBlockCount(utilized_block_count);
	volume_to_load->index.SetLastFreeBlockListId(last_free_voxel_block_id);


	//count filled entries
	int allocated_hash_entry_count;
	in_filter.read(reinterpret_cast<char* >(&allocated_hash_entry_count), sizeof(int));
	for (int i_block = 0; i_block < allocated_hash_entry_count; i_block++) {
		int hash_code;
		in_filter.read(reinterpret_cast<char* >(&hash_code), sizeof(int));
		HashEntry& hash_entry = hash_table[hash_code];
		in_filter.read(reinterpret_cast<char* >(&hash_entry), sizeof(HashEntry));
		TVoxel* voxel_block = &(voxel_blocks[hash_entry.ptr * (VOXEL_BLOCK_SIZE3)]);
		in_filter.read(reinterpret_cast<char* >(voxel_block), sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
	}
	int* utilized_hash_codes = volume_to_load->index.GetUtilizedBlockHashCodes();
	for (int i_utilized_hash_code = 0; i_utilized_hash_code < utilized_block_count; i_utilized_hash_code++) {
		in_filter.read(reinterpret_cast<char* >(utilized_hash_codes + i_utilized_hash_code), sizeof(int));
	}

	if (temporary_volume_used) {
		volume.SetFrom(*volume_to_load);
		delete volume_to_load;
	}
}

template<typename TVoxel>
void VolumeFileIOEngine<TVoxel, VoxelBlockHash>::AppendFileWithUtilizedIndexSpaceInformation(
		ORUtils::MemoryBlockOStreamWrapper& file, const VoxelVolume<TVoxel, VoxelBlockHash>& volume) {

}

// endregion ===========================================================================================================
// region ================================= PLAIN VOXEL ARRAY ==========================================================

template<typename TVoxel>
void
VolumeFileIOEngine<TVoxel, PlainVoxelArray>::SaveVolumeCompact(
		const VoxelVolume<TVoxel, PlainVoxelArray>& volume,
		const std::string& path) {
	ORUtils::MemoryBlockOStreamWrapper file(path, true);
	volume.index.Save(file);
	volume.SaveVoxels(file);
}


template<typename TVoxel>
void
VolumeFileIOEngine<TVoxel, PlainVoxelArray>::LoadVolumeCompact(
		VoxelVolume<TVoxel, PlainVoxelArray>& volume,
		const std::string& path) {
	ORUtils::MemoryBlockIStreamWrapper file(path, true);
	volume.index.Load(file);
	volume.LoadVoxels(file);
}

template<typename TVoxel>
void VolumeFileIOEngine<TVoxel, PlainVoxelArray>::AppendFileWithUtilizedIndexSpaceInformation(
		ORUtils::MemoryBlockOStreamWrapper& file, const VoxelVolume<TVoxel, PlainVoxelArray>& volume) {

}

// endregion ===========================================================================================================