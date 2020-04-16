//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/16/20.
//  Copyright (c) 2020 Gregory Kramida
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

#include "GlobalCache.h"

using namespace ITMLib;

template<typename TVoxel>
GlobalCache<TVoxel, PlainVoxelArray>::GlobalCache(const PlainVoxelArray& index) {
	DIEWITHEXCEPTION_REPORTLOCATION("Swappling / global cache not implemented for plain voxel array indexing.");
}


template<typename TVoxel>
GlobalCache<TVoxel, VoxelBlockHash>::GlobalCache(const int hash_entry_count) : hash_entry_count(hash_entry_count) {
	has_stored_data = (bool*) malloc(hash_entry_count * sizeof(bool));
	stored_voxel_blocks = (TVoxel*) malloc(hash_entry_count * sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
	memset(has_stored_data, 0, hash_entry_count);

	swap_states_host = (ITMHashSwapState*) malloc(hash_entry_count * sizeof(ITMHashSwapState));
	memset(swap_states_host, 0, sizeof(ITMHashSwapState) * hash_entry_count);

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaMallocHost((void**) &synced_voxel_blocks_host,
	                              SWAP_OPERATION_BLOCK_COUNT * sizeof(TVoxel) * VOXEL_BLOCK_SIZE3));
	ORcudaSafeCall(cudaMallocHost((void**) &has_synced_data_host, SWAP_OPERATION_BLOCK_COUNT * sizeof(bool)));
	ORcudaSafeCall(cudaMallocHost((void**) &needed_hash_codes_host, SWAP_OPERATION_BLOCK_COUNT * sizeof(int)));

	ORcudaSafeCall(cudaMalloc((void**) &swap_states_device, hash_entry_count * sizeof(ITMHashSwapState)));
	ORcudaSafeCall(cudaMemset(swap_states_device, 0, hash_entry_count * sizeof(ITMHashSwapState)));

	ORcudaSafeCall(cudaMalloc((void**) &synced_voxel_blocks_device,
	                          SWAP_OPERATION_BLOCK_COUNT * sizeof(TVoxel) * VOXEL_BLOCK_SIZE3));
	ORcudaSafeCall(cudaMalloc((void**) &has_synced_data_device, SWAP_OPERATION_BLOCK_COUNT * sizeof(bool)));

	ORcudaSafeCall(cudaMalloc((void**) &needed_hash_codes_device, SWAP_OPERATION_BLOCK_COUNT * sizeof(int)));
#else
	synced_voxel_blocks_host = (TVoxel*)malloc(SWAP_OPERATION_BLOCK_COUNT * sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
	has_synced_data_host = (bool*)malloc(SWAP_OPERATION_BLOCK_COUNT * sizeof(bool));
	needed_hash_codes_host = (int*)malloc(SWAP_OPERATION_BLOCK_COUNT * sizeof(int));
#endif
}

template<typename TVoxel>
GlobalCache<TVoxel, VoxelBlockHash>::GlobalCache(const GlobalCache& other) : GlobalCache(other.hash_entry_count) {

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaMemcpy(synced_voxel_blocks_host, other.synced_voxel_blocks_host,
	                          SWAP_OPERATION_BLOCK_COUNT * sizeof(TVoxel) * VOXEL_BLOCK_SIZE3, cudaMemcpyHostToHost));
	ORcudaSafeCall(cudaMemcpy(has_synced_data_host, other.has_synced_data_host,
	                          SWAP_OPERATION_BLOCK_COUNT * sizeof(bool), cudaMemcpyHostToHost));
	ORcudaSafeCall(cudaMemcpy(needed_hash_codes_host, other.needed_hash_codes_host,
	                          SWAP_OPERATION_BLOCK_COUNT * sizeof(int), cudaMemcpyHostToHost));
	ORcudaSafeCall(cudaMemcpy(swap_states_device, other.swap_states_device,
	                          hash_entry_count * sizeof(ITMHashSwapState), cudaMemcpyDeviceToDevice));
	ORcudaSafeCall(cudaMemcpy(synced_voxel_blocks_device, other.synced_voxel_blocks_device,
	                          SWAP_OPERATION_BLOCK_COUNT * sizeof(TVoxel) * VOXEL_BLOCK_SIZE3,
	                          cudaMemcpyDeviceToDevice));
	ORcudaSafeCall(cudaMemcpy(has_synced_data_device, other.has_synced_data_device,
	                          SWAP_OPERATION_BLOCK_COUNT * sizeof(bool), cudaMemcpyDeviceToDevice));
	ORcudaSafeCall(cudaMemcpy(needed_hash_codes_device, other.needed_hash_codes_device,
	                          SWAP_OPERATION_BLOCK_COUNT * sizeof(int), cudaMemcpyDeviceToDevice));
#else
	memcpy(synced_voxel_blocks_host, other.synced_voxel_blocks_host,SWAP_OPERATION_BLOCK_COUNT * sizeof(TVoxel) * VOXEL_BLOCK_SIZE3);
	memcpy(has_synced_data_host, other.has_synced_data_host, SWAP_OPERATION_BLOCK_COUNT * sizeof(bool));
	memcpy(needed_hash_codes_host,other.needed_hash_codes_host, SWAP_OPERATION_BLOCK_COUNT * sizeof(int));
#endif
}

template<typename TVoxel>
GlobalCache<TVoxel, VoxelBlockHash>::~GlobalCache() {
	free(has_stored_data);
	free(stored_voxel_blocks);

	free(swap_states_host);

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaFreeHost(has_synced_data_host));
	ORcudaSafeCall(cudaFreeHost(synced_voxel_blocks_host));
	ORcudaSafeCall(cudaFreeHost(needed_hash_codes_host));

	ORcudaSafeCall(cudaFree(swap_states_device));
	ORcudaSafeCall(cudaFree(synced_voxel_blocks_device));
	ORcudaSafeCall(cudaFree(has_synced_data_device));
	ORcudaSafeCall(cudaFree(needed_hash_codes_device));
#else
	free(has_synced_data_host);
			free(synced_voxel_blocks_host);
			free(needed_hash_codes_host);
#endif
}

template<typename TVoxel>
void GlobalCache<TVoxel, VoxelBlockHash>::Swap(GlobalCache<TVoxel, VoxelBlockHash>& rhs) {
	using std::swap;

	swap(this->hash_entry_count, rhs.hash_entry_count);

	swap(this->has_stored_data, rhs.has_stored_data);
	swap(this->stored_voxel_blocks, rhs.stored_voxel_blocks);
	swap(this->swap_states_host, rhs.swap_states_host);
	swap(this->swap_states_device, rhs.swap_states_device);

	swap(this->has_synced_data_host, rhs.has_synced_data_host);
	swap(this->has_synced_data_device, rhs.has_synced_data_device);
	swap(this->synced_voxel_blocks_host, rhs.synced_voxel_blocks_host);
	swap(this->synced_voxel_blocks_device, rhs.synced_voxel_blocks_device);

	swap(this->needed_hash_codes_host, rhs.needed_hash_codes_host);
	swap(this->needed_hash_codes_device, rhs.needed_hash_codes_device);
}

template<typename TVoxel>
GlobalCache<TVoxel, VoxelBlockHash>::GlobalCache(GlobalCache&& other) :
		hash_entry_count(other.hash_entry_count),

		has_stored_data(other.has_stored_data),
		stored_voxel_blocks(other.stored_voxel_blocks),
		swap_states_host(other.swap_states_host),
		swap_states_device(other.swap_states_device),

		has_synced_data_host(other.has_synced_data_host),
		has_synced_data_device(other.has_synced_data_device),
		synced_voxel_blocks_host(other.synced_voxel_blocks_host),
		synced_voxel_blocks_device(other.synced_voxel_blocks_device),

		needed_hash_codes_host(other.needed_hash_codes_host),
		needed_hash_codes_device(other.needed_hash_codes_device) {
	other.has_stored_data = nullptr;
	other.stored_voxel_blocks = nullptr;
	other.swap_states_host = nullptr;
	other.swap_states_device = nullptr;

	other.has_synced_data_host = nullptr;
	other.has_synced_data_device = nullptr;
	other.synced_voxel_blocks_host = nullptr;
	other.synced_voxel_blocks_device = nullptr;

	other.needed_hash_codes_host = nullptr;
	other.needed_hash_codes_device = nullptr;
}

template<typename TVoxel>
int GlobalCache<TVoxel, VoxelBlockHash>::GetHashEntryCount() const {
	return this->hash_entry_count;
}

