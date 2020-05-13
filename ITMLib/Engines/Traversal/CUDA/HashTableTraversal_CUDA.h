//  ================================================================
//  Created by Gregory Kramida on 2/4/20.
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
//local
#include "../Interface/HashTableTraversal.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "HashTableTraversal_CUDA_Kernels.h"

namespace ITMLib {

template<>
class HashTableTraversalEngine<MEMORYDEVICE_CUDA> {
private: // member functions
	template<typename TVoxelBlockHash, typename THashEntry, typename TFunctor>
	inline static void
	TraverseAllWithHashCode_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		THashEntry* hash_table = index.GetEntries();
		const int hash_entry_count = index.hash_entry_count;
		TFunctor* functor_device = nullptr;

		dim3 cuda_block_size(256, 1);
		dim3 cuda_grid_size((int) ceil((float) hash_entry_count / (float) cuda_block_size.x));

		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		hashTableAllEntryTraversalWithHashCode_device < TFunctor >
		<<< cuda_grid_size, cuda_block_size >>>
				(hash_table, hash_entry_count, functor_device);
		ORcudaKernelCheck;

		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}
	template<typename TVoxelBlockHash, typename THashEntry, typename TFunctor>
	inline static void
	TraverseUtilizedWithHashCode_Generic(TVoxelBlockHash& index, TFunctor& functor){
		THashEntry* hash_table = index.GetEntries();
		const int utilized_entry_count = index.GetUtilizedBlockCount();
		const int* utilized_entry_codes = index.GetUtilizedBlockHashCodes();
		TFunctor* functor_device = nullptr;

		dim3 cuda_block_size(256, 1);
		dim3 cuda_grid_size((int) ceil((float) utilized_entry_count / (float) cuda_block_size.x));

		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		hashTableUtilizedEntryTraversalWithHashCode_device < TFunctor >
		<<< cuda_grid_size, cuda_block_size >>>
				(hash_table, utilized_entry_codes, utilized_entry_count, functor_device);
		ORcudaKernelCheck;

		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}
public: // member functions
	template<typename TFunctor>
	inline static void TraverseAllWithHashCode(VoxelBlockHash& index, TFunctor& functor) {
		TraverseAllWithHashCode_Generic<VoxelBlockHash, HashEntry, TFunctor>(index, functor);
	}
	template<typename TFunctor>
	inline static void TraverseAllWithHashCode(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseAllWithHashCode_Generic<const VoxelBlockHash, const HashEntry, TFunctor>(index, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithHashCode(VoxelBlockHash& index, TFunctor& functor){
		TraverseUtilizedWithHashCode_Generic<VoxelBlockHash, HashEntry, TFunctor>(index, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithHashCode(const VoxelBlockHash& index, TFunctor& functor){
		TraverseUtilizedWithHashCode_Generic<const VoxelBlockHash, const HashEntry, TFunctor>(index, functor);
	}
};

} // namespace ITMLib