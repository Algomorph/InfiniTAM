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
#pragma once
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

		hashTableAllEntryTraversalWithHashCode_device<THashEntry, TFunctor>
		<<< cuda_grid_size, cuda_block_size >>>
				(hash_table, hash_entry_count, functor_device);
		ORcudaKernelCheck;

		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TVoxelBlockHash, typename THashEntry, typename TFunctor, typename TGetSubarraySizeFunction, typename TGetSubarrayFunction>
	inline static void
	TraverseSubarrayWithHashCode_Generic(TVoxelBlockHash& index, TFunctor& functor, TGetSubarraySizeFunction&& get_count,
	                                     TGetSubarrayFunction&& get_subarray) {
		THashEntry* hash_table = index.GetEntries();
		const int subarray_entry_count = std::forward<TGetSubarraySizeFunction>(get_count)();
		const int* subarray_entry_codes = std::forward<TGetSubarrayFunction>(get_subarray)();
		TFunctor* functor_device = nullptr;

		dim3 cuda_block_size(256, 1);
		dim3 cuda_grid_size((int) ceil((float) subarray_entry_count / (float) cuda_block_size.x));

		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		hashTableSubarrayEntryTraversalWithHashCode_device<THashEntry, TFunctor>
		<<< cuda_grid_size, cuda_block_size >>>
				(hash_table, subarray_entry_codes, subarray_entry_count, functor_device);
		ORcudaKernelCheck;

		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TVoxelBlockHash, typename THashEntry, typename TFunctor>
	inline static void
	TraverseUtilizedWithHashCode_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		TraverseSubarrayWithHashCode_Generic<TVoxelBlockHash, THashEntry, TFunctor>
				(index, functor,
				 [&index]() { return index.GetUtilizedBlockCount(); },
				 [&index]() { return index.GetUtilizedBlockHashCodes(); });
	}

	template<typename TVoxelBlockHash, typename THashEntry, typename TFunctor>
	inline static void
	TraverseVisibleWithHashCode_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		TraverseSubarrayWithHashCode_Generic<TVoxelBlockHash, THashEntry, TFunctor>
				(index, functor,
				 [&index]() { return index.GetVisibleBlockCount(); },
				 [&index]() { return index.GetVisibleBlockHashCodes(); });
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
	TraverseUtilizedWithHashCode(VoxelBlockHash& index, TFunctor& functor) {
		TraverseUtilizedWithHashCode_Generic<VoxelBlockHash, HashEntry, TFunctor>(index, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithHashCode(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseUtilizedWithHashCode_Generic<const VoxelBlockHash, const HashEntry, TFunctor>(index, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseVisibleWithHashCode(VoxelBlockHash& index, TFunctor& functor) {
		TraverseVisibleWithHashCode_Generic<VoxelBlockHash, HashEntry, TFunctor>(index, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseVisibleWithHashCode(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseVisibleWithHashCode_Generic<const VoxelBlockHash, const HashEntry, TFunctor>(index, functor);
	}

};

} // namespace ITMLib