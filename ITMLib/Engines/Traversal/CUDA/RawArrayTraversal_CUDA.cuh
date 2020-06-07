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
#include "../Interface/RawArrayTraversal.h"
#include "../../../../ORUtils/MemoryBlock.h"
#include "RawArrayTraversal_CUDA_Kernels.cuh"
#include "../Shared/CudaCallWrappers.cuh"

namespace ITMLib {

namespace internal {

template<>
class RawArrayTraversalEngine_Internal<MEMORYDEVICE_CUDA, JobCountPolicy::EXACT, CONTIGUOUS> {
	friend class RawArrayTraversalEngine<MEMORYDEVICE_CUDA>;
protected: // static functions
	template<typename TData, typename TFunctor, int TBlockSize = 256>
	inline static void TraverseWithIndex_Generic(TData* data, TFunctor& functor, const unsigned int element_count) {
		CallCUDAonUploadedFunctor(
				functor,
				[&element_count, &data](TFunctor* functor_device) {
					dim3 cuda_block_size(TBlockSize);
					dim3 cuda_grid_size(ceil_of_integer_quotient(element_count, cuda_block_size.x));
					TraverseContiguousWithIndex_Exact_device<TData, TFunctor> <<<cuda_grid_size, cuda_block_size>>>
							(data, element_count, functor_device);
				}
		);
	}

	template<typename TData, typename TFunctor, int TBlockSize = 256>
	inline static void TraverseWithoutIndex_Generic(TData* data, TFunctor& functor, const unsigned int element_count) {
		CallCUDAonUploadedFunctor(
				functor,
				[&element_count, &data](TFunctor* functor_device) {
					dim3 cuda_block_size(TBlockSize);
					dim3 cuda_grid_size(ceil_of_integer_quotient(element_count, cuda_block_size.x));
					TraverseContiguousWithoutItemIndex_Exact_device<TData, TFunctor> <<<cuda_grid_size, cuda_block_size>>>
							(data, element_count, functor_device);
				}
		);
	}
};

template<>
class RawArrayTraversalEngine_Internal<MEMORYDEVICE_CUDA, JobCountPolicy::EXACT, INDEX_SAMPLE> {
	friend class RawArrayTraversalEngine<MEMORYDEVICE_CUDA>;
protected: // static functions
	template<typename TData, typename TFunctor, typename TGetSampleSizeFunction, typename TGetSampleIndicesFunction, int TBlockSize = 256>
	inline static void TraverseWithIndex_Generic(TData* data, TFunctor& functor,
	                                             TGetSampleSizeFunction&& get_sample_size, TGetSampleIndicesFunction&& get_sample_indices) {
		const int sample_size = std::forward<TGetSampleSizeFunction>(get_sample_size)();
		const int* sample_indices = std::forward<TGetSampleIndicesFunction>(get_sample_indices)();
		CallCUDAonUploadedFunctor(
				functor,
				[&sample_size, &sample_indices, &data](TFunctor* functor_device) {
					dim3 cuda_block_size(TBlockSize);
					dim3 cuda_grid_size(ceil_of_integer_quotient(sample_size, cuda_block_size.x));
					TraverseSampleWithIndex_Exact_device<TData, TFunctor> <<<cuda_grid_size, cuda_block_size>>>
							(data, sample_indices, sample_size, functor_device);
				}
		);
	}

	template<typename TData, typename TFunctor, typename TGetSampleSizeFunction, typename TGetSampleIndicesFunction, int TBlockSize = 256>
	inline static void TraverseWithoutIndex_Generic(TData* data, TFunctor& functor,
	                                                TGetSampleSizeFunction&& get_sample_size, TGetSampleIndicesFunction&& get_sample_indices) {
		const int sample_size = std::forward<TGetSampleSizeFunction>(get_sample_size)();
		const int* sample_indices = std::forward<TGetSampleIndicesFunction>(get_sample_indices)();
		CallCUDAonUploadedFunctor(
				functor,
				[&sample_size, &sample_indices, &data](TFunctor* functor_device) {
					dim3 cuda_block_size(TBlockSize);
					dim3 cuda_grid_size(ceil_of_integer_quotient(sample_size, cuda_block_size.x));
					TraverseSampleWithoutIndex_Exact_device<TData, TFunctor> <<<cuda_grid_size, cuda_block_size>>>
							(data, sample_indices, sample_size, functor_device);
				}
		);
	}

};
template<>
class RawArrayTraversalEngine_Internal<MEMORYDEVICE_CUDA, JobCountPolicy::PADDED, CONTIGUOUS> {
	friend class RawArrayTraversalEngine<MEMORYDEVICE_CUDA>;
protected: // static functions
	template<typename TData, typename TFunctor>
	inline static void Traverse_Generic(TData* data, TFunctor& functor, const unsigned int element_count) {
		CallCUDAonUploadedFunctor(
				functor,
				[&element_count, &data](TFunctor* functor_device) {
					dim3 cuda_block_size(256);
					dim3 cuda_grid_size(ceil_of_integer_quotient(element_count, cuda_block_size.x));
					TraverseContiguous_Padded_Generic_device<TData, TFunctor> <<<cuda_grid_size, cuda_block_size>>>
							(data, element_count, functor_device);
				}
		);
	}

};
template<>
class RawArrayTraversalEngine_Internal<MEMORYDEVICE_CUDA, JobCountPolicy::PADDED, INDEX_SAMPLE> {
	friend class RawArrayTraversalEngine<MEMORYDEVICE_CUDA>;
protected: // static functions
	template<typename TData, typename TFunctor, typename TGetSampleSizeFunction, typename TGetSampleIndicesFunction, int TBlockSize = 256>
	inline static void Traverse_Generic(TData* data, TFunctor& functor,
	                                             TGetSampleSizeFunction&& get_sample_size, TGetSampleIndicesFunction&& get_sample_indices) {
		const int sample_size = std::forward<TGetSampleSizeFunction>(get_sample_size)();
		const int* sample_indices = std::forward<TGetSampleIndicesFunction>(get_sample_indices)();
		CallCUDAonUploadedFunctor(
				functor,
				[&sample_size, &sample_indices, &data](TFunctor* functor_device) {
					dim3 cuda_block_size(TBlockSize);
					dim3 cuda_grid_size(ceil_of_integer_quotient(sample_size, cuda_block_size.x));
					TraverseSample_Padded_Generic_device<TData, TFunctor> <<<cuda_grid_size, cuda_block_size>>>
							(data, sample_indices, sample_size, functor_device);
				}
		);
	}

};
} // namespace internal
} // namespace ITMLib