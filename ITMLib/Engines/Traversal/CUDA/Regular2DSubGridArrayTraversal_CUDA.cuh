//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/31/20.
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

#include "../Interface/Regular2DSubGridArrayTraversal.h"
#include "RawArrayTraversal_CUDA.cuh"
#include "../Shared/CudaCallWrappers.cuh"
#include "Regular2DSubGridArrayTraversal_CUDA_Kernels.cuh"

namespace ITMLib {
template<>
class Regular2DSubGridArrayTraversal<MEMORYDEVICE_CUDA>{
protected: // static functions
	template<int TMaxSubGridX,
			int TMaxSubGridY,
			typename TData, typename TFunctor,
			typename TGetSubGridBoundsFunction>
	inline static void
	TraverseRawWithIndex_Generic(
			TData* data, const unsigned int element_count, TFunctor& functor,
			TGetSubGridBoundsFunction&& get_sub_element_bounds
	) {
		internal::CallCUDAonUploadedFunctor(
				functor,
				[&data, &element_count, &get_sub_element_bounds](TFunctor* functor_device) {
					dim3 cuda_block_size(TMaxSubGridX, TMaxSubGridY);
					dim3 cuda_grid_size(ceil_of_integer_quotient(element_count, 4u), 4u);
					SubGrid2DTraversalWithItemIndex_device<TData, TFunctor>
					<<< cuda_grid_size, cuda_block_size >>>
							(data, element_count, functor_device, get_sub_element_bounds);
				}
		);
	}

	template<int TMaxSubGridX, int TMaxSubGridY, typename TData, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	TraverseRawWithoutIndex_Generic(
			TData* data, const unsigned int element_count, TFunctor& functor,
			TGetSubGridBoundsFunction&& get_sub_element_bounds
	) {
		internal::CallCUDAonUploadedFunctor(
				functor,
				[&data, &element_count, &get_sub_element_bounds](TFunctor* functor_device) {
					dim3 cuda_block_size(TMaxSubGridX, TMaxSubGridY);
					dim3 cuda_grid_size(ceil_of_integer_quotient(element_count, 4u), 4u);
					SubGrid2DTraversalWithoutItemIndex_device<TData, TFunctor>
					<<< cuda_grid_size, cuda_block_size >>>
							(data, element_count, functor_device, get_sub_element_bounds);
				}
		);
	}

	template<int TMaxSubGridX, int TMaxSubGridY, typename TData, typename TMemoryBlock, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	TraverseWithIndex_Generic(TMemoryBlock& memory_block, const unsigned int element_count, TFunctor& functor,
	                          TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TData* data = memory_block.GetData(MEMORYDEVICE_CUDA);
		assert(element_count <= memory_block.size());
		TraverseRawWithIndex_Generic<TMaxSubGridX, TMaxSubGridY>(data, element_count, functor, get_sub_element_bounds);
	}

	template<int TMaxSubGridX, int TMaxSubGridY, typename TData, typename TMemoryBlock, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	TraverseWithoutIndex_Generic(TMemoryBlock& memory_block, const unsigned int element_count, TFunctor& functor,
	                             TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TData* data = memory_block.GetData(MEMORYDEVICE_CUDA);
		assert(element_count <= memory_block.size());
		TraverseRawWithoutIndex_Generic<TMaxSubGridX, TMaxSubGridY>(data, element_count, functor, get_sub_element_bounds);
	}

public: // static functions
	template<int TMaxSubGridX, int TMaxSubGridY, typename T, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	Traverse(ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor,
	         TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TraverseWithoutIndex_Generic<TMaxSubGridX, TMaxSubGridY, T, ORUtils::MemoryBlock<T>>(
				memory_block, element_count, functor, get_sub_element_bounds);
	}

	template<int TMaxSubGridX, int TMaxSubGridY, typename T, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	Traverse(const ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor,
	         TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TraverseWithoutIndex_Generic<TMaxSubGridX, TMaxSubGridY, const T, const ORUtils::MemoryBlock<T>>(
				memory_block, element_count, functor, get_sub_element_bounds);
	}

	template<int TMaxSubGridX, int TMaxSubGridY, typename T, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	TraverseWithIndex(ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor,
	                  TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TraverseWithIndex_Generic<TMaxSubGridX, TMaxSubGridY, T, ORUtils::MemoryBlock<T>>(
				memory_block, element_count, functor, get_sub_element_bounds);
	}

	template<int TMaxSubGridX, int TMaxSubGridY, typename T, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	TraverseWithIndex(const ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor,
	                  TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TraverseWithIndex_Generic<TMaxSubGridX, TMaxSubGridY, const T, const ORUtils::MemoryBlock<T>>(
				memory_block, element_count, functor, get_sub_element_bounds);
	}
};
}//namespace ITMLib