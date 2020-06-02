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

namespace ITMLib {

template<>
class RawArrayTraversalEngine<MEMORYDEVICE_CUDA> {
protected: // static functions
	template<typename TFunctor, typename TCudaCall>
	inline static void TraverseRaw_Generic(TFunctor& functor, TCudaCall&& cuda_call) {
		TFunctor* functor_device;

		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		cuda_call(functor_device);
		ORcudaKernelCheck;

		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TData, typename TFunctor>
	inline static void TraverseRawWithIndex_Generic(TData* data, const unsigned int element_count, TFunctor& functor) {
		TraverseRaw_Generic(
				functor,
				[&element_count, &data](TFunctor* functor_device) {
					dim3 cuda_block_size(256);
					dim3 cuda_grid_size(ceil_of_integer_quotient(element_count, cuda_block_size.x));
					RawArrayTraversalWithItemIndex_Exact_device<TData, TFunctor> <<<cuda_grid_size, cuda_block_size>>>
							(data, element_count, functor_device);
				}
		);
	}

	template<typename TData, typename TFunctor>
	inline static void TraverseRawWithoutIndex_Generic(TData* data, const unsigned int element_count, TFunctor& functor) {
		TraverseRaw_Generic(
				functor,
				[&element_count, &data](TFunctor* functor_device) {
					dim3 cuda_block_size(256);
					dim3 cuda_grid_size(ceil_of_integer_quotient(element_count, cuda_block_size.x));
					RawArrayTraversalWithoutItemIndex_Exact_device<TData, TFunctor> <<<cuda_grid_size, cuda_block_size>>>
							(data, element_count, functor_device);
				}
		);
	}

public: // static functions
	template<typename T, typename TFunctor>
	inline static void
	TraverseRaw(T* data, const unsigned int element_count, TFunctor& functor) {
		TraverseRawWithoutIndex_Generic<T, TFunctor>(data, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	TraverseRaw(const T* data, const unsigned int element_count, TFunctor& functor) {
		TraverseRawWithoutIndex_Generic<const T, TFunctor>(data, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	TraverseWithIndexRaw(T* data, const unsigned int element_count, TFunctor& functor) {
		TraverseRawWithIndex_Generic<T, TFunctor>(data, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	TraverseWithIndexRaw(const T* data, const unsigned int element_count, TFunctor& functor) {
		TraverseRawWithIndex_Generic<const T, TFunctor>(data, element_count, functor);
	}
};

} // namespace ITMLib