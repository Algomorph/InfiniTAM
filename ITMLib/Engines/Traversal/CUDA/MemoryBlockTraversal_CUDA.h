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
#include "../Interface/MemoryBlockTraversal.h"
#include "../../../../ORUtils/MemoryBlock.h"
#include "MemoryBlockTraversal_CUDA_Kernels.h"

namespace ITMLib {

template<>
class MemoryBlockTraversalEngine<MEMORYDEVICE_CUDA> {
private: // static functions
	template<typename TData, typename TFunctor>
	inline static void TraverseRaw_Generic(TData* data, const unsigned int element_count, TFunctor& functor){
		dim3 cuda_block_size(256, 1);
		dim3 cuda_grid_size((int) ceil((float) element_count / (float) cuda_block_size.x));

		TFunctor* functor_device;

		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		memoryBlockTraversalWithItemIndex_device < TData, TFunctor > <<< cuda_grid_size, cuda_block_size >>>(data, element_count, functor_device);
		ORcudaKernelCheck;

		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TData, typename TMemoryBlock, typename TFunctor>
	inline static void Traverse_Generic(TMemoryBlock& memory_block, unsigned int element_count, TFunctor& functor){
		TData* data = memory_block.GetData(MEMORYDEVICE_CUDA);
		assert(element_count < memory_block.size());
		TraverseRaw_Generic(data, element_count, functor);
	}

public: // static functions

	template<typename T, typename TFunctor>
	inline static void
	TraverseRaw(T* data, const unsigned int element_count, TFunctor& functor){
		TraverseRaw_Generic<T, TFunctor>(data, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	TraverseRaw(const T* data, const unsigned int element_count, TFunctor& functor){
		TraverseRaw_Generic<const T, TFunctor>(data, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	Traverse(ORUtils::MemoryBlock<T>& memory_block, unsigned int element_count, TFunctor& functor){
		Traverse_Generic<T, ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	Traverse(const ORUtils::MemoryBlock<T>& memory_block, unsigned int element_count, TFunctor& functor){
		Traverse_Generic<const T, const ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}

};

} // namespace ITMLib