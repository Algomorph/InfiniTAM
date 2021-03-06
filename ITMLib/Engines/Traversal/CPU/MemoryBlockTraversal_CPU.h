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
#include "../Interface/RawArrayTraversal.h"
#include "../../../../ORUtils/MemoryBlock.h"
#include "RawArrayTraversal_CPU.h"

namespace ITMLib {
namespace internal{
template<>
class MemoryBlockTraversalEngine_Internal<MEMORYDEVICE_CPU> : private internal::RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, EXACT, CONTIGUOUS> {
protected: // static functions
	template<typename TData, typename TMemoryBlock, typename TFunctor>
	inline static void TraverseWithIndex_Generic(TMemoryBlock& memory_block, TFunctor& functor, unsigned int element_count){
		TData* data = memory_block.GetData(MEMORYDEVICE_CPU);
		assert(element_count <= memory_block.size());
		RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, EXACT, CONTIGUOUS>::Traverse_Generic(data, functor, element_count);
	}

	template<typename TData, typename TMemoryBlock, typename TFunctor>
	inline static void TraverseWithoutIndex_Generic(TMemoryBlock& memory_block, TFunctor& functor, unsigned int element_count){
		TData* data = memory_block.GetData(MEMORYDEVICE_CPU);
		assert(element_count <= memory_block.size());
		RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, EXACT, CONTIGUOUS>::TraverseWithoutIndex_Generic(data, functor, element_count);
	}
};
} // namespace internal
} // namespace ITMLib