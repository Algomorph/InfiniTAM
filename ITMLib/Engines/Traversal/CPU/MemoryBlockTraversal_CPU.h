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

template<>
class MemoryBlockTraversalEngine<MEMORYDEVICE_CPU> : private RawArrayTraversalEngine<MEMORYDEVICE_CPU> {
protected: // static functions
	template<typename TData, typename TMemoryBlock, typename TFunctor>
	inline static void
	TraverseWithIndex_Generic(TMemoryBlock& memory_block, unsigned int element_count, TFunctor& functor){
		TData* data = memory_block.GetData(MEMORYDEVICE_CPU);
		assert(element_count <= memory_block.size());
		RawArrayTraversalEngine<MEMORYDEVICE_CPU>::TraverseRawWithIndex_Generic(data, element_count, functor);
	}
	template<typename TData, typename TMemoryBlock, typename TFunctor>
	inline static void
	TraverseWithoutIndex_Generic(TMemoryBlock& memory_block, unsigned int element_count, TFunctor& functor){
		TData* data = memory_block.GetData(MEMORYDEVICE_CPU);
		assert(element_count <= memory_block.size());
		RawArrayTraversalEngine<MEMORYDEVICE_CPU>::TraverseRawWithoutIndex_Generic(data, element_count, functor);
	}

public: // static functions

	template<typename T, typename TFunctor>
	inline static void
	Traverse(ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor){
		TraverseWithoutIndex_Generic<T, ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	Traverse(const ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor){
		TraverseWithoutIndex_Generic<const T, const ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	TraverseWithIndex(ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor){
		TraverseWithIndex_Generic<T, ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	TraverseWithIndex(const ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor){
		TraverseWithIndex_Generic<const T, const ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}

};

} // namespace ITMLib