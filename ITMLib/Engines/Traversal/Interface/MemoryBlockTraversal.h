//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/9/20.
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
#include "../../../../ORUtils/MemoryDeviceType.h"
#include "../../../../ORUtils/MemoryBlock.h"
#include "../Shared/ContiguousCollectionTraversal_Shared.h"

namespace ITMLib {

namespace internal{
	template<MemoryDeviceType TDeviceType>
	class MemoryBlockTraversalEngine_Internal;
} // namespace internal
template<MemoryDeviceType TMemoryDeviceType>
class MemoryBlockTraversalEngine : private internal::MemoryBlockTraversalEngine_Internal<TMemoryDeviceType>{
private: // static functions

public: // static functions
	template<typename T, typename TFunctor>
	inline static void
	Traverse(ORUtils::MemoryBlock<T>& memory_block, TFunctor& functor, unsigned int element_count = static_cast<unsigned int>(SpecialValue::USE_MAX)){
		HandleDefaultElementCount(element_count, memory_block);
		internal::MemoryBlockTraversalEngine_Internal<TMemoryDeviceType>::template TraverseWithoutIndex_Generic<T, ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	Traverse(const ORUtils::MemoryBlock<T>& memory_block, TFunctor& functor, const unsigned int element_count = static_cast<unsigned int>(SpecialValue::USE_MAX)){
		HandleDefaultElementCount(element_count, memory_block);
		internal::MemoryBlockTraversalEngine_Internal<TMemoryDeviceType>::template TraverseWithoutIndex_Generic<const T, const ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	TraverseWithIndex(ORUtils::MemoryBlock<T>& memory_block, TFunctor& functor, const unsigned int element_count = static_cast<unsigned int>(SpecialValue::USE_MAX)){
		HandleDefaultElementCount(element_count, memory_block);
		internal::MemoryBlockTraversalEngine_Internal<TMemoryDeviceType>::template TraverseWithIndex_Generic<T, ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	TraverseWithIndex(const ORUtils::MemoryBlock<T>& memory_block, TFunctor& functor, const unsigned int element_count = static_cast<unsigned int>(SpecialValue::USE_MAX)){
		HandleDefaultElementCount(element_count, memory_block);
		internal::MemoryBlockTraversalEngine_Internal<TMemoryDeviceType>::template TraverseWithIndex_Generic<const T, const ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}

};
} // namespace ITMLib