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

namespace ITMLib {

template<>
class RawArrayTraversalEngine<MEMORYDEVICE_CPU> {
protected: // static functions
	template<typename TApplyFunction>
	inline static void
	TraverseRaw_Generic(const unsigned int element_count, TApplyFunction&& apply_function){
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(apply_function)
#endif
		for (int i_item = 0; i_item < element_count; i_item++){
			apply_function(i_item);
		}
	}

	template<typename TData, typename TFunctor>
	inline static void
	TraverseRawWithIndex_Generic(TData* data, const unsigned int element_count, TFunctor& functor){
		TraverseRaw_Generic(element_count, [&functor, &data](int i_item) { functor(data[i_item], i_item); });
	}
	
	template<typename TData, typename TFunctor>
	inline static void
	TraverseRawWithoutIndex_Generic(TData* data, const unsigned int element_count, TFunctor& functor){
		TraverseRaw_Generic(element_count, [&functor, &data](int i_item) { functor(data[i_item]); });
	}

public: // static functions
	
	template<typename T, typename TFunctor>
	inline static void
	TraverseRaw(T* data, const unsigned int element_count, TFunctor& functor){
		TraverseRawWithoutIndex_Generic<T, TFunctor>(data, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	TraverseRaw(const T* data, const unsigned int element_count, TFunctor& functor){
		TraverseRawWithoutIndex_Generic<const T, TFunctor>(data, element_count, functor);
	}
	
	template<typename T, typename TFunctor>
	inline static void
	TraverseWithIndexRaw(T* data, const unsigned int element_count, TFunctor& functor){
		TraverseRawWithIndex_Generic<T, TFunctor>(data, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	TraverseWithIndexRaw(const T* data, const unsigned int element_count, TFunctor& functor){
		TraverseRawWithIndex_Generic<const T, TFunctor>(data, element_count, functor);
	}

};

} // namespace ITMLib