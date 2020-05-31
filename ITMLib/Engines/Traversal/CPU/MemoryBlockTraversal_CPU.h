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

namespace ITMLib {

template<>
class MemoryBlockTraversalEngine<MEMORYDEVICE_CPU> {
protected: // static functions
	template<typename TData, typename TFunctor,
			typename TGetSubElementStartX,
			typename TGetSubElementEndX,
			typename TGetSubElementStartY,
			typename TGetSubElementEndY,
	        int TMaxSubElementCountX,
	        int TMaxSubElementCountY>
	inline static void
	TraverseRawSubCollections2D_Generic(
			TData* data, const unsigned int element_count, TFunctor& functor,
			TGetSubElementStartX get_start_x,
			TGetSubElementEndX get_start_y,
			TGetSubElementStartY get_end_x,
			TGetSubElementEndY get_end_y
	){
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(data, functor)
#endif
		for (int i_item = 0; i_item < element_count; i_item++){
			TData& item = data[i_item];
			for(int y = get_start_y(item); y < get_end_y(item); y++){
				for(int x = get_start_x(item); x < get_end_x(item); x++){
					functor(item, i_item, x, y);
				}
			}
		}
	}

	template<typename TData, typename TFunctor>
	inline static void
	TraverseRaw_Generic(TData* data, const unsigned int element_count, TFunctor& functor){
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(data, functor)
#endif
		for (int i_item = 0; i_item < element_count; i_item++){
			functor(data[i_item], i_item);
		}
	}

	template<typename TData, typename TMemoryBlock, typename TFunctor>
	inline static void
	Traverse_Generic(TMemoryBlock& memory_block, const unsigned int element_count, TFunctor& functor){
		TData* data = memory_block.GetData(MEMORYDEVICE_CPU);
		assert(element_count <= memory_block.size());
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
	Traverse(ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor){
		Traverse_Generic<T, ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}

	template<typename T, typename TFunctor>
	inline static void
	Traverse(const ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor){
		Traverse_Generic<const T, const ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor);
	}
};

} // namespace ITMLib