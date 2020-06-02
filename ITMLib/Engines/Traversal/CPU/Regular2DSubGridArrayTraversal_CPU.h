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
#include "RawArrayTraversal_CPU.h"

namespace ITMLib {
template<>
class Regular2DSubGridArrayTraversal<MEMORYDEVICE_CPU> : private RawArrayTraversalEngine<MEMORYDEVICE_CPU> {
protected: // static functions

	template<typename TAppyFunction>
	inline static void Traverse2DBlock(int start_x, int end_x, int start_y, int end_y, TAppyFunction&& apply) {
		for (int y = start_y; y < end_y; y++) {
			for (int x = start_x; x < end_x; x++) {
				apply(x, y);
			}
		}
	}

	template<typename TData, typename TApplyFunction, typename TGetSubGridBoundsFunction>
	inline static void
	TraverseRaw_Generic(TData* data, const unsigned int element_count, TApplyFunction&& apply,
	                    TGetSubGridBoundsFunction&& get_sub_element_bounds){
		RawArrayTraversalEngine<MEMORYDEVICE_CPU>::TraverseRaw_Generic(
				element_count,
				[&data, &get_sub_element_bounds, &apply](int i_item) {
					TData& item = data[i_item];
					int start_x, end_x, start_y, end_y;
					get_sub_element_bounds(item, start_x, end_x, start_y, end_y);
					Traverse2DBlock(start_x, end_x, start_y, end_y, [&i_item, &apply](int x, int y) { apply(i_item, x, y); });
				});
	};

	template<typename TData, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	TraverseRawWithIndex_Generic(
			TData* data, const unsigned int element_count, TFunctor& functor,
			TGetSubGridBoundsFunction&& get_sub_element_bounds
	) {
		TraverseRaw_Generic<TData>(data, element_count,
		                           [&data, &functor](const int i_item, const int x, const int y) { functor(data[i_item], i_item, x, y); },
		                           get_sub_element_bounds);
	}

	template<typename TData, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	TraverseRawWithoutIndex_Generic(
			TData* data, const unsigned int element_count, TFunctor& functor,
			TGetSubGridBoundsFunction&& get_sub_element_bounds
	) {
		TraverseRaw_Generic<TData>(data, element_count,
		                           [&data, &functor](const int i_item, const int x, const int y) { functor(data[i_item], x, y); },
		                           get_sub_element_bounds);
	}

	template<typename TData, typename TMemoryBlock, typename TFunctor,
			typename TGetSubGridBoundsFunction>
	inline static void
	TraverseWithIndex_Generic(TMemoryBlock& memory_block, unsigned int element_count, TFunctor& functor,
	                          TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TData* data = memory_block.GetData(MEMORYDEVICE_CPU);
		assert(element_count <= memory_block.size());
		TraverseRawWithIndex_Generic(data, element_count, functor, get_sub_element_bounds);
	}

	template<typename TData, typename TMemoryBlock, typename TFunctor,
			typename TGetSubGridBoundsFunction>
	inline static void
	TraverseWithoutIndex_Generic(TMemoryBlock& memory_block, unsigned int element_count, TFunctor& functor,
	                             TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TData* data = memory_block.GetData(MEMORYDEVICE_CPU);
		assert(element_count <= memory_block.size());
		TraverseRawWithoutIndex_Generic(data, element_count, functor, get_sub_element_bounds);
	}

public: // static functions
	template<int TMaxSubGridX, int TMaxSubGridY, typename T, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	Traverse(ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor,
	         TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TraverseWithoutIndex_Generic<T, ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor, get_sub_element_bounds);
	}

	template<int TMaxSubGridX, int TMaxSubGridY, typename T, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	Traverse(const ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor,
	         TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TraverseWithoutIndex_Generic<const T, const ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor, get_sub_element_bounds);
	}

	template<int TMaxSubGridX, int TMaxSubGridY, typename T, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	TraverseWithIndex(ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor,
	                  TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TraverseWithIndex_Generic<T, ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor, get_sub_element_bounds);
	}

	template<int TMaxSubGridX, int TMaxSubGridY, typename T, typename TFunctor, typename TGetSubGridBoundsFunction>
	inline static void
	TraverseWithIndex(const ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor,
	                  TGetSubGridBoundsFunction&& get_sub_element_bounds) {
		TraverseWithIndex_Generic<const T, const ORUtils::MemoryBlock<T>, TFunctor>(memory_block, element_count, functor, get_sub_element_bounds);
	}
};
}//namespace ITMLib
