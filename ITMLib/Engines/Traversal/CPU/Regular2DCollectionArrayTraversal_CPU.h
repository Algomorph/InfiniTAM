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

#include "../Interface/Regular2DCollectionArrayTraversal.h"
#include "MemoryBlockTraversal_CPU.h"

namespace ITMLib {
template<>
class Regular2DCollectionArrayTraversal<MEMORYDEVICE_CPU> : public MemoryBlockTraversalEngine<MEMORYDEVICE_CPU> {
protected: // static functions

	template<typename TAppyFunction>
	inline static void Traverse2DBlock(int start_x, int start_y, int end_x, int end_y, TAppyFunction apply) {
		for (int y = start_y; y < end_y; y++) {
			for (int x = start_x; x < end_y; x++) {
				apply(x, y);
			}
		}
	}

	template<typename TData, typename TFunctor,
			typename TGetSubElementStartX,
			typename TGetSubElementEndX,
			typename TGetSubElementStartY,
			typename TGetSubElementEndY,
			int TMaxSubElementCountX = 0,
			int TMaxSubElementCountY = 0>
	inline static void
	TraverseRawSubCollections2DWithIndex_Generic(
			TData* data, const unsigned int element_count, TFunctor& functor,
			TGetSubElementStartX get_start_x,
			TGetSubElementEndX get_start_y,
			TGetSubElementStartY get_end_x,
			TGetSubElementEndY get_end_y
	) {
		Traverse_Generic<TData>(data, element_count, [&functor, &data, &get_start_x, &get_start_y, &get_end_x, &get_end_y](int i_item) {
			TData& item = data[i_item];
			Traverse2DBlock(get_start_x(item), get_start_y(item), get_end_x(item), get_end_y(item),
			                [&item, &i_item, &functor](int x, int y) {
				                functor(item, i_item, x, y);
			                });
		});
	}
	template<typename TData, typename TFunctor,
			typename TGetSubElementStartX,
			typename TGetSubElementEndX,
			typename TGetSubElementStartY,
			typename TGetSubElementEndY,
			int TMaxSubElementCountX = 0,
			int TMaxSubElementCountY = 0>
	inline static void
	TraverseRawSubCollections2DWithoutIndex_Generic(
			TData* data, const unsigned int element_count, TFunctor& functor,
			TGetSubElementStartX get_start_x,
			TGetSubElementEndX get_start_y,
			TGetSubElementStartY get_end_x,
			TGetSubElementEndY get_end_y
	) {
		Traverse_Generic<TData>(data, element_count, [&functor, &data, &get_start_x, &get_start_y, &get_end_x, &get_end_y](int i_item) {
			TData& item = data[i_item];
			Traverse2DBlock(get_start_x(item), get_start_y(item), get_end_x(item), get_end_y(item),
			                [&item, &functor](int x, int y) {
				                functor(item, x, y);
			                });
		});
	}

public: // static functions

};
}//namespace ITMLib
