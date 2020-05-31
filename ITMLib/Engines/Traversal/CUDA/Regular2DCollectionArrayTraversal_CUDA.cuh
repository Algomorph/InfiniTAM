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
#include "Regular2DCollectionArrayTraversal_CUDA_Kernels.cuh"
#include "MemoryBlockTraversal_CUDA.h"

namespace ITMLib {
template<>
class Regular2DCollectionArrayTraversal<MEMORYDEVICE_CUDA> : public MemoryBlockTraversalEngine<MEMORYDEVICE_CUDA> {
protected: // static functions

	template<typename TData, typename TFunctor,
			typename TGetSubElementStartX,
			typename TGetSubElementEndX,
			typename TGetSubElementStartY,
			typename TGetSubElementEndY,
			int TMaxSubElementCountX,
			int TMaxSubElementCountY>
	inline static void
	TraverseRawSubCollections2DWithIndex_Generic(
			TData* data, const unsigned int element_count, TFunctor& functor,
			TGetSubElementStartX get_start_x,
			TGetSubElementEndX get_start_y,
			TGetSubElementStartY get_end_x,
			TGetSubElementEndY get_end_y
	) {
		TraverseRaw_Generic(data, element_count,
		                    [&element_count, &get_start_x, &get_start_y, &get_end_x, &get_end_y]
				                    (TData* data, const Vector2i resolution, TFunctor* functor_device) {
			                    dim3 cuda_block_size(TMaxSubElementCountX, TMaxSubElementCountY);
			                    dim3 cuda_grid_size(ceil_of_integer_quotient(element_count, 4u), 4u);
			                    memoryBlockSubElement2DTraversalWithItemIndex_device<TData, TFunctor>
			                    <<< cuda_grid_size, cuda_block_size >>>
			                                        (data, element_count, functor_device, get_start_x, get_start_y, get_end_x, get_end_y);
		                    }
		);
	}

	template<typename TData, typename TFunctor,
			typename TGetSubElementStartX,
			typename TGetSubElementEndX,
			typename TGetSubElementStartY,
			typename TGetSubElementEndY,
			int TMaxSubElementCountX,
			int TMaxSubElementCountY>
	inline static void
	TraverseRawSubCollections2DWithoutIndex_Generic(
			TData* data, const unsigned int element_count, TFunctor& functor,
			TGetSubElementStartX get_start_x,
			TGetSubElementEndX get_start_y,
			TGetSubElementStartY get_end_x,
			TGetSubElementEndY get_end_y
	) {
		TraverseRaw_Generic(data, element_count,
		                    [&element_count, &get_start_x, &get_start_y, &get_end_x, &get_end_y]
				                    (TData* data, const Vector2i resolution, TFunctor* functor_device) {
			                    dim3 cuda_block_size(TMaxSubElementCountX, TMaxSubElementCountY);
			                    dim3 cuda_grid_size(ceil_of_integer_quotient(element_count, 4u), 4u);
			                    memoryBlockSubElement2DTraversalWithoutItemIndex_device<TData, TFunctor>
			                    <<< cuda_grid_size, cuda_block_size >>>
			                                        (data, element_count, functor_device, get_start_x, get_start_y, get_end_x, get_end_y);
		                    }
		);
	}

public: // static functions

};
}//namespace ITMLib