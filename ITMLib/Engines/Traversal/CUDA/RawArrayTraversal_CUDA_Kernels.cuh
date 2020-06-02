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
#include "../../../Utils/Math.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../Shared/JobCountPolicy.h"

namespace {
// CUDA global kernels
template<typename TApplyFunction>
__device__ inline void RawArrayTraversal_Exact_Generic_device(const unsigned int element_count, TApplyFunction&& apply) {
	unsigned int i_item = threadIdx.x + blockIdx.x * blockDim.x;
	if (i_item >= element_count) return;
	apply(i_item);
}

template<typename TData, typename TFunctor>
__global__ void RawArrayTraversalWithoutItemIndex_Exact_device(TData* data, const unsigned int element_count, TFunctor* functor_device) {
	RawArrayTraversal_Exact_Generic_device(element_count, [&data, &functor_device](const int i_item) { (*functor_device)(data[i_item]); });
}

template<typename TData, typename TFunctor>
__global__ void RawArrayTraversalWithItemIndex_Exact_device(TData* data, const unsigned int element_count, TFunctor* functor_device) {
	RawArrayTraversal_Exact_Generic_device(element_count, [&data, &functor_device](const int i_item) { (*functor_device)(data[i_item], i_item); });
}

template<typename TApplyInRangeFunction, typename TApplyInPaddingFunction>
__device__ inline void RawArrayTraversal_Padded_Generic_device(const unsigned int element_count,
                                                               TApplyInRangeFunction&& apply_in_range,
                                                               TApplyInPaddingFunction&& apply_in_padding) {
	unsigned int i_item = threadIdx.x + blockIdx.x * blockDim.x;
	if (i_item >= element_count) {
		apply_in_range(i_item);
	} else {
		apply_in_padding(i_item);
	}
}

template<typename TData, typename TFunctor>
__global__ void RawArrayTraversalWithoutItemIndex_Padded_device(TData* data, const unsigned int element_count, TFunctor* functor_device) {
	RawArrayTraversal_Padded_Generic_device(
			element_count,
			[&data, &functor_device](const int i_item) { (*functor_device)(data[i_item]); },
			[&functor_device](const int i_item) { functor_device->padding_job(); }
	);
}

template<typename TData, typename TFunctor>
__global__ void RawArrayTraversalWithItemIndex_Padded_device(TData* data, const unsigned int element_count, TFunctor* functor_device) {
	RawArrayTraversal_Padded_Generic_device(
			element_count,
			[&data, &functor_device](const int i_item) { (*functor_device)(data[i_item], i_item); },
			[&functor_device](const int i_item) { functor_device->padding_job(i_item); }
	);
}

} // end anonymous namespace: CUDA global kernels