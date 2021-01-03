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

#include "../../../Utils/Math.h"

namespace {

template<typename TApplyFunction>
__device__ inline static void TraverseTwoArrays_Generic_device(const unsigned int element_count, TApplyFunction&& apply) {
	unsigned int i_item = threadIdx.x + blockIdx.x * blockDim.x;
	if (i_item >= element_count) return;
	apply(i_item);
}

template<typename TData1, typename TData2, typename TFunctor>
__global__ static void
TraverseTwoArraysWithoutItemIndex_device(TData1* data_1, TData2* data_2, const unsigned int element_count, TFunctor* functor_device) {
	TraverseTwoArrays_Generic_device(element_count,
	                        [&data_1, &data_2, &functor_device](const int i_item) { (*functor_device)(data_1[i_item], data_2[i_item]); });
}

template<typename TData1, typename TData2, typename TFunctor>
__global__ static void TraverseTwoArraysWithIndex_device(TData1* data_1, TData2* data_2, const unsigned int element_count, TFunctor* functor_device) {
	TraverseTwoArrays_Generic_device(element_count,
	                        [&data_1, &data_2, &functor_device](const int i_item) { (*functor_device)(data_1[i_item], data_2[i_item], i_item); });
}

template<typename TImageElement1, typename TImageElement2, typename TFunctor>
__global__ static void TraverseWithPixelCoordinate_device(TImageElement1* image_data1,
                                                          TImageElement2* image_data2,
                                                          const Vector2i resolution,
                                                          TFunctor* functor_device) {
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
	if (x >= resolution.x || y >= resolution.y) return;
	int index = x + y * resolution.x;
	(*functor_device)(image_data1[index], image_data2[index], x, y);
}

// // CUDA global kernels
// template <typename TImage1Element, typename TImage2Element, typename TFunctor >
// __global__ void twoImageTraversalWithPosition_device (const TImage1Element* image1_data, const TImage2Element* image2_data, const Vector2i resolution, TFunctor* functor_device){
// 	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
// 	if (x > resolution.x - 1 || y > resolution.y - 1) return;
// 	(*functor_device)(image1_data[x + y * resolution.x], image2_data[x + y * resolution.x], x, y);
// }

} // end anonymous namespace (CUDA global kernels)
