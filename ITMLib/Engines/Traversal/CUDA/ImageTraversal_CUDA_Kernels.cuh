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
#include "../Shared/TraversalMethod.h"
#include "../Shared/JobCountPolicy.h"

namespace {
// CUDA global kernels

template<typename TImageElement, typename TFunctor>
__global__ static void TraverseWithPosition_device(TImageElement* image_data, const Vector2i resolution, TFunctor* functor_device) {
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
	if (x >= resolution.x || y >= resolution.y) return;
	(*functor_device)(image_data[x + y * resolution.x], x, y);
}

template<typename TFunctor>
__global__ static void TraversePositionOnly_device(const Vector2i resolution, TFunctor* functor_device) {
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
	if (x >= resolution.x || y >= resolution.y) return;
	(*functor_device)(x + y * resolution.x, x, y);
}

template<typename TLambda>
__global__ static void TraversePositionOnly_Lambda_device(const Vector2i resolution, TLambda lambda) {
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
	if (x >= resolution.x || y >= resolution.y) return;
	lambda(x + y * resolution.x, x, y);
}

template<typename TImageElement, typename TFunctor>
__global__ static void TraverseWithPosition_device(TImageElement* image_data, const int* sample_pixel_indices, const int sample_size,
                                                   const int image_width, TFunctor* functor_device) {
	const int i_index = threadIdx.x + blockIdx.x * blockDim.x;
	if (i_index >= sample_size) return;
	const int pixel_index = sample_pixel_indices[i_index];
	const int y = pixel_index / image_width;
	const int x = pixel_index % image_width;
	(*functor_device)(image_data[pixel_index], x, y);
}

template<typename TFunctor>
__global__ static void TraversePositionOnly_device(const int* sample_pixel_indices, const int sample_size, const int image_width,
                                                   TFunctor* functor_device) {
	const int i_index = threadIdx.x + blockIdx.x * blockDim.x;
	if (i_index >= sample_size) return;
	const int pixel_index = sample_pixel_indices[i_index];
	const int y = pixel_index / image_width;
	const int x = pixel_index % image_width;
	(*functor_device)(pixel_index, x, y);
}

} // end anonymous namespace (CUDA global kernels)
