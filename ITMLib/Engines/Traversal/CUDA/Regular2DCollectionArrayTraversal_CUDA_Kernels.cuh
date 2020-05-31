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

namespace {
// CUDA global kernels

template<typename TData, typename TFunctor,
		typename TGetSubElementStartX,
		typename TGetSubElementEndX,
		typename TGetSubElementStartY,
		typename TGetSubElementEndY>
__global__ void memoryBlockSubElement2DTraversalWithItemIndex_device(
		TData* data, const unsigned int element_count, TFunctor* functor_device,
		TGetSubElementStartX get_start_x,
		TGetSubElementEndX get_start_y,
		TGetSubElementStartY get_end_x,
		TGetSubElementEndY get_end_y) {
	unsigned int i_item = blockIdx.x * gridDim.x + blockIdx.y;
	if (i_item >= element_count) return;
	TData& item = data[i_item];
	int x = threadIdx.x;
	int y = threadIdx.y;
	int y_sub_element = get_start_y(item) + y;
	if (y_sub_element > get_end_y(item)) return;
	int x_sub_element = get_start_x(item) + x;
	if (x_sub_element > get_end_x(item)) return;
	(*functor_device)(data[i_item], i_item, x, y);
}

template<typename TData, typename TFunctor,
		typename TGetSubElementStartX,
		typename TGetSubElementEndX,
		typename TGetSubElementStartY,
		typename TGetSubElementEndY>
__global__ void memoryBlockSubElement2DTraversalWithoutItemIndex_device(
		TData* data, const unsigned int element_count, TFunctor* functor_device,
		TGetSubElementStartX get_start_x,
		TGetSubElementEndX get_start_y,
		TGetSubElementStartY get_end_x,
		TGetSubElementEndY get_end_y) {
	unsigned int i_item = blockIdx.x * gridDim.x + blockIdx.y;
	if (i_item >= element_count) return;
	TData& item = data[i_item];
	int x = threadIdx.x;
	int y = threadIdx.y;
	int y_sub_element = get_start_y(item) + y;
	if (y_sub_element > get_end_y(item)) return;
	int x_sub_element = get_start_x(item) + x;
	if (x_sub_element > get_end_x(item)) return;
	(*functor_device)(data[i_item], x, y);
}
} // end anonymous namespace (CUDA Kernels)