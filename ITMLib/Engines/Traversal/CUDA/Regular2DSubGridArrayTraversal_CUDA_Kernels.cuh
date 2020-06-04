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

template<typename TData,
		typename TGetSubElementBoundsFunction, typename TApplyFunction>
__device__ void SubElement2DTraversal_Generic_device(
		TData* data, const unsigned int element_count,
		TGetSubElementBoundsFunction&& get_sub_element_bounds, TApplyFunction&& apply_function) {
	unsigned int i_item = blockIdx.x * gridDim.y + blockIdx.y;
	if (i_item >= element_count) return;
	TData& item = data[i_item];
	int x_local = threadIdx.x;
	int y_local = threadIdx.y;
	int start_x, end_x, start_y, end_y;
	get_sub_element_bounds(item, start_x, end_x, start_y, end_y);
	int y = start_y + y_local;

	if (y >= end_y) return;
	int x = start_x + x_local;
	if (x >= end_x) return;

	apply_function(i_item, x, y);
}

template<typename TData, typename TFunctor,
		typename TGetSubElementBoundsFunction>
__global__ void SubGrid2DTraversalWithItemIndex_device(
		TData* data, const unsigned int element_count, TFunctor* functor_device,
		TGetSubElementBoundsFunction get_sub_element_bounds) {
	SubElement2DTraversal_Generic_device(
			data, element_count, get_sub_element_bounds,
			[&functor_device, &data](int i_item, int x, int y) {
				(*functor_device)(data[i_item], i_item, x, y);
			}
	);
}

template<typename TData, typename TFunctor,
		typename TGetSubElementBoundsFunction>
__global__ void SubGrid2DTraversalWithoutItemIndex_device(
		TData* data, const unsigned int element_count, TFunctor* functor_device,
		TGetSubElementBoundsFunction get_sub_element_bounds) {
	SubElement2DTraversal_Generic_device(
			data, element_count, get_sub_element_bounds,
			[&functor_device, &data](int i_item, int x, int y) {
				(*functor_device)(data[i_item], x, y);
			}
	);
}

} // end anonymous namespace (CUDA Kernels)