//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/22/20.
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

#include "../Math.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib {

namespace {

template<typename TElement>
struct RawArrayComparisonData {
	bool mismatch_found;
	int i_mismatched_elements;
	TElement mismatched_element_l;
	TElement mismatched_element_r;
};

template<typename TElement, typename TComparisonFunction>
__global__ void TwoRawArrayItemGenericComparison(RawArrayComparisonData<TElement>* output_data,
                                                       const TElement* l, const TElement* r,
                                                       const int element_count,
                                                       TComparisonFunction compare_elements) {
	unsigned int i_element = threadIdx.x + blockIdx.x * blockDim.x;
	if (i_element > element_count || output_data->mismatch_found) return;
	if (!compare_elements(l[i_element], r[i_element])) {
		output_data->mismatch_found = true;
		output_data->i_mismatched_elements = i_element;
		output_data->mismatched_element_l = l[i_element];
		output_data->mismatched_element_r = r[i_element];
	}
}

} // end anonymous namespace (static CUDA kernels isolated to each compile uint that includes the header)

namespace internal {

template<typename TElement, typename TComparisonFunction, typename TReportMismatchFunction>
bool CompareRawArrays_Generic_CUDA(const TElement* left, const TElement* right, const int element_count,
                                   TComparisonFunction&& compare_elements, TReportMismatchFunction&& report_mismatch) {
	assert(element_count > 0);
	dim3 cuda_block_dimensions(256);
	dim3 cuda_grid_dimensions(ceil_of_integer_quotient((unsigned) element_count, cuda_block_dimensions.x));
	ORUtils::MemoryBlock<RawArrayComparisonData<TElement>> raw_array_comparison_data(1, true, true);
	raw_array_comparison_data.GetData(MEMORYDEVICE_CPU)->mismatch_found = false;
	raw_array_comparison_data.UpdateDeviceFromHost();

	TwoRawArrayItemGenericComparison <<< cuda_grid_dimensions, cuda_block_dimensions >>>
			(raw_array_comparison_data.GetData(MEMORYDEVICE_CUDA), left, right, element_count, compare_elements);

	ORcudaKernelCheck;
	raw_array_comparison_data.UpdateHostFromDevice();
	RawArrayComparisonData<TElement>& data_CPU = *raw_array_comparison_data.GetData(MEMORYDEVICE_CPU);
	bool mismatch_found = data_CPU.mismatch_found;
	if (mismatch_found) {
		std::forward<TReportMismatchFunction>(report_mismatch)(data_CPU.mismatched_element_l, data_CPU.mismatched_element_r,
		                                                       data_CPU.i_mismatched_elements);
	}
	return !mismatch_found;
}

} // end namespace internal

} // namespace ITMLib