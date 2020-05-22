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

#include "RawMemoryArrayComparison.h"

namespace ITMLib {

namespace {


//template<typename TElement>
//__global__ void TwoRawMemoryArrayItemsAlmostEqual(bool* tolerance_exceeded,
//                                                  const TElement* l, const TElement* r,
//                                                  const int element_count, const float absolute_tolerance) {
//	unsigned int i_element = threadIdx.x + blockIdx.x * blockDim.x;
//	if (i_element > element_count || *tolerance_exceeded) return;
//	if (!AlmostEqual(l[i_element], r[i_element], absolute_tolerance)) {
//		*tolerance_exceeded = true;
//	}
//}

template<typename TElement, typename TComparisonFunction>
__global__ void TwoRawMemoryArrayItemGenericComparison(bool* tolerance_exceeded,
                                                       const TElement* l, const TElement* r,
                                                       const int element_count,
                                                       TComparisonFunction&& compare_elements) {
	unsigned int i_element = threadIdx.x + blockIdx.x * blockDim.x;
	if (i_element > element_count || *tolerance_exceeded) return;
	//AlmostEqual(l[i_element], r[i_element], absolute_tolerance)
	if (!compare_elements(l[i_element], r[i_element])) {
		*tolerance_exceeded = true;
	}
}

} // end anonymous namespace (static CUDA kernels isolated to each compile uint that includes the header)

namespace internal {

template<typename TElement, typename TComparisonFunction>
bool CompareRawMemoryArrays_Generic_CUDA(const TElement* l, const TElement* r, const int element_count,
                                         TComparisonFunction&& compare_elements) {
	assert(element_count > 0);
	dim3 cuda_block_dimensions(256);
	dim3 cuda_grid_dimensions(ceil_of_integer_quotient((unsigned) element_count, cuda_block_dimensions.x));
	ORUtils::MemoryBlock<bool> tolerance_exceeded(1, true, true);
	*tolerance_exceeded.GetData(MEMORYDEVICE_CPU) = false;
	tolerance_exceeded.UpdateDeviceFromHost();

	TwoRawMemoryArrayItemGenericComparison <<< cuda_grid_dimensions, cuda_block_dimensions >> >
			(tolerance_exceeded.GetData(MEMORYDEVICE_CUDA), l, r, element_count, compare_elements);

	ORcudaKernelCheck;
	tolerance_exceeded.UpdateHostFromDevice();
	return *tolerance_exceeded.GetData(MEMORYDEVICE_CPU);
}

} // end namespace internal

template<typename TElement>
bool RawMemoryArraysAlmostEqual_CUDA(const TElement* l, const TElement* r, const int element_count,
                                     const float absolute_tolerance) {
	//TODO: potentially, eliminate this method and use the generic directly by using a __host__ __device__ compare_elements lambda in calling functions
	auto compare_elements = [=] __device__(const TElement& element_l, const TElement& element_r) {
		return AlmostEqual(element_l, element_r, absolute_tolerance);
	};
	return internal::CompareRawMemoryArrays_Generic_CUDA(l, r, element_count, compare_elements);
}

template<typename TElement>
bool RawMemoryArraysEqual_CUDA(const TElement* l, const TElement* r, const int element_count) {
	//TODO: potentially, eliminate this method and use the generic directly by using a __host__ __device__ compare_elements lambda in calling functions
	auto compare_elements = [] __device__(const TElement& element_l, const TElement& element_r) {
		return element_l == element_r;
	};
	return internal::CompareRawMemoryArrays_Generic_CUDA(l, r, element_count, compare_elements);
}
// *** exact comparisons ***

// primitive specializations
extern template bool
RawMemoryArraysEqual_CUDA<bool>(const bool* l, const bool* r, const int element_count);

extern template bool
RawMemoryArraysEqual_CUDA<short>(const short* l, const short* r, const int element_count);

extern template bool
RawMemoryArraysEqual_CUDA<int>(const int* l, const int* r, const int element_count);

// Vector specializations
extern template bool
RawMemoryArraysEqual_CUDA<Vector3u>(const Vector3u* l, const Vector3u* r, const int element_count);

extern template bool
RawMemoryArraysEqual_CUDA<Vector2s>(const Vector2s* l,  const Vector2s* r, const int element_count);
extern template bool
RawMemoryArraysEqual_CUDA<Vector3s>(const Vector3s* l,  const Vector3s* r, const int element_count);
extern template bool
RawMemoryArraysEqual_CUDA<Vector4s>(const Vector4s* l,  const Vector4s* r, const int element_count);

extern template bool
RawMemoryArraysEqual_CUDA<Vector2i>(const Vector2i* l,  const Vector2i* r, const int element_count);
extern template bool
RawMemoryArraysEqual_CUDA<Vector3i>(const Vector3i* l,  const Vector3i* r, const int element_count);
extern template bool
RawMemoryArraysEqual_CUDA<Vector4i>(const Vector4i* l,  const Vector4i* r, const int element_count);
extern template bool
RawMemoryArraysEqual_CUDA<Vector6i>(const Vector6i* l,  const Vector6i* r, const int element_count);

extern template bool
RawMemoryArraysEqual_CUDA<Vector2f>(const Vector2f* l, const Vector2f* r, const int element_count);
extern template bool
RawMemoryArraysEqual_CUDA<Vector3f>(const Vector3f* l, const Vector3f* r, const int element_count);
extern template bool
RawMemoryArraysEqual_CUDA<Vector4f>(const Vector4f* l, const Vector4f* r, const int element_count);
extern template bool
RawMemoryArraysEqual_CUDA<Vector6f>(const Vector6f* l, const Vector6f* r, const int element_count);

extern template bool
RawMemoryArraysEqual_CUDA<Vector2d>(const Vector2d* l, const Vector2d* r, const int element_count);
extern template bool
RawMemoryArraysEqual_CUDA<Vector3d>(const Vector3d* l, const Vector3d* r, const int element_count);
extern template bool
RawMemoryArraysEqual_CUDA<Vector4d>(const Vector4d* l, const Vector4d* r, const int element_count);

// Matrix specializations
extern template bool
RawMemoryArraysEqual_CUDA<Matrix3f>(const Matrix3f* l, const Matrix3f* r, const int element_count);
extern template bool
RawMemoryArraysEqual_CUDA<Matrix4f>(const Matrix4f* l, const Matrix4f* r, const int element_count);

// *** approximate comparisons ***

// Vector specializations
extern template bool
RawMemoryArraysAlmostEqual_CUDA<Vector3u>(const Vector3u* l, const Vector3u* r, const int element_count,
                                          const float absolute_tolerance);

extern template bool
RawMemoryArraysAlmostEqual_CUDA<Vector2f>(const Vector2f* l, const Vector2f* r, const int element_count,
                                          const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual_CUDA<Vector3f>(const Vector3f* l, const Vector3f* r, const int element_count,
                                          const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual_CUDA<Vector4f>(const Vector4f* l, const Vector4f* r, const int element_count,
                                          const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual_CUDA<Vector6f>(const Vector6f* l, const Vector6f* r, const int element_count,
                                          const float absolute_tolerance);

extern template bool
RawMemoryArraysAlmostEqual_CUDA<Vector2d>(const Vector2d* l, const Vector2d* r, const int element_count,
                                          const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual_CUDA<Vector3d>(const Vector3d* l, const Vector3d* r, const int element_count,
                                          const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual_CUDA<Vector4d>(const Vector4d* l, const Vector4d* r, const int element_count,
                                          const float absolute_tolerance);

// Matrix specializations
extern template bool
RawMemoryArraysAlmostEqual_CUDA<Matrix3f>(const Matrix3f* l, const Matrix3f* r, const int element_count,
                                          const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual_CUDA<Matrix4f>(const Matrix4f* l, const Matrix4f* r, const int element_count,
                                          const float absolute_tolerance);

} // namespace ITMLib