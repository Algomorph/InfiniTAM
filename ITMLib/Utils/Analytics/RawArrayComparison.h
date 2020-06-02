//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/21/20.
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

//stdlib
#include <algorithm>

//thrust
#ifndef COMPILE_WITHOUT_CUDA

#include <thrust/sort.h>
#include <thrust/execution_policy.h>

#endif

#include "../../../ORUtils/PlatformIndependence.h"
#include "../../../ORUtils/CrossPlatformMacros.h"
#include "../../../ORUtils/MemoryDeviceType.h"
#include "AlmostEqual.h"


namespace ITMLib {

namespace internal {

#ifndef COMPILE_WITHOUT_CUDA
template<typename TElement, typename TComparisonFunction, typename TReportMismatchFunction>
bool CompareRawArrays_Generic_CUDA(const TElement* left, const TElement* right, const int element_count,
                                   TComparisonFunction&& compare_elements, TReportMismatchFunction&& report_mismatch);
#endif

template<typename TElement, typename TCompareElementsFunction, /*typename TCompareArraysCUDAFunction,*/ typename TReportMismatchFunction>
bool CompareRawArrays_Generic(const TElement* left, MemoryDeviceType memory_device_type_left,
                              const TElement* right, MemoryDeviceType memory_device_type_right,
                              const int element_count,
                              TCompareElementsFunction&& compare_elements,
                              TReportMismatchFunction&& report_mismatch,
                              bool presort = false) {
	assert(element_count >= 0);
	if (element_count == 0) return true;

	MemoryCopyDirection direction = DetermineMemoryCopyDirection(memory_device_type_right, memory_device_type_left);

	switch (direction) {
		case CPU_TO_CPU: {
			bool mismatch_found = false;
			const TElement* left_prepared;
			const TElement* right_prepared;
			TElement* left_sorted;
			TElement* right_sorted;
			if (presort) {
				left_sorted = new TElement[element_count];
				memcpy(left_sorted, left, element_count * sizeof(TElement));
				std::sort(left_sorted, left_sorted + element_count);
				right_sorted = new TElement[element_count];
				memcpy(right_sorted, right, element_count * sizeof(TElement));
				std::sort(right_sorted, right_sorted + element_count);
				left_prepared = left_sorted;
				right_prepared = right_sorted;
			} else {
				left_prepared = left;
				right_prepared = right;
			}
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(left_prepared, right_prepared, compare_elements, mismatch_found, report_mismatch)
#endif
			for (int i_element = 0; i_element < element_count; i_element++) {
				if (mismatch_found) {
					continue;
				}
				if (!std::forward<TCompareElementsFunction>(compare_elements)(left_prepared[i_element], right_prepared[i_element])) {
					mismatch_found = true;
					std::forward<TReportMismatchFunction>(report_mismatch)(left_prepared[i_element], right_prepared[i_element], i_element);
				}
			}
			if (presort) {
				delete[] left_sorted;
				delete[] right_sorted;
			}
			return !mismatch_found;
		}
			break;
		case CUDA_TO_CPU: {
#ifdef COMPILE_WITHOUT_CUDA
			DIEWITHEXCEPTIONREPORTLOCATION("Raw memory array comparison called on CPU & CUDA array while code build without CUDA support.");
		return false;
#else
			TElement* left_CPU = new TElement[element_count];
			ORcudaSafeCall(cudaMemcpy(left_CPU, left, element_count * sizeof(TElement), cudaMemcpyDeviceToHost));
			//TODO: presort here and make recursive call w/o sorting to avoid unnecessary copying
			bool result = CompareRawArrays_Generic
					(left_CPU, MEMORYDEVICE_CPU, right, MEMORYDEVICE_CPU,
					 element_count, compare_elements, report_mismatch, presort);
			delete[] left_CPU;
			return result;
#endif

		}
		case CPU_TO_CUDA: {
#ifdef COMPILE_WITHOUT_CUDA
			DIEWITHEXCEPTIONREPORTLOCATION("Raw memory array comparison called on CPU & CUDA array while code build without CUDA support.");
		return false;
#else
			auto* right_CPU = new TElement[element_count];
			ORcudaSafeCall(cudaMemcpy(right_CPU, right, element_count * sizeof(TElement), cudaMemcpyDeviceToHost));
			//TODO: presort here and make recursive call w/o sorting to avoid unnecessary copying
			bool result = CompareRawArrays_Generic
					(left, MEMORYDEVICE_CPU, right_CPU, MEMORYDEVICE_CPU,
					 element_count, compare_elements, report_mismatch, presort);
			delete[] right_CPU;
			return result;
#endif
		}
		case CUDA_TO_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
			DIEWITHEXCEPTIONREPORTLOCATION("Raw memory array comparison called on two CUDA arrays while code build without CUDA support.");
		return false;
#else
			const TElement* left_prepared;
			const TElement* right_prepared;
			TElement* left_sorted;
			TElement* right_sorted;
			if (presort) {
				ORcudaSafeCall(cudaMalloc((void**) &left_sorted, element_count * sizeof(TElement)));
				ORcudaSafeCall(cudaMemcpy(left_sorted, left, element_count * sizeof(TElement), cudaMemcpyDeviceToDevice));
				thrust::sort(thrust::device, left_sorted, left_sorted + element_count);
				ORcudaSafeCall(cudaMalloc((void**) &right_sorted, element_count * sizeof(TElement)));
				ORcudaSafeCall(cudaMemcpy(right_sorted, right, element_count * sizeof(TElement), cudaMemcpyDeviceToDevice));
				thrust::sort(thrust::device, right_sorted, right_sorted + element_count);
				left_prepared = left_sorted;
				right_prepared = right_sorted;
			} else {
				left_prepared = left;
				right_prepared = right;
			}
			internal::CompareRawArrays_Generic_CUDA(left_prepared, right_prepared, element_count, compare_elements, report_mismatch);
#endif
			if (presort) {
				ORcudaSafeCall(cudaFree(left_sorted));
				ORcudaSafeCall(cudaFree(right_sorted));
			}
			break;
		default:
			// unsupported MemoryCopyDirection
			assert(false);
			return false;
	}
	return true;
}
} // namespace internal

template<typename TElement>
bool RawArraysEqual(const TElement* a, MemoryDeviceType memory_device_type_l,
                    const TElement* b, MemoryDeviceType memory_device_type_r,
                    const int element_count, bool presort = false) {
	return internal::CompareRawArrays_Generic(
			a, memory_device_type_l, b, memory_device_type_r, element_count,
			CPU_AND_GPU_CAPTURE_LAMBDA()(const TElement& element_a, const TElement& element_b) {
				return element_a == element_b;
			},
			[](const TElement& element_a, const TElement& element_b, const int mismatch_index) {},
			presort
	);
}

template<typename TElement>
bool RawArraysEqual_Verbose(const TElement* a, MemoryDeviceType memory_device_type_l,
                                  const TElement* b, MemoryDeviceType memory_device_type_r,
                                  const int element_count, bool presort = false) {
	return internal::CompareRawArrays_Generic(
			a, memory_device_type_l, b, memory_device_type_r, element_count,
			CPU_AND_GPU_CAPTURE_LAMBDA()(const TElement& element_a, const TElement& element_b) {
				return element_a == element_b;
			},
			[](const TElement& element_a, const TElement& element_b, const int mismatch_index) {
				std::cerr << "Memory array comparison failure. First discovered mismatch: "
				          << element_a << " vs. " << element_b << " at position " << mismatch_index << "." << std::endl;
			},
			presort
	);
}

template<typename TElement>
bool RawArraysAlmostEqual(const TElement* a, MemoryDeviceType memory_device_type_l,
                                const TElement* b, MemoryDeviceType memory_device_type_r,
                                const int element_count, const float absolute_tolerance = 1e-6, bool presort = false) {
	return internal::CompareRawArrays_Generic(
			a, memory_device_type_l, b, memory_device_type_r, element_count,
			CPU_AND_GPU_CAPTURE_LAMBDA(&absolute_tolerance)(
					const TElement& element_a,
					const TElement& element_b) {
				return AlmostEqual(element_a, element_b, absolute_tolerance);
			},
			[](const TElement& element_a, const TElement& element_b, const int mismatch_index) {},
			presort
	);
}

template<typename TElement>
bool RawArraysAlmostEqual_Verbose(const TElement* a, MemoryDeviceType memory_device_type_l,
                                        const TElement* b, MemoryDeviceType memory_device_type_r,
                                        const int element_count, const float absolute_tolerance = 1e-6, bool presort = false) {
	return internal::CompareRawArrays_Generic(
			a, memory_device_type_l, b, memory_device_type_r, element_count,
			CPU_AND_GPU_CAPTURE_LAMBDA(&absolute_tolerance)(
					const TElement& element_a,
					const TElement& element_b) {
				return AlmostEqual(element_a, element_b, absolute_tolerance);
			},
			[&absolute_tolerance](const TElement& element_a, const TElement& element_b, const int mismatch_index) {
				std::cerr << "Memory array approximate comparison failure. First discovered mismatch: "
				          << element_a << " vs. " << element_b << " at position " << mismatch_index
				          << ". Tolerance " << absolute_tolerance << " exceeded." << std::endl;
			},
			presort
	);
}

// region =================================== external template declarations ===========================================
// *** exact comparisons ***
// primitive specializations
extern template bool
RawArraysEqual<bool>(const bool* l, MemoryDeviceType memory_device_type_l, const bool* r,
                           MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual<short>(const short* l, MemoryDeviceType memory_device_type_l, const short* r,
                            MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual<int>(const int* l, MemoryDeviceType memory_device_type_l, const int* r,
                          MemoryDeviceType memory_device_type_r, const int element_count, bool presort);


// Vector specializations
extern template bool
RawArraysEqual<Vector3u>(const Vector3u* l, MemoryDeviceType memory_device_type_l, const Vector3u* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Vector4u>(const Vector4u* l, MemoryDeviceType memory_device_type_l, const Vector4u* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual<Vector2s>(const Vector2s* l, MemoryDeviceType memory_device_type_l, const Vector2s* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Vector3s>(const Vector3s* l, MemoryDeviceType memory_device_type_l, const Vector3s* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Vector4s>(const Vector4s* l, MemoryDeviceType memory_device_type_l, const Vector4s* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual<Vector2i>(const Vector2i* l, MemoryDeviceType memory_device_type_l, const Vector2i* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Vector3i>(const Vector3i* l, MemoryDeviceType memory_device_type_l, const Vector3i* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Vector4i>(const Vector4i* l, MemoryDeviceType memory_device_type_l, const Vector4i* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Vector6i>(const Vector6i* l, MemoryDeviceType memory_device_type_l, const Vector6i* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual<Vector2f>(const Vector2f* l, MemoryDeviceType memory_device_type_l, const Vector2f* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Vector3f>(const Vector3f* l, MemoryDeviceType memory_device_type_l, const Vector3f* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Vector4f>(const Vector4f* l, MemoryDeviceType memory_device_type_l, const Vector4f* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Vector6f>(const Vector6f* l, MemoryDeviceType memory_device_type_l, const Vector6f* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual<Vector2d>(const Vector2d* l, MemoryDeviceType memory_device_type_l, const Vector2d* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Vector3d>(const Vector3d* l, MemoryDeviceType memory_device_type_l, const Vector3d* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Vector4d>(const Vector4d* l, MemoryDeviceType memory_device_type_l, const Vector4d* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

// Matrix specializations
extern template bool
RawArraysEqual<Matrix3f>(const Matrix3f* l, MemoryDeviceType memory_device_type_l, const Matrix3f* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual<Matrix4f>(const Matrix4f* l, MemoryDeviceType memory_device_type_l, const Matrix4f* r,
                               MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

// *** verbose exact comparisons ***
// primitive specializations
extern template bool
RawArraysEqual_Verbose<bool>(const bool* l, MemoryDeviceType memory_device_type_l, const bool* r,
                                   MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual_Verbose<short>(const short* l, MemoryDeviceType memory_device_type_l, const short* r,
                                    MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual_Verbose<int>(const int* l, MemoryDeviceType memory_device_type_l, const int* r,
                                  MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

// Vector specializations
extern template bool
RawArraysEqual_Verbose<Vector3u>(const Vector3u* l, MemoryDeviceType memory_device_type_l, const Vector3u* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Vector4u>(const Vector4u* l, MemoryDeviceType memory_device_type_l, const Vector4u* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual_Verbose<Vector2s>(const Vector2s* l, MemoryDeviceType memory_device_type_l, const Vector2s* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Vector3s>(const Vector3s* l, MemoryDeviceType memory_device_type_l, const Vector3s* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Vector4s>(const Vector4s* l, MemoryDeviceType memory_device_type_l, const Vector4s* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual_Verbose<Vector2i>(const Vector2i* l, MemoryDeviceType memory_device_type_l, const Vector2i* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Vector3i>(const Vector3i* l, MemoryDeviceType memory_device_type_l, const Vector3i* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Vector4i>(const Vector4i* l, MemoryDeviceType memory_device_type_l, const Vector4i* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Vector6i>(const Vector6i* l, MemoryDeviceType memory_device_type_l, const Vector6i* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual_Verbose<Vector2f>(const Vector2f* l, MemoryDeviceType memory_device_type_l, const Vector2f* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Vector3f>(const Vector3f* l, MemoryDeviceType memory_device_type_l, const Vector3f* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Vector4f>(const Vector4f* l, MemoryDeviceType memory_device_type_l, const Vector4f* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Vector6f>(const Vector6f* l, MemoryDeviceType memory_device_type_l, const Vector6f* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

extern template bool
RawArraysEqual_Verbose<Vector2d>(const Vector2d* l, MemoryDeviceType memory_device_type_l, const Vector2d* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Vector3d>(const Vector3d* l, MemoryDeviceType memory_device_type_l, const Vector3d* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Vector4d>(const Vector4d* l, MemoryDeviceType memory_device_type_l, const Vector4d* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

// Matrix specializations
extern template bool
RawArraysEqual_Verbose<Matrix3f>(const Matrix3f* l, MemoryDeviceType memory_device_type_l, const Matrix3f* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);
extern template bool
RawArraysEqual_Verbose<Matrix4f>(const Matrix4f* l, MemoryDeviceType memory_device_type_l, const Matrix4f* r,
                                       MemoryDeviceType memory_device_type_r, const int element_count, bool presort);

// *** approximate comparisons *** 

// Vector specializations
extern template bool
RawArraysAlmostEqual<Vector3u>(const Vector3u* l, MemoryDeviceType memory_device_type_l, const Vector3u* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual<Vector4u>(const Vector4u* l, MemoryDeviceType memory_device_type_l, const Vector4u* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance, bool presort);

extern template bool
RawArraysAlmostEqual<Vector2f>(const Vector2f* l, MemoryDeviceType memory_device_type_l, const Vector2f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual<Vector3f>(const Vector3f* l, MemoryDeviceType memory_device_type_l, const Vector3f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual<Vector4f>(const Vector4f* l, MemoryDeviceType memory_device_type_l, const Vector4f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual<Vector6f>(const Vector6f* l, MemoryDeviceType memory_device_type_l, const Vector6f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance, bool presort);

extern template bool
RawArraysAlmostEqual<Vector2d>(const Vector2d* l, MemoryDeviceType memory_device_type_l, const Vector2d* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual<Vector3d>(const Vector3d* l, MemoryDeviceType memory_device_type_l, const Vector3d* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual<Vector4d>(const Vector4d* l, MemoryDeviceType memory_device_type_l, const Vector4d* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance, bool presort);

// Matrix specializations
extern template bool
RawArraysAlmostEqual<Matrix3f>(const Matrix3f* l, MemoryDeviceType memory_device_type_l, const Matrix3f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual<Matrix4f>(const Matrix4f* l, MemoryDeviceType memory_device_type_l, const Matrix4f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance, bool presort);

// *** verbose approximate comparisons *** 

// Vector specializations
extern template bool
RawArraysAlmostEqual_Verbose<Vector3u>(const Vector3u* l, MemoryDeviceType memory_device_type_l, const Vector3u* r,
                                             MemoryDeviceType memory_device_type_r, const int element_count,
                                             const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual_Verbose<Vector4u>(const Vector4u* l, MemoryDeviceType memory_device_type_l, const Vector4u* r,
                                             MemoryDeviceType memory_device_type_r, const int element_count,
                                             const float absolute_tolerance, bool presort);

extern template bool
RawArraysAlmostEqual_Verbose<Vector2f>(const Vector2f* l, MemoryDeviceType memory_device_type_l, const Vector2f* r,
                                             MemoryDeviceType memory_device_type_r, const int element_count,
                                             const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual_Verbose<Vector3f>(const Vector3f* l, MemoryDeviceType memory_device_type_l, const Vector3f* r,
                                             MemoryDeviceType memory_device_type_r, const int element_count,
                                             const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual_Verbose<Vector4f>(const Vector4f* l, MemoryDeviceType memory_device_type_l, const Vector4f* r,
                                             MemoryDeviceType memory_device_type_r, const int element_count,
                                             const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual_Verbose<Vector6f>(const Vector6f* l, MemoryDeviceType memory_device_type_l, const Vector6f* r,
                                             MemoryDeviceType memory_device_type_r, const int element_count,
                                             const float absolute_tolerance, bool presort);

extern template bool
RawArraysAlmostEqual_Verbose<Vector2d>(const Vector2d* l, MemoryDeviceType memory_device_type_l, const Vector2d* r,
                                             MemoryDeviceType memory_device_type_r, const int element_count,
                                             const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual_Verbose<Vector3d>(const Vector3d* l, MemoryDeviceType memory_device_type_l, const Vector3d* r,
                                             MemoryDeviceType memory_device_type_r, const int element_count,
                                             const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual_Verbose<Vector4d>(const Vector4d* l, MemoryDeviceType memory_device_type_l, const Vector4d* r,
                                             MemoryDeviceType memory_device_type_r, const int element_count,
                                             const float absolute_tolerance, bool presort);

// Matrix specializations
extern template bool
RawArraysAlmostEqual_Verbose<Matrix3f>(const Matrix3f* l, MemoryDeviceType memory_device_type_l, const Matrix3f* r,
                                             MemoryDeviceType memory_device_type_r, const int element_count,
                                             const float absolute_tolerance, bool presort);
extern template bool
RawArraysAlmostEqual_Verbose<Matrix4f>(const Matrix4f* l, MemoryDeviceType memory_device_type_l, const Matrix4f* r,
                                             MemoryDeviceType memory_device_type_r, const int element_count,
                                             const float absolute_tolerance, bool presort);

// endregion
} // namespace ITMLib