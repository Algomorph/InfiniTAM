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

#include "../../../ORUtils/PlatformIndependence.h"
#include "../../../ORUtils/MemoryDeviceType.h"
#include "AlmostEqual.h"

namespace ITMLib {

namespace internal {
template<typename TElement, typename TCompareElementsCPUFunction, typename TCompareArraysCUDAFunction>
bool CompareRawMemoryArrays_Generic(const TElement* a, MemoryDeviceType memory_device_type_l,
                                    const TElement* b, MemoryDeviceType memory_device_type_r,
                                    const int element_count,
                                    TCompareElementsCPUFunction&& compare_elements,
                                    TCompareArraysCUDAFunction&& compare_arrays_CUDA) {
	assert(element_count >= 0);
	if (element_count == 0) return true;

	MemoryCopyDirection direction = DetermineMemoryCopyDirection(memory_device_type_l, memory_device_type_r);

	switch (direction) {
		case CPU_TO_CPU: {
			bool mismatch_found = false;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(a, b, compare_elements, mismatch_found)
#endif
			for (int i_element = 0; i_element < element_count; i_element++) {
				if (mismatch_found) {
					continue;
				}
				if (!std::forward<TCompareElementsCPUFunction>(compare_elements)(a[i_element], b[i_element])) {
					mismatch_found = true;
				}
			}
			return !mismatch_found;
		}
			break;
		case CUDA_TO_CPU: {
#ifdef COMPILE_WITHOUT_CUDA
			DIEWITHEXCEPTIONREPORTLOCATION("Raw memory array comparison called on CPU & CUDA array while code build without CUDA support.");
		return false;
#else
			auto* a_TEMP = new TElement[element_count];
			ORcudaSafeCall(cudaMemcpy(a_TEMP, a, element_count * sizeof(TElement), cudaMemcpyDeviceToHost));
			bool result = CompareRawMemoryArrays_Generic
					(a_TEMP, MEMORYDEVICE_CPU, b, MEMORYDEVICE_CPU,
					 element_count, compare_elements, compare_arrays_CUDA);
			delete[] a_TEMP;
			return result;
#endif

		}
		case CPU_TO_CUDA: {
#ifdef COMPILE_WITHOUT_CUDA
			DIEWITHEXCEPTIONREPORTLOCATION("Raw memory array comparison called on CPU & CUDA array while code build without CUDA support.");
		return false;
#else
			auto* b_TEMP = new TElement[element_count];
			ORcudaSafeCall(cudaMemcpy(b_TEMP, b, element_count * sizeof(TElement), cudaMemcpyDeviceToHost));
			bool result = CompareRawMemoryArrays_Generic
					(a, MEMORYDEVICE_CPU, b_TEMP, MEMORYDEVICE_CPU,
					 element_count, compare_elements, compare_arrays_CUDA);
			delete[] b_TEMP;
			return result;
#endif
		}
		case CUDA_TO_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
			DIEWITHEXCEPTIONREPORTLOCATION("Raw memory array comparison called on two CUDA arrays while code build without CUDA support.");
		return false;
#else
			return std::forward<TCompareArraysCUDAFunction>(compare_arrays_CUDA)(b, a, element_count);
#endif
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
bool RawMemoryArraysEqual_CUDA(const TElement* l, const TElement* r, const int element_count);

template<typename TElement>
bool RawMemoryArraysAlmostEqual_CUDA(const TElement* l, const TElement* r, const int element_count,
                                     const float absolute_tolerance = 1e-6);

template<typename TElement>
bool RawMemoryArraysAlmostEqual(const TElement* a, MemoryDeviceType memory_device_type_l,
                                const TElement* b, MemoryDeviceType memory_device_type_r,
                                const int element_count, const float absolute_tolerance = 1e-6) {
	return internal::CompareRawMemoryArrays_Generic(
			a, memory_device_type_l, b, memory_device_type_r, element_count,
			[&absolute_tolerance](
					const TElement& element_a,
					const TElement& element_b) {
				return AlmostEqual(element_a, element_b, absolute_tolerance);
			},
			[&absolute_tolerance](const TElement* a, const TElement* b, const int element_count) {
				return RawMemoryArraysAlmostEqual_CUDA<TElement>(b, a, element_count, absolute_tolerance);
			}
	);
}



// region =================================== external template declarations ===========================================
// *** Generic comparisons

// Vector specializations
extern template bool
RawMemoryArraysAlmostEqual<Vector3u>(const Vector3u* l, MemoryDeviceType memory_device_type_l, const Vector3u* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);

extern template bool
RawMemoryArraysAlmostEqual<Vector2s>(const Vector2s* l, MemoryDeviceType memory_device_type_l, const Vector2s* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual<Vector3s>(const Vector3s* l, MemoryDeviceType memory_device_type_l, const Vector3s* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual<Vector4s>(const Vector4s* l, MemoryDeviceType memory_device_type_l, const Vector4s* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);

extern template bool
RawMemoryArraysAlmostEqual<Vector2i>(const Vector2i* l, MemoryDeviceType memory_device_type_l, const Vector2i* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual<Vector3i>(const Vector3i* l, MemoryDeviceType memory_device_type_l, const Vector3i* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual<Vector4i>(const Vector4i* l, MemoryDeviceType memory_device_type_l, const Vector4i* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual<Vector6i>(const Vector6i* l, MemoryDeviceType memory_device_type_l, const Vector6i* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);

extern template bool
RawMemoryArraysAlmostEqual<Vector2f>(const Vector2f* l, MemoryDeviceType memory_device_type_l, const Vector2f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual<Vector3f>(const Vector3f* l, MemoryDeviceType memory_device_type_l, const Vector3f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual<Vector4f>(const Vector4f* l, MemoryDeviceType memory_device_type_l, const Vector4f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual<Vector6f>(const Vector6f* l, MemoryDeviceType memory_device_type_l, const Vector6f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);

extern template bool
RawMemoryArraysAlmostEqual<Vector2d>(const Vector2d* l, MemoryDeviceType memory_device_type_l, const Vector2d* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual<Vector3d>(const Vector3d* l, MemoryDeviceType memory_device_type_l, const Vector3d* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual<Vector4d>(const Vector4d* l, MemoryDeviceType memory_device_type_l, const Vector4d* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);

// Matrix specializations
extern template bool
RawMemoryArraysAlmostEqual<Matrix3f>(const Matrix3f* l, MemoryDeviceType memory_device_type_l, const Matrix3f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
extern template bool
RawMemoryArraysAlmostEqual<Matrix4f>(const Matrix4f* l, MemoryDeviceType memory_device_type_l, const Matrix4f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);

// endregion
} // namespace ITMLib