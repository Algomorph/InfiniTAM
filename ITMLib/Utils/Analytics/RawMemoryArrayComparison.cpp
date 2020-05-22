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
#include "RawMemoryArrayComparison.h"

namespace ITMLib {

// Vector specializations
template bool
RawMemoryArraysAlmostEqual<Vector3u>(const Vector3u* l, MemoryDeviceType memory_device_type_l, const Vector3u* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);

template bool
RawMemoryArraysAlmostEqual<Vector2f>(const Vector2f* l, MemoryDeviceType memory_device_type_l, const Vector2f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual<Vector3f>(const Vector3f* l, MemoryDeviceType memory_device_type_l, const Vector3f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual<Vector4f>(const Vector4f* l, MemoryDeviceType memory_device_type_l, const Vector4f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual<Vector6f>(const Vector6f* l, MemoryDeviceType memory_device_type_l, const Vector6f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);

template bool
RawMemoryArraysAlmostEqual<Vector2d>(const Vector2d* l, MemoryDeviceType memory_device_type_l, const Vector2d* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual<Vector3d>(const Vector3d* l, MemoryDeviceType memory_device_type_l, const Vector3d* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual<Vector4d>(const Vector4d* l, MemoryDeviceType memory_device_type_l, const Vector4d* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);

// Matrix specializations
template bool
RawMemoryArraysAlmostEqual<Matrix3f>(const Matrix3f* l, MemoryDeviceType memory_device_type_l, const Matrix3f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual<Matrix4f>(const Matrix4f* l, MemoryDeviceType memory_device_type_l, const Matrix4f* r,
                                     MemoryDeviceType memory_device_type_r, const int element_count,
                                     const float absolute_tolerance);

} // namespace ITMLib