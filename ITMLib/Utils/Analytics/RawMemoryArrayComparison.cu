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

#include "RawMemoryArrayComparison.cuh"

namespace ITMLib {

// Vector specializations
template bool
RawMemoryArraysAlmostEqual_CUDA<Vector3u>(const Vector3u* l, const Vector3u* r, const int element_count,
                                          const float absolute_tolerance);

template bool
RawMemoryArraysAlmostEqual_CUDA<Vector2f>(const Vector2f* l, const Vector2f* r, const int element_count,
                                          const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual_CUDA<Vector3f>(const Vector3f* l, const Vector3f* r, const int element_count,
                                          const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual_CUDA<Vector4f>(const Vector4f* l, const Vector4f* r, const int element_count,
                                          const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual_CUDA<Vector6f>(const Vector6f* l, const Vector6f* r, const int element_count,
                                          const float absolute_tolerance);

template bool
RawMemoryArraysAlmostEqual_CUDA<Vector2d>(const Vector2d* l, const Vector2d* r, const int element_count,
                                          const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual_CUDA<Vector3d>(const Vector3d* l, const Vector3d* r, const int element_count,
                                          const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual_CUDA<Vector4d>(const Vector4d* l, const Vector4d* r, const int element_count,
                                          const float absolute_tolerance);

// Matrix specializations
template bool
RawMemoryArraysAlmostEqual_CUDA<Matrix3f>(const Matrix3f* l, const Matrix3f* r, const int element_count,
                                          const float absolute_tolerance);
template bool
RawMemoryArraysAlmostEqual_CUDA<Matrix4f>(const Matrix4f* l, const Matrix4f* r, const int element_count,
                                          const float absolute_tolerance);

} // namespace ITMLib
