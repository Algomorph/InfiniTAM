//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 6/9/20.
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

#include "../Matrix.h"
#include "../Vector.h"
#include "../OStreamWrapper.h"
#include "../IStreamWrapper.h"

namespace ORUtils {

// ============== operator read/write ===================
// ==== Matrix4 ====
// == out ==
OStreamWrapper& operator<<(OStreamWrapper& destination, const Matrix4<float>& matrix);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Matrix4<double>& matrix);
// == in ==
IStreamWrapper& operator>>(IStreamWrapper& source, Matrix4<float>& matrix);
IStreamWrapper& operator>>(IStreamWrapper& source, Matrix4<double>& matrix);

// ==== Matrix3 ====
// == out ==
OStreamWrapper& operator<<(OStreamWrapper& destination, const Matrix3<float>& matrix);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Matrix3<double>& matrix);
// == in == 
IStreamWrapper& operator>>(IStreamWrapper& source,  Matrix3<float>& matrix);
IStreamWrapper& operator>>(IStreamWrapper& source,  Matrix3<double>& matrix);

// ==== Vector6 ====
// == out ==
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector6<int>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector6<float>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector6<double>& vector);
// == in ==
IStreamWrapper& operator>>(IStreamWrapper& source, Vector6<int>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector6<float>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector6<double>& vector);

// ==== Vector4 ====
// == out ==
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector4<unsigned char>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector4<short>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector4<int>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector4<unsigned int>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector4<float>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector4<double>& vector);
// == in ==
IStreamWrapper& operator>>(IStreamWrapper& source, Vector4<unsigned char>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector4<short>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector4<int>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector4<unsigned int>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector4<float>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector4<double>& vector);

// ==== Vector3 ====
// == out ==
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector3<unsigned char>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector3<short>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector3<int>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector3<unsigned int>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector3<float>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector3<double>& vector);
// == in ==
IStreamWrapper& operator>>(IStreamWrapper& source, Vector3<unsigned char>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector3<short>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector3<unsigned int>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector3<int>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector3<float>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector3<double>& vector);

// ==== Vector2 ====
// == out ==
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector2<short>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector2<int>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector2<float>& vector);
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector2<double>& vector);
// == in ==
IStreamWrapper& operator>>(IStreamWrapper& source, Vector2<short>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector2<int>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector2<float>& vector);
IStreamWrapper& operator>>(IStreamWrapper& source, Vector2<double>& vector);

} // namespace ORUtils