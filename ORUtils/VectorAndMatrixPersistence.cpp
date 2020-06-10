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
#include "VectorAndMatrixPersistence.h"

namespace ORUtils {
template void SaveMatrix<Matrix4<float>>(OStreamWrapper& file, const Matrix4<float>& matrix);
template void SaveMatrix<Matrix4<double>>(OStreamWrapper& file, const Matrix4<double>& matrix);
template void SaveMatrix<Matrix4<int>>(OStreamWrapper& file, const Matrix4<int>& matrix);
template void SaveMatrix<Matrix4<unsigned int>>(OStreamWrapper& file, const Matrix4<unsigned int>& matrix);
template void SaveMatrix<Matrix4<short>>(OStreamWrapper& file, const Matrix4<short>& matrix);
template void SaveMatrix<Matrix4<unsigned short>>(OStreamWrapper& file, const Matrix4<unsigned short>& matrix);
template void SaveMatrix<Matrix4<char>>(OStreamWrapper& file, const Matrix4<char>& matrix);
template void SaveMatrix<Matrix4<unsigned char>>(OStreamWrapper& file, const Matrix4<unsigned char>& matrix);
} // namespace ORUtils