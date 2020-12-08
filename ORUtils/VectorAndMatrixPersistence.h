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

#include "Matrix.h"
#include "OStreamWrapper.h"
#include "IStreamWrapper.h"

namespace ORUtils {

template<typename TMatrix>
void SaveMatrix(OStreamWrapper& file, const TMatrix& matrix) {
	for (int i_value = 0; i_value < TMatrix::element_count; i_value++) {
		typename TMatrix::value_type value = matrix.getValues()[i_value];
		file.OStream().write(reinterpret_cast<const char*>(&value), sizeof(typename TMatrix::value_type));
	}
}

template<typename TMatrix>
TMatrix LoadMatrix(IStreamWrapper& file) {
	TMatrix matrix;
	for (int i_value = 0; i_value < TMatrix::element_count; i_value++) {
		file.IStream().read(reinterpret_cast<char*>(matrix.values + i_value), sizeof(typename TMatrix::value_type));
	}
	return matrix;
}

template<typename TVector>
const auto SaveVector = SaveMatrix<TVector>;

template<typename TVector>
const auto LoadVector = LoadMatrix<TVector>;

// Matrix4
extern template void SaveMatrix<Matrix4<float>>(OStreamWrapper& file, const Matrix4<float>& matrix);
extern template void SaveMatrix<Matrix4<double>>(OStreamWrapper& file, const Matrix4<double>& matrix);
extern template void SaveMatrix<Matrix4<int>>(OStreamWrapper& file, const Matrix4<int>& matrix);
extern template void SaveMatrix<Matrix4<unsigned int>>(OStreamWrapper& file, const Matrix4<unsigned int>& matrix);
extern template void SaveMatrix<Matrix4<short>>(OStreamWrapper& file, const Matrix4<short>& matrix);
extern template void SaveMatrix<Matrix4<unsigned short>>(OStreamWrapper& file, const Matrix4<unsigned short>& matrix);
extern template void SaveMatrix<Matrix4<char>>(OStreamWrapper& file, const Matrix4<char>& matrix);
extern template void SaveMatrix<Matrix4<unsigned char>>(OStreamWrapper& file, const Matrix4<unsigned char>& matrix);

extern template Matrix4<float> LoadMatrix<Matrix4<float>>(IStreamWrapper& file);
extern template Matrix4<double> LoadMatrix<Matrix4<double>>(IStreamWrapper& file);
extern template Matrix4<int> LoadMatrix<Matrix4<int>>(IStreamWrapper& file);
extern template Matrix4<unsigned int> LoadMatrix<Matrix4<unsigned int>>(IStreamWrapper& file);
extern template Matrix4<short> LoadMatrix<Matrix4<short>>(IStreamWrapper& file);
extern template Matrix4<unsigned short> LoadMatrix<Matrix4<unsigned short>>(IStreamWrapper& file);
extern template Matrix4<char> LoadMatrix<Matrix4<char>>(IStreamWrapper& file);
extern template Matrix4<unsigned char> LoadMatrix<Matrix4<unsigned char>>(IStreamWrapper& file);

// Vector6
extern template void SaveMatrix<Vector6<int>>(OStreamWrapper& file, const Vector6<int>& vector_6);
extern template Vector6<int> LoadMatrix<Vector6<int>>(IStreamWrapper& file);


} // namespace ORUtils