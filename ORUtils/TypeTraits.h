//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/24/20.
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
#include "Vector.h"

namespace ORUtils{

template<typename T>
struct TypeTraits {};

template<>
struct TypeTraits<bool>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<char>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<unsigned char>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<short>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<unsigned short>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<int>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<unsigned int>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<long>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<unsigned long>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<long long>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<unsigned long long>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<float>{
	static constexpr int element_count = 1;
};
template<>
struct TypeTraits<double>{
	static constexpr int element_count = 1;
};



template<typename T>
struct TypeTraits<Vector2<T>>{
	static constexpr int element_count = Vector2<T>::element_count;
};

template<typename T>
struct TypeTraits<Vector3<T>>{
	static constexpr int element_count = Vector3<T>::element_count;
};

template<typename T>
struct TypeTraits<Vector4<T>>{
	static constexpr int element_count = Vector4<T>::element_count;
};

template<typename T>
struct TypeTraits<Vector6<T>>{
	static constexpr int element_count = Vector6<T>::element_count;
};

template<typename T, int s>
struct TypeTraits<VectorX<T, s>>{
	static constexpr int element_count = VectorX<T,s>::element_count;
};

extern template struct TypeTraits<bool>;
extern template struct TypeTraits<short>;
extern template struct TypeTraits<int>;
extern template struct TypeTraits<Vector3<unsigned char>>;
extern template struct TypeTraits<Vector3<float>>;


} // namespace ORUtils
