//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/21/20.
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
#include "MathTypePersistence.h"
#include "MathTypePersistence_Internal.h"

using namespace ORUtils::internal;

namespace ORUtils {

// == out ==
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector2<short>& vector) {
	SaveMathObject(destination, vector);
	return destination;
}
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector2<int>& vector) {
	SaveMathObject(destination, vector);
	return destination;
}
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector2<float>& vector) {
	SaveMathObject(destination, vector);
	return destination;
}
OStreamWrapper& operator<<(OStreamWrapper& destination, const Vector2<double>& vector) {
	SaveMathObject(destination, vector);
	return destination;
}

// == in ==
IStreamWrapper& operator>>(IStreamWrapper& source, Vector2<short>& vector) {
	LoadMathObject(source, vector);
	return source;
}
IStreamWrapper& operator>>(IStreamWrapper& source, Vector2<int>& vector) {
	LoadMathObject(source, vector);
	return source;
}
IStreamWrapper& operator>>(IStreamWrapper& source, Vector2<float>& vector) {
	LoadMathObject(source, vector);
	return source;
}
IStreamWrapper& operator>>(IStreamWrapper& source, Vector2<double>& vector) {
	LoadMathObject(source, vector);
	return source;
}

} // namespace ORUtils