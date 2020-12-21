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
#pragma once

namespace ORUtils::internal{

template<typename TMathObject>
void SaveMathObject(OStreamWrapper& file, const TMathObject& math_object) {
	for (int i_value = 0; i_value < TMathObject::element_count; i_value++) {
		typename TMathObject::value_type value = math_object.getValues()[i_value];
		file.OStream().write(reinterpret_cast<const char*>(&value), sizeof(typename TMathObject::value_type));
	}
}

template<typename TMathObject>
void LoadMathObject(IStreamWrapper& file, TMathObject& math_object){
	for (int i_value = 0; i_value < TMathObject::element_count; i_value++) {
		file.IStream().read(reinterpret_cast<char*>(math_object.values + i_value), sizeof(typename TMathObject::value_type));
	}
}

template<typename TMathObject>
TMathObject LoadMathObject(IStreamWrapper& file) {
	TMathObject math_object;
	LoadMathObject(file, math_object);
	return math_object;
}

} // namespace ORUtils::internal