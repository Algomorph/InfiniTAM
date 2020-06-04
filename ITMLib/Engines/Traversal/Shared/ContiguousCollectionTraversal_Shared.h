//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 6/2/20.
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
#include <climits>
#include "../../../../ORUtils/MemoryBlock.h"

namespace ITMLib{
enum class SpecialValue : unsigned int{
	USE_MAX = UINT_MAX
};
template<typename TCollection>
inline static void HandleDefaultElementCount(unsigned int& element_count, const TCollection& collection){
	if(element_count == static_cast<unsigned int>(SpecialValue::USE_MAX)){
		element_count =  static_cast<unsigned int>(collection.size());
	}
}

} // namespace ITMLib