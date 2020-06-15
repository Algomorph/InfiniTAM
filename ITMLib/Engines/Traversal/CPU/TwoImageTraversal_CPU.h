//  ================================================================
//  Created by Gregory Kramida on 2/4/20.
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
//local
#include "../Interface/TwoImageTraversal.h"
#include "../../../Utils/Math.h"
#include "../../../../ORUtils/Image.h"

namespace ITMLib {

template<typename TImage1Element, typename TImage2Element>
class TwoImageTraversalEngine<TImage1Element, TImage2Element, MEMORYDEVICE_CPU> {
public:
	template<typename TFunctor>
	inline static void
	TraverseWithPosition(const ORUtils::Image<TImage1Element>* image1, const ORUtils::Image<TImage2Element>* image2, TFunctor& functor){

		const Vector2i resolution = image1->dimensions;
		const int element_count = resolution.x * resolution.y;

		const TImage1Element* image1_data = image1->GetData(MEMORYDEVICE_CPU);
		const TImage2Element* image2_data = image2->GetData(MEMORYDEVICE_CPU);
#ifdef WITH_OPENMP
	#pragma omp parallel for default(none) shared(functor, image1_data, image2_data) firstprivate(element_count) \
	firstprivate(resolution)
#endif
		for (int i_element = 0; i_element < element_count; i_element++){
			int y = i_element / resolution.x;
			int x = i_element - y * resolution.x;
			functor(image1_data[i_element], image2_data[i_element], x, y);
		}
	}
};

} // namespace ITMLib


// TODO