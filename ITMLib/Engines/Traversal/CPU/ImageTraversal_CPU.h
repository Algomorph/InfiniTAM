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
#include "../Interface/ImageTraversal.h"
#include "../../../Utils/Math.h"
#include "../../../../ORUtils/Image.h"

namespace ITMLib {
namespace internal {
template<>
class ImageTraversalEngine_Internal<MEMORYDEVICE_CPU> {
	friend class ImageTraversalEngine<MEMORYDEVICE_CPU>;
protected: // static functions
	template<typename TImageElement, typename TImage, typename TApplyFunction>
	inline static void
	Traverse_Generic(TImage* image, TApplyFunction&& apply_function) {
		const Vector2i resolution = image->dimensions;
		const int element_count = resolution.x * resolution.y;
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CPU);
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(apply_function, image_data)
#endif
		for (int i_element = 0; i_element < element_count; i_element++) {
			apply_function(image_data, i_element, resolution);
		}
	}

	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithPosition_Generic(TImage* image, TFunctor& functor) {
		Traverse_Generic<TImageElement>(
				image,
				[&functor](TImageElement* image_data, int i_element, const Vector2i& resolution) {
					int y = i_element / resolution.x;
					int x = i_element - y * resolution.x;
					functor(image_data[i_element], x, y);
				});
	}

	template<typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithoutPosition_Generic(TImage* image, TFunctor& functor) {
		Traverse_Generic<TImageElement>(
				image,
				[&functor](TImageElement* image_data, int i_element, const Vector2i& resolution) {
					functor(image_data[i_element]);
				});
	}
};
} // namespace internal
} // namespace ITMLib