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
#include "RawArrayTraversal_CPU.h"
#include "../Shared/JobCountPolicy.h"

namespace ITMLib {
namespace internal {

template<JobCountPolicy TJobCountPolicy>
using BasicMemoryTraversalEngine = RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, TJobCountPolicy, CONTIGUOUS>;

template<JobCountPolicy TJobCountPolicy>
class ImageTraversalEngine_Internal<MEMORYDEVICE_CPU, TJobCountPolicy>
		: private BasicMemoryTraversalEngine<TJobCountPolicy> {
	friend class ImageTraversalEngine<MEMORYDEVICE_CPU>;
protected: // static functions
	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithPosition_Generic(TImage* image, TFunctor& functor) {
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CPU);
		const Vector2i& resolution = image->dimensions;
		const unsigned int element_count = image->size();
		BasicMemoryTraversalEngine<TJobCountPolicy>::Traverse_Generic(
				element_count,
				[&functor, &image_data, &resolution](const int i_element) {
					const int y = i_element / resolution.x;
					const int x = i_element - y * resolution.x;
					functor(image_data[i_element], x, y);
				});
	}

	template<typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithoutPosition_Generic(TImage* image, TFunctor& functor) {
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CPU);
		const unsigned int element_count = image->size();
		BasicMemoryTraversalEngine<TJobCountPolicy>::Traverse_Generic(
				element_count,
				[&functor, &image_data](const int i_element) {
					functor(image_data[i_element]);
				});
	}

	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraversePositionOnly_Generic(TImage* image, TFunctor& functor) {
		const Vector2i& resolution = image->dimensions;
		const unsigned int element_count = image->size();
		BasicMemoryTraversalEngine<TJobCountPolicy>::Traverse_Generic(
				element_count,
				[&functor, &resolution](const int i_element) {
					int y = i_element / resolution.x;
					int x = i_element - y * resolution.x;
					functor(i_element, x, y);
				});
	}
};

} // namespace internal
} // namespace ITMLib