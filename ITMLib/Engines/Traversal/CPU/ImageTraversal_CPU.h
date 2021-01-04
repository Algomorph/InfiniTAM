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
#include "../Shared/TraversalMethod.h"

namespace ITMLib {
namespace internal {

template<JobCountPolicy TJobCountPolicy, TraversalMethod TTraversalMethod>
using BasicMemoryTraversalEngine = RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, TJobCountPolicy, TTraversalMethod>;

template<JobCountPolicy TJobCountPolicy>
class ImageTraversalEngine_Internal<MEMORYDEVICE_CPU, TJobCountPolicy, CONTIGUOUS>
		: private BasicMemoryTraversalEngine<TJobCountPolicy, CONTIGUOUS> {
	friend class ImageTraversalEngine<MEMORYDEVICE_CPU>;
protected: // static functions

	// =========================================== lambda-based traversal =========================================================
	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TImage, typename TLambda>
	inline static void
	TraversePositionOnly_Lambda_Generic(TImage& image, TLambda&& lambda) {
		const int element_count = static_cast<int>(image.size());
		const int image_width = image.dimensions.width;
		BasicMemoryTraversalEngine<TJobCountPolicy, CONTIGUOUS>::Traverse_Generic(
				element_count,
				[&lambda, &image_width](const int i_element) {
					int y = i_element / image_width;
					int x = i_element % image_width;
					lambda(i_element, x, y);
				});
	}


	// =========================================== dynamic-functor traversal ====================================================
	template<typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithoutPosition_Generic(TImage* image, TFunctor& functor) {
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CPU);
		const int element_count = static_cast<int>(image->size());
		BasicMemoryTraversalEngine<TJobCountPolicy, CONTIGUOUS>::TraverseWithoutIndex_Generic(image_data, functor, element_count);
	}

	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithPosition_Generic(TImage* image, TFunctor& functor) {
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CPU);
		const int element_count = static_cast<int>(image->size());
		const int image_width = image->dimensions.width;
		BasicMemoryTraversalEngine<TJobCountPolicy, CONTIGUOUS>::Traverse_Generic(
				element_count,
				[&functor, &image_data, &image_width](const int i_element) {
					const int y = i_element / image_width;
					const int x = i_element % image_width;
					functor(image_data[i_element], x, y);
				});
	}

	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraversePositionOnly_Generic(TImage* image, TFunctor& functor) {
		const int element_count = static_cast<int>(image->size());
		const int image_width = image->dimensions.width;
		BasicMemoryTraversalEngine<TJobCountPolicy, CONTIGUOUS>::Traverse_Generic(
				element_count,
				[&functor, &image_width](const int i_element) {
					int y = i_element / image_width;
					int x = i_element % image_width;
					functor(i_element, x, y);
				});
	}
};


template<JobCountPolicy TJobCountPolicy>
class ImageTraversalEngine_Internal<MEMORYDEVICE_CPU, TJobCountPolicy, INDEX_SAMPLE>
		: private BasicMemoryTraversalEngine<TJobCountPolicy, INDEX_SAMPLE> {
	friend class ImageTraversalEngine<MEMORYDEVICE_CPU>;
protected: // static functions

	template<typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithoutPosition_Generic(const int sample_size, const int* sample_pixel_indices, TImage* image, TFunctor& functor) {
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CPU);
		BasicMemoryTraversalEngine<TJobCountPolicy, INDEX_SAMPLE>::Traverse_Generic(sample_size, sample_pixel_indices, image_data, functor);
	}

	template<typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithPosition_Generic(const int sample_size, const int* sample_pixel_indices, TImage* image, TFunctor& functor) {
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CPU);
		const int image_width = image->dimensions.width;
		BasicMemoryTraversalEngine<TJobCountPolicy, INDEX_SAMPLE>::Traverse_Generic(
				sample_size, sample_pixel_indices,
				[&functor, &image_data, &image_width](const int pixel_index) {
					const int y = pixel_index / image_width;
					const int x = pixel_index % image_width;
					functor(image_data[pixel_index], x, y);
				});
	}

	template<typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraversePositionOnly_Generic(const int sample_size, const int* sample_pixel_indices, TImage* image, TFunctor& functor) {
		const int image_width = image->dimensions.width;
		BasicMemoryTraversalEngine<TJobCountPolicy, INDEX_SAMPLE>::Traverse_Generic(
				sample_size, sample_pixel_indices,
				[&functor, &image_width](const int pixel_index) {
					const int y = pixel_index / image_width;
					const int x = pixel_index % image_width;
					functor(pixel_index, x, y);
				});
	}
};

} // namespace internal
} // namespace ITMLib