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
#pragma once

#include "../../../../ORUtils/MemoryDeviceType.h"
#include "../Shared/JobCountPolicy.h"
#include "../Shared/TraversalMethod.h"

namespace ITMLib {
namespace internal {
template<MemoryDeviceType TMemoryDeviceType, JobCountPolicy TJobCountPoicy, TraversalMethod TTraversalMethod>
class ImageTraversalEngine_Internal;
} // namespace internal

template<MemoryDeviceType TMemoryDeviceType>
class ImageTraversalEngine {
public: // static functions
	template<typename TImageElement, typename TFunctor>
	inline static void
	Traverse(ORUtils::Image<TImageElement>* image, TFunctor& functor) {
		internal::ImageTraversalEngine_Internal<TMemoryDeviceType, EXACT, CONTIGUOUS>::
		        template TraverseWithoutPosition_Generic<TImageElement>(image, functor);
	}

	template<typename TImageElement, typename TFunctor>
	inline static void
	Traverse(const ORUtils::Image<TImageElement>* image, TFunctor& functor) {
		internal::ImageTraversalEngine_Internal<TMemoryDeviceType, EXACT, CONTIGUOUS>::template TraverseWithoutPosition_Generic<const TImageElement>(
				image, functor);
	}
	//TODO: for clarity, refactor TraverseWithPosition --> TraverseWithPixelCoordinates (also in internal classes employed here)
	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TFunctor>
	inline static void
	TraverseWithPosition(ORUtils::Image<TImageElement>* image, TFunctor& functor) {
		internal::ImageTraversalEngine_Internal<TMemoryDeviceType, EXACT, CONTIGUOUS>::template TraverseWithPosition_Generic<TCudaBlockSizeX, TCudaBlockSizeY, TImageElement>(
				image, functor);
	}

	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TFunctor>
	inline static void
	TraverseWithPosition(const ORUtils::Image<TImageElement>* image, TFunctor& functor) {
		internal::ImageTraversalEngine_Internal<TMemoryDeviceType, EXACT, CONTIGUOUS>::template TraverseWithPosition_Generic<TCudaBlockSizeX, TCudaBlockSizeY, const TImageElement>(
				image, functor);
	}

	//TODO: for clarity, refactor TraversePositionOnly --> TraversePixelIndexAndCoordinatesOnly (also in internal classes employed here)
	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TFunctor>
	inline static void
	TraversePositionOnly(ORUtils::Image<TImageElement>* image, TFunctor& functor) {
		internal::ImageTraversalEngine_Internal<TMemoryDeviceType, EXACT, CONTIGUOUS>::template TraversePositionOnly_Generic<TCudaBlockSizeX, TCudaBlockSizeY, TImageElement>(
				image, functor);
	}

	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TFunctor>
	inline static void
	TraversePositionOnly(const ORUtils::Image<TImageElement>* image, TFunctor& functor) {
		internal::ImageTraversalEngine_Internal<TMemoryDeviceType, EXACT, CONTIGUOUS>::template TraversePositionOnly_Generic<TCudaBlockSizeX, TCudaBlockSizeY, const TImageElement>(
				image, functor);
	}

	template<typename TImageElement, typename TFunctor>
	inline static void
	TraverseSample(const int sample_size, const int* sample_pixel_indices, ORUtils::Image<TImageElement>* image, TFunctor& functor) {
		internal::ImageTraversalEngine_Internal<TMemoryDeviceType, EXACT, INDEX_SAMPLE>::template
		TraverseWithoutPosition_Generic<TImageElement>(sample_size, sample_pixel_indices, image, functor);
	}
	template<typename TImageElement, typename TFunctor>
	inline static void
	TraverseSampleWithPixelCoordinates(const int sample_size, const int* sample_pixel_indices, ORUtils::Image<TImageElement>* image, TFunctor& functor) {
		internal::ImageTraversalEngine_Internal<TMemoryDeviceType, EXACT, INDEX_SAMPLE>::template
		TraverseWithPosition_Generic<TImageElement>(sample_size, sample_pixel_indices, image, functor);
	}

	template<typename TImageElement, typename TFunctor>
	inline static void
	TraverseSamplePixelIndexAndCoordinatesOnly(const int sample_size, const int* sample_pixel_indices, const ORUtils::Image<TImageElement>* image, TFunctor& functor) {
		internal::ImageTraversalEngine_Internal<TMemoryDeviceType, EXACT, INDEX_SAMPLE>::template
		TraversePositionOnly_Generic<const TImageElement>(sample_size, sample_pixel_indices, image, functor);
	}

};
} // namespace ITMLib
