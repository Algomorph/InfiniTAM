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

namespace ITMLib {
namespace internal {
//TODO: split out the RawTwoArrayTraversalEngine into its own set of files
template<MemoryDeviceType TDeviceType, JobCountPolicy TJobCountPolicy>
class RawTwoArrayTraversalEngine_Internal;

template<MemoryDeviceType TMemoryDeviceType, JobCountPolicy TJobCountPoicy>
class TwoImageTraversalEngine_Internal;
} // namespace internal

template<MemoryDeviceType TMemoryDeviceType>
class TwoImageTraversalEngine {
public: // static functions
	template<typename TImageElement1, typename TImageElement2, typename TFunctor>
	inline static void
	Traverse(ORUtils::Image<TImageElement1>& image1, ORUtils::Image<TImageElement2>& image2, TFunctor& functor) {
		internal::TwoImageTraversalEngine_Internal<TMemoryDeviceType, EXACT>::
		template TraverseWithoutPixelCoordinate_Generic<TImageElement1, TImageElement2>(image1, image2, functor);
	}

	template<typename TImageElement1, typename TImageElement2, typename TFunctor>
	inline static void
	Traverse(ORUtils::Image<TImageElement1>& image1, const ORUtils::Image<TImageElement2>& image2, TFunctor& functor) {
		internal::TwoImageTraversalEngine_Internal<TMemoryDeviceType, EXACT>::
		template TraverseWithoutPixelCoordinate_Generic<TImageElement1, const TImageElement2>(image1, image2, functor);
	}

	template<typename TImageElement1, typename TImageElement2, typename TFunctor>
	inline static void
	Traverse(const ORUtils::Image<TImageElement1>& image1, const ORUtils::Image<TImageElement2>& image2, TFunctor& functor) {
		internal::TwoImageTraversalEngine_Internal<TMemoryDeviceType, EXACT>::
		template TraverseWithoutPixelCoordinate_Generic<const TImageElement1, const TImageElement2>(image1, image2, functor);
	}

	template<typename TImageElement1, typename TImageElement2, typename TFunctor, int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16>
	inline static void
	TraverseWithPixelCoordinates(ORUtils::Image<TImageElement1>& image1, ORUtils::Image<TImageElement2>& image2, TFunctor& functor) {
		internal::TwoImageTraversalEngine_Internal<TMemoryDeviceType, EXACT>::
		template TraverseWithPixelCoordinate_Generic<TImageElement1, TImageElement2,
				TCudaBlockSizeX, TCudaBlockSizeY>(image1, image2, functor);
	}

	template<typename TImageElement1, typename TImageElement2, typename TFunctor, int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16>
	inline static void
	TraverseWithPixelCoordinates(ORUtils::Image<TImageElement1>& image1, const ORUtils::Image<TImageElement2>& image2, TFunctor& functor) {
		internal::TwoImageTraversalEngine_Internal<TMemoryDeviceType, EXACT>::
		template TraverseWithPixelCoordinate_Generic<TImageElement1, const TImageElement2,
				TCudaBlockSizeX, TCudaBlockSizeY>(image1, image2, functor);
	}

	template<typename TImageElement1, typename TImageElement2, typename TFunctor, int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16>
	inline static void
	TraverseWithPixelCoordinates(const ORUtils::Image<TImageElement1>& image1, const ORUtils::Image<TImageElement2>& image2, TFunctor& functor) {
		internal::TwoImageTraversalEngine_Internal<TMemoryDeviceType, EXACT>::
		template TraverseWithPixelCoordinate_Generic<const TImageElement1, const TImageElement2,
				TCudaBlockSizeX, TCudaBlockSizeY>(image1, image2, functor);
	}

};
} // namespace ITMLib
