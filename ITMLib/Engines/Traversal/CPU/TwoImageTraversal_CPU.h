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


namespace internal {

template<>
class RawTwoArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, JobCountPolicy::EXACT> {

protected: // static functions
	template<typename TApplyFunction>
	inline static void Traverse_Generic(const int element_count, TApplyFunction&& apply_function) {
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(apply_function) firstprivate(element_count)
#endif
		for (int i_item = 0; i_item < element_count; i_item++) {
			apply_function(i_item);
		}
	}

	template<typename TData1, typename TData2, typename TFunctor>
	inline static void TraverseWithIndex_Generic(TData1* data_1, TData2* data_2, TFunctor& functor, const int element_count) {
		Traverse_Generic(element_count, [&functor, &data_1, &data_2](int i_item) { functor(data_1[i_item], data_2[i_item], i_item); });
	}

	template<typename TData1, typename TData2, typename TFunctor>
	inline static void TraverseWithoutIndex_Generic(TData1* data_1, TData2* data_2, TFunctor& functor, const int element_count) {
		Traverse_Generic(element_count, [&functor, &data_1, &data_2](int i_item) { functor(data_1[i_item], data_2[i_item]); });
	}
};

template<JobCountPolicy TJobCountPolicy>
class TwoImageTraversalEngine_Internal<MEMORYDEVICE_CPU, TJobCountPolicy> :
		RawTwoArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, TJobCountPolicy> {
	friend class TwoImageTraversalEngine<MEMORYDEVICE_CPU>;
protected:// static functions
	template<typename TImageElement1, typename TImageElement2, typename TImage1, typename TImage2, typename TFunctor>
	inline static void
	TraverseWithoutPixelCoordinate_Generic(TImage1& image_1, TImage2& image_2, TFunctor& functor) {
		TImageElement1* image_data1 = image_1.GetData(MEMORYDEVICE_CPU);
		TImageElement2* image_data2 = image_2.GetData(MEMORYDEVICE_CPU);
		assert(image_1.size() == image_2.size());
		const int element_count = static_cast<int>(image_1.size());
		RawTwoArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, TJobCountPolicy>::TraverseWithoutIndex_Generic(
				image_data1, image_data2, functor, element_count);
	}

	template<typename TImageElement1, typename TImageElement2, int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16,
			typename TImage1, typename TImage2, typename TFunctor>
	inline static void
	TraverseWithPixelCoordinate_Generic(TImage1& image_1, TImage2& image_2, TFunctor& functor) {
		TImageElement1* image_data1 = image_1.GetData(MEMORYDEVICE_CPU);
		TImageElement2* image_data2 = image_2.GetData(MEMORYDEVICE_CPU);
		const int element_count = static_cast<int>(image_1.size());
		const int image_width = image_1.dimensions.width;
		assert(image_1.dimensions == image_2.dimensions);
		RawTwoArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, TJobCountPolicy>::Traverse_Generic(
				element_count,
				[&functor, &image_data1, &image_data2, &image_width](const int i_element) {
					const int y = i_element / image_width;
					const int x = i_element % image_width;
					functor(image_data1[i_element], image_data2[i_element], x, y);
				});
	}
};

} // namespace internal


// template<typename TImage1Element, typename TImage2Element>
// class TwoImageTraversalEngine<TImage1Element, TImage2Element, MEMORYDEVICE_CPU> {
// public:
// 	template<typename TFunctor>
// 	inline static void
// 	TraverseWithPosition(const ORUtils::Image<TImage1Element>& image1, const ORUtils::Image<TImage2Element>& image2, TFunctor& functor){
//
// 		const Vector2i resolution = image1.dimensions;
// 		const int element_count = resolution.x * resolution.y;
//
// 		const TImage1Element* image1_data = image1.GetData(MEMORYDEVICE_CPU);
// 		const TImage2Element* image2_data = image2.GetData(MEMORYDEVICE_CPU);
// #ifdef WITH_OPENMP
// 	#pragma omp parallel for default(none) shared(functor, image1_data, image2_data) firstprivate(element_count) \
// 	firstprivate(resolution)
// #endif
// 		for (int i_element = 0; i_element < element_count; i_element++){
// 			int y = i_element / resolution.x;
// 			int x = i_element - y * resolution.x;
// 			functor(image1_data[i_element], image2_data[i_element], x, y);
// 		}
// 	}
// };

} // namespace ITMLib
