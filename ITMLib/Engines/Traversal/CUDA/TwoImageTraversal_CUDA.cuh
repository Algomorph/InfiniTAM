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
#include "TwoImageTraversal_CUDA_Kernels.h"
#include "../Interface/TwoImageTraversal.h"
#include "../../../Utils/Math.h"
#include "../../../../ORUtils/Image.h"

namespace ITMLib {

namespace internal {
template<>
class RawTwoArrayTraversalEngine_Internal<MEMORYDEVICE_CUDA, JobCountPolicy::EXACT> {
protected: // static functions
	template<typename TData, typename TFunctor>
	inline static void TraverseWithoutIndex_Generic(TData* data, TFunctor& functor, const int element_count) {
		CallCUDAonUploadedFunctor(
				functor,
				[&element_count, &data](TFunctor* functor_device) {
					dim3 cuda_block_size(256);
					dim3 cuda_grid_size(ceil_of_integer_quotient(element_count, cuda_block_size.x));
					TraverseTwoArraysWithoutItemIndex_device<<<cuda_grid_size, cuda_block_size >>>(data, element_count, functor_device);
				}
		);
	}

	template<typename TData, typename TFunctor>
	inline static void TraverseWithIndex_Generic(TData* data, TFunctor& functor, const int element_count) {
		CallCUDAonUploadedFunctor(
				functor,
				[&element_count, &data](TFunctor* functor_device) {
					dim3 cuda_block_size(256);
					dim3 cuda_grid_size(ceil_of_integer_quotient(element_count, cuda_block_size.x));
					TraverseTwoArraysWithIndex_device<<<cuda_grid_size, cuda_block_size >>>(data, element_count, functor_device);
				}
		);
	}
};

template<>
class TwoImageTraversalEngine_Internal<MEMORYDEVICE_CUDA, EXACT> :
		RawTwoArrayTraversalEngine_Internal<MEMORYDEVICE_CUDA, EXACT> {
	friend class TwoImageTraversalEngine<MEMORYDEVICE_CUDA>;
protected:// static functions
	template<typename TImageElement1, typename TImageElement2, typename TImage1, typename TImage2, typename TFunctor>
	inline static void
	TraverseWithoutPixelCoordinate_Generic(TImage1& image_1, TImage2& image_2, TFunctor& functor) {
		TImageElement1* image_data1 = image_1.GetData(MEMORYDEVICE_CUDA);
		TImageElement2* image_data2 = image_2.GetData(MEMORYDEVICE_CUDA);
		assert(image_1.size() == image_2.size());
		const int pixel_count = static_cast<int>(image_1.size());
		RawTwoArrayTraversalEngine_Internal<MEMORYDEVICE_CUDA, EXACT>::
		        TraverseWithoutIndex_Generic(image_data1, image_data2, functor, pixel_count);
	}
	template<typename TImageElement1, typename TImageElement2,
			int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16,
			typename TImage1, typename TImage2, typename TFunctor>
	inline static void
	TraverseWithPixelCoordinate_Generic(TImage1& image_1, TImage2& image_2, TFunctor& functor) {
		TImageElement1* image_data1 = image_1.GetData(MEMORYDEVICE_CUDA);
		TImageElement2* image_data2 = image_2.GetData(MEMORYDEVICE_CUDA);
		const int element_count = static_cast<int>(image_1.size());
		assert(image_1.dimensions == image_2.dimensions);
		const Vector2i resolution = image_1.dimensions;
		CallCUDAonUploadedFunctor(
				functor,
				[&image_data1, &image_data2, &resolution](TFunctor* functor_device) {
					dim3 cuda_block_size(TCudaBlockSizeX, TCudaBlockSizeY);
					dim3 cuda_grid_size(ceil_of_integer_quotient(resolution.x, cuda_block_size.x),
					                    ceil_of_integer_quotient(resolution.y, cuda_block_size.y));
					TraverseWithPixelCoordinate_device <<< cuda_grid_size, cuda_block_size >>>(image_data1, image_data2, resolution, functor_device);
				}
		);
	}
};

} // namespace internal

// template<typename TImage1Element, typename TImage2Element>
// class TwoImageTraversalEngine<TImage1Element, TImage2Element, MEMORYDEVICE_CUDA> {
// public:
// 	template<typename TFunctor>
// 	inline static void
// 	TraverseWithPosition(const ORUtils::Image<TImage1Element>& image1, const ORUtils::Image<TImage2Element>& image2, TFunctor& functor) {
// 		const Vector2i resolution = image1.dimensions;
// 		const TImage1Element* image1_data = image1.GetData(MEMORYDEVICE_CUDA);
// 		const TImage2Element* image2_data = image2.GetData(MEMORYDEVICE_CUDA);
//
// 		TFunctor* functor_device = nullptr;
//
// 		dim3 cuda_block_size(16, 16);
// 		dim3 cuda_grid_size((int) ceil((float) resolution.x / (float) cuda_block_size.x),
// 		                    (int) ceil((float) resolution.y / (float) cuda_block_size.y));
//
// 		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
// 		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));
//
// 		twoImageTraversalWithPosition_device<TImage1Element, TImage2Element, TFunctor>
// 		        <<< cuda_grid_size, cuda_block_size >>>
// 		                             (image1_data, image2_data, resolution, functor_device);
// 		ORcudaKernelCheck;
//
// 		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
// 		ORcudaSafeCall(cudaFree(functor_device));
// 	}
// };

} // namespace ITMLib