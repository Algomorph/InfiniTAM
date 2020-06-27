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
#include "ImageTraversal_CUDA_Kernels.cuh"
#include "RawArrayTraversal_CUDA.cuh"
#include "../Interface/ImageTraversal.h"
#include "../Shared/CudaCallWrappers.cuh"
#include "../../../Utils/Math.h"
#include "../../../../ORUtils/Image.h"

namespace ITMLib {
namespace internal {
template<JobCountPolicy TJobCountPolicy, TraversalMethod TTraversalMethod>
using BasicMemoryTraversalEngine = RawArrayTraversalEngine_Internal<MEMORYDEVICE_CUDA, TJobCountPolicy, TTraversalMethod>;

template<>
class ImageTraversalEngine_Internal<MEMORYDEVICE_CUDA, EXACT, CONTIGUOUS>
		: private BasicMemoryTraversalEngine<EXACT, CONTIGUOUS> {
	friend class ImageTraversalEngine<MEMORYDEVICE_CUDA>;
protected: // static functions

	template<typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithoutPosition_Generic(TImage* image, TFunctor& functor) {
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CUDA);
		const int pixel_count = static_cast<int>(image->size());
		BasicMemoryTraversalEngine<EXACT, CONTIGUOUS>::TraverseWithoutIndex_Generic(image_data, functor, pixel_count);
	}

	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithPosition_Generic(TImage* image, TFunctor& functor) {
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CUDA);
		const Vector2i resolution = image->dimensions;
		CallCUDAonUploadedFunctor(
				functor,
				[&image_data, &resolution](TFunctor* functor_device) {
					dim3 cuda_block_size(TCudaBlockSizeX, TCudaBlockSizeY);
					dim3 cuda_grid_size(ceil_of_integer_quotient(resolution.x, cuda_block_size.x),
					                    ceil_of_integer_quotient(resolution.y, cuda_block_size.y));
					TraverseWithPosition_device <<< cuda_grid_size, cuda_block_size >>>(image_data, resolution, functor_device);
				}
		);
	}

	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraversePositionOnly_Generic(TImage* image, TFunctor& functor) {
		const Vector2i resolution = image->dimensions;
		CallCUDAonUploadedFunctor(
				functor,
				[&resolution](TFunctor* functor_device) {
					dim3 cuda_block_size(TCudaBlockSizeX, TCudaBlockSizeY);
					dim3 cuda_grid_size(ceil_of_integer_quotient(resolution.x, cuda_block_size.x),
					                    ceil_of_integer_quotient(resolution.y, cuda_block_size.y));
					TraversePositionOnly_device <<< cuda_grid_size, cuda_block_size >>>(resolution, functor_device);
				}
		);
	}
};

template<>
class ImageTraversalEngine_Internal<MEMORYDEVICE_CUDA, EXACT, INDEX_SAMPLE>
		: private BasicMemoryTraversalEngine<EXACT, INDEX_SAMPLE> {
	friend class ImageTraversalEngine<MEMORYDEVICE_CUDA>;
protected: // static functions

	template<typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithoutPosition_Generic(const int sample_size, const int* sample_pixel_indices, TImage* image, TFunctor& functor) {
		BasicMemoryTraversalEngine<EXACT, INDEX_SAMPLE>::TraverseWithoutIndex_Generic(
				sample_size, sample_pixel_indices, image->GetData(MEMORYDEVICE_CPU), functor);
	}

	template<typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithPosition_Generic(const int sample_size, const int* sample_pixel_indices, TImage* image, TFunctor& functor) {
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CUDA);
		const int image_width = image->dimensions.width;
		CallCUDAonUploadedFunctor(
				functor,
				[&image_data, &image_width, &sample_size, &sample_pixel_indices](TFunctor* functor_device) {
					dim3 cuda_block_size(256);
					dim3 cuda_grid_size(ceil_of_integer_quotient(sample_size, cuda_block_size.x));
					TraverseWithPosition_device <<< cuda_grid_size, cuda_block_size >>>(image_data, sample_pixel_indices, sample_size, image_width, functor_device);
				}
		);
	}

	template<typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraversePositionOnly_Generic(const int sample_size, const int* sample_pixel_indices, TImage* image, TFunctor& functor) {
		const int image_width = image->dimensions.width;
		CallCUDAonUploadedFunctor(
				functor,
				[&image_width, &sample_size, &sample_pixel_indices](TFunctor* functor_device) {
					dim3 cuda_block_size(256);
					dim3 cuda_grid_size(ceil_of_integer_quotient(sample_size, cuda_block_size.x));
					TraversePositionOnly_device <<< cuda_grid_size, cuda_block_size >>>(sample_pixel_indices, sample_size, image_width, functor_device);
				}
		);
	}
};
} // namespace internal
} // namespace ITMLib