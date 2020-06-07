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
#include "../Interface/ImageTraversal.h"
#include "../Shared/CudaCallWrappers.cuh"
#include "../../../Utils/Math.h"
#include "../../../../ORUtils/Image.h"

namespace ITMLib {
namespace internal {
template<>
class ImageTraversalEngine_Internal<MEMORYDEVICE_CUDA, EXACT> {
	friend class ImageTraversalEngine<MEMORYDEVICE_CUDA>;
protected: // static functions

	template<int TCudaBlockSizeX = 16, int TCudaBlockSizeY = 16, typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithPosition_Generic(TImage* image, TFunctor& functor) {
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CUDA);
		const Vector2i resolution = image->dimensions;
		CallCUDAonUploadedFunctor(functor,
				[&image_data, &resolution](TFunctor* functor_device) {
					dim3 cuda_block_size(TCudaBlockSizeX, TCudaBlockSizeY);
					dim3 cuda_grid_size(ceil_of_integer_quotient(resolution.x, cuda_block_size.x),
					                    ceil_of_integer_quotient(resolution.y, cuda_block_size.y));
					ImageTraversalWithPosition_device<TImageElement, TFunctor>
					<<< cuda_grid_size, cuda_block_size >>>
							(image_data, resolution, functor_device);
				}
		);
	}

	template<typename TImageElement, typename TImage, typename TFunctor>
	inline static void
	TraverseWithoutPosition_Generic(TImage* image, TFunctor& functor) {
		TImageElement* image_data = image->GetData(MEMORYDEVICE_CUDA);
		const int pixel_count = image->size();
		CallCUDAonUploadedFunctor(functor,
				[&image_data, &pixel_count](TFunctor* functor_device) {
					dim3 cuda_block_size(256);
					dim3 cuda_grid_size(ceil_of_integer_quotient(pixel_count, cuda_block_size.x));
					ImageTraversalWithoutPosition_device<TImageElement, TFunctor>
					<<< cuda_grid_size, cuda_block_size >>>
							(image_data, pixel_count, functor_device);
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
					ImagePositionOnlyTraversal_device<TImageElement, TFunctor> <<< cuda_grid_size, cuda_block_size >>>(resolution, functor_device);
				}
		);
	}
};
} // namespace internal
} // namespace ITMLib