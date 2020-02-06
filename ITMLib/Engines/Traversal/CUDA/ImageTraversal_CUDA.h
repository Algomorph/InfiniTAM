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
#include "ImageTraversal_CUDA_Kernels.h"
#include "../Interface/ImageTraversal.h"
#include "../../../Utils/Math.h"
#include "../../../../ORUtils/Image.h"

namespace ITMLib {

template<typename TImageElement>
class ImageTraversalEngine<TImageElement, MEMORYDEVICE_CUDA> {
public:
	template<typename TFunctor>
	inline static void
	TraverseWithPosition(ORUtils::Image<TImageElement>* image, TFunctor& functor) {
		const Vector2i resolution = image->noDims;
		const TImageElement* image_data = image->GetData(MEMORYDEVICE_CUDA);

		TFunctor* functor_device = nullptr;

		dim3 cuda_block_size(16, 16);
		dim3 cuda_grid_size((int) ceil((float) resolution.x / (float) cuda_block_size.x),
		                    (int) ceil((float) resolution.y / (float) cuda_block_size.y));

		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		imageTraversalWithPosition_device<TImageElement, TFunctor>
				<< < cuda_grid_size, cuda_block_size >> >
		                             (image_data, resolution, functor_device);
		ORcudaKernelCheck;

		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}
};

} // namespace ITMLib