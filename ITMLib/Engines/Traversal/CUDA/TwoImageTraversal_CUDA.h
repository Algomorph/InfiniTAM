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

template<typename TImage1Element, typename TImage2Element>
class TwoImageTraversalEngine<TImage1Element, TImage2Element, MEMORYDEVICE_CUDA> {
public:
	template<typename TFunctor>
	inline static void
	TraverseWithPosition(ORUtils::Image<TImage1Element>* image1, ORUtils::Image<TImage2Element>* image2, TFunctor& functor) {
		const Vector2i resolution = image1->noDims;
		const TImage1Element* image1_data = image1->GetData(MEMORYDEVICE_CUDA);
		const TImage2Element* image2_data = image2->GetData(MEMORYDEVICE_CUDA);

		TFunctor* functor_device = nullptr;

		dim3 cuda_block_size(16, 16);
		dim3 cuda_grid_size((int) ceil((float) resolution.x / (float) cuda_block_size.x),
		                    (int) ceil((float) resolution.y / (float) cuda_block_size.y));

		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		twoImageTraversalWithPosition_device<TImage1Element, TImage2Element, TFunctor>
				<< < cuda_grid_size, cuda_block_size >> >
		                             (image1_data, image2_data, resolution, functor_device);
		ORcudaKernelCheck;

		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}
};

} // namespace ITMLib