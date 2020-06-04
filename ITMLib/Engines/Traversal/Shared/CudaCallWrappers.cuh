//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 6/3/20.
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

#include "../../../../ORUtils/CUDADefines.h"

namespace ITMLib {
namespace internal {
template<typename TFunctor, typename TCudaCall>
inline static void CUDA_CallWithFunctor_Generic(TFunctor& functor, TCudaCall&& cuda_call) {
	TFunctor* functor_device;

	ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
	ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

	cuda_call(functor_device);
	ORcudaKernelCheck;

	ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
	ORcudaSafeCall(cudaFree(functor_device));
}
} // namespace internal
} // namespace ITMLib
