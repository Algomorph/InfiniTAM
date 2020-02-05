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

#include "../../../Utils/Math.h"

namespace {

// CUDA global kernels
template <typename TImage1Element, typename TImage2Element, typename TFunctor >
__global__ void twoImageTraversalWithPosition_device (const TImage1Element* image1_data, const TImage1Element* image2_data, const Vector2i resolution, TFunctor* functor_device){
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
	if (x > resolution.x - 1 || y > resolution.y - 1) return;
	functor_device(image1_data[x + y * resolution.x], image2_data[x + y * resolution.x], x, y);
}

} // end anonymous namespace (CUDA global kernels)
