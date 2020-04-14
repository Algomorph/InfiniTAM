//  ================================================================
//  Created by Gregory Kramida on 3/2/20.
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


#include <curand_kernel.h>



namespace { // (CUDA global kernels)

__global__
void initialize_random_numbers_for_spatial_blocks(curandState* random_states){
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	curand_init(1337, idx, 0, &random_states[idx]);
}


} // end anonymous namespace (CUDA global kernels)