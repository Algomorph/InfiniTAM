//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 1/5/21.
//  Copyright (c) 2021 Gregory Kramida
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
#include "CountValidDepths.h"
#include "../../Utils/CUDA/CUDAUtils.h"

namespace { // CUDA kernels (anonymous namespace)
__global__ void countValidDepths_device(const float *imageData_in, int imgSizeTotal, int *counterTempData_device)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;
	int locId_local = threadIdx.x;

	__shared__ int dim_shared[256]; // What the hell is "dim"? Who ever coded this was dim. Juuuuust kidding.
	//__shared__ bool should_prefix;

	//should_prefix = false;
	//__syncthreads();

	bool isValidPoint = false;

	if (i < imgSizeTotal)
	{
		if (imageData_in[i] > 0.0f) isValidPoint = true;
	}

	//__syncthreads();
	//if (!should_prefix) return;

	dim_shared[locId_local] = isValidPoint;
	__syncthreads();

	if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
	__syncthreads();
	if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
	__syncthreads();

	if (locId_local < 32) warpReduce(dim_shared, locId_local);

	if (locId_local == 0) atomicAdd(counterTempData_device, dim_shared[locId_local]);
}
} // CUDA kernels (anonymous namespace)

namespace ITMLib::internal {


template<>
int CountValidDepths<MEMORYDEVICE_CUDA>(const FloatImage& image_in){
	const float* image_data = image_in.GetData(MEMORYDEVICE_CUDA);

	dim3 cuda_block_size(256);
	dim3 cuda_grid_size(ITMLib::ceil_of_integer_quotient(static_cast<unsigned int>(image_in.size()), cuda_block_size.x));

	ORUtils::MemoryBlock<int> valid_depth_count(1,true, true);
	valid_depth_count.Clear();

	countValidDepths_device <<<cuda_grid_size, cuda_block_size>>>(image_data, image_in.size(), valid_depth_count.GetData(MEMORYDEVICE_CUDA));
	ORcudaKernelCheck;

	valid_depth_count.UpdateHostFromDevice();

	return *valid_depth_count.GetData(MEMORYDEVICE_CPU);
};

} // namespace ITMLib::internal