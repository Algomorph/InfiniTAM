//  ================================================================
//  Created by Gregory Kramida on 2/6/20.
//  Copyright (c)  2020 Gregory Kramida
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

//local
#include "AtomicArrayThreadGuard.h"
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/MemoryBlock.h"
#include "../../../Utils/CUDAUtils.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"

namespace ITMLib {

struct CUDA_Semaphore {
	__device__ volatile char sem = 0;

	__device__ void acquire_semaphore() {
		while (atomicCAS((char*) &sem, 0, 1) != 0);
	}

	__device__ void release_semaphore() {
		sem = 0;
		__threadfence();
	}
};

template<>
class AtomicArrayThreadGuard<MEMORYDEVICE_CUDA> {
public:
	explicit AtomicArrayThreadGuard(int item_count) {
		ORcudaSafeCall(cudaMalloc((void**) &semaphores, sizeof(CUDA_Semaphore) * item_count));
		ORcudaSafeCall(cudaMemset(semaphores, 0, sizeof(CUDA_Semaphore) * item_count));
	}

	~AtomicArrayThreadGuard() {
		ORcudaSafeCall(cudaFree(semaphores));
	}

	__device__
	void lock(int item_number) {
		extern __shared__ char lock_array[];
		while (atomicCAS(lock_array + item_number, (char) 0, (char) 1) != 0) {};
	}

	__device__
	void release(int item_number) {
		extern __shared__ char lock_array[];
		lock_array[item_number] = 0;
	}

private:
	CUDA_Semaphore* semaphores;
};
} // namespace ITMLib


