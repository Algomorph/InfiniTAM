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
#include "../Interface/HashTableTraversal.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "HashTableTraversal_CUDA_Kernels.h"

namespace ITMLib {

template<>
class HashTableTraversalEngine<MEMORYDEVICE_CUDA> {
public:
	template<typename TFunctor>
	inline static void
	TraverseWithHashCode(VoxelBlockHash& index, TFunctor& functor) {
		HashEntry* hash_table = index.GetEntries();
		const int hash_entry_count = index.hashEntryCount;
		TFunctor* functor_device = nullptr;

		dim3 cuda_block_size(256, 1);
		dim3 cuda_grid_size((int) ceil((float) hash_entry_count / (float) cuda_block_size.x));

		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		hashTableTraversalWithHashCode_device < TFunctor >
		<< <cuda_grid_size, cuda_block_size >> >
		(hash_table, hash_entry_count, functor_device);
		ORcudaKernelCheck;

		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));

	}
};

} // namespace ITMLib


// TODO