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
#include "../../../Objects/Volume/VoxelBlockHash.h"

namespace {
// CUDA global kernels

template <typename TFunctor>
__global__ void hashTableAllEntryTraversalWithHashCode_device (HashEntry* hash_table, const int hash_entry_count, TFunctor* functor_device){
	int hash_code = threadIdx.x + blockIdx.x * blockDim.x;
	if (hash_code >= hash_entry_count) return;
	(*functor_device)(hash_table[hash_code], hash_code);
}

template <typename TFunctor>
__global__ void hashTableUtilizedEntryTraversalWithHashCode_device (HashEntry* hash_table, int* utilized_hash_codes, const int utilized_entry_count, TFunctor* functor_device){
	int hash_code_index = threadIdx.x + blockIdx.x * blockDim.x;
	if (hash_code_index >= utilized_entry_count) return;
	int hash_code = utilized_hash_codes[hash_code_index];
	(*functor_device)(hash_table[hash_code], hash_code);
}

template <typename TFunctor>
__global__ void hashTableUtilizedEntryTraversalWithHashCode_device (const HashEntry* hash_table, const int* utilized_hash_codes, const int utilized_entry_count, TFunctor* functor_device){
	int hash_code_index = threadIdx.x + blockIdx.x * blockDim.x;
	if (hash_code_index >= utilized_entry_count) return;
	int hash_code = utilized_hash_codes[hash_code_index];
	(*functor_device)(hash_table[hash_code], hash_code);
}

} // end anonymous namespace (CUDA global kernels)
