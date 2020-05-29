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
//local
#include "../Interface/HashTableTraversal.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"

namespace ITMLib {

template<>
class HashTableTraversalEngine<MEMORYDEVICE_CPU> {
private: // static functions
	template<typename TVoxelBlockHash, typename THashEntry, typename TFunctor>
	inline static void TraverseAllWithHashCode_Generic(TVoxelBlockHash& index, TFunctor& functor){
		THashEntry* hash_table = index.GetEntries();
		const int hash_entry_count = index.hash_entry_count;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor, hash_table)
#endif
		for (int hash_code = 0; hash_code < hash_entry_count; hash_code++){
			functor(hash_table[hash_code], hash_code);
		}
	}

	template<typename TVoxelBlockHash, typename THashEntry, typename TFunctor>
	inline static void TraverseUtilizedWithHashCode_Generic(TVoxelBlockHash& index, TFunctor& functor){
		THashEntry* hash_table = index.GetEntries();
		const int utilized_entry_count = index.GetUtilizedBlockCount();
		const int* utilized_entry_codes = index.GetUtilizedBlockHashCodes();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor, hash_table, utilized_entry_codes)
#endif
		for (int hash_code_index = 0; hash_code_index < utilized_entry_count; hash_code_index++){
			int hash_code = utilized_entry_codes[hash_code_index];
			functor(hash_table[hash_code], hash_code);
		}
	}

	template<typename TVoxelBlockHash, typename THashEntry, typename TFunctor>
	inline static void TraverseVisibleWithHashCode_Generic(TVoxelBlockHash& index, TFunctor& functor){
		THashEntry* hash_table = index.GetEntries();
		const int visible_entry_count = index.GetVisibleBlockCount();
		const int* visible_entry_codes = index.GetVisibleBlockHashCodes();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor, hash_table, visible_entry_codes)
#endif
		for (int hash_code_index = 0; hash_code_index < visible_entry_count; hash_code_index++){
			int hash_code = visible_entry_codes[hash_code_index];
			functor(hash_table[hash_code], hash_code);
		}
	}

public: // static functions
	template<typename TFunctor>
	inline static void
	TraverseAllWithHashCode(VoxelBlockHash& index, TFunctor& functor){
		TraverseAllWithHashCode_Generic<VoxelBlockHash, HashEntry, TFunctor>(index, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithHashCode(const VoxelBlockHash& index, TFunctor& functor){
		TraverseAllWithHashCode_Generic<const VoxelBlockHash, const HashEntry, TFunctor>(index, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithHashCode(VoxelBlockHash& index, TFunctor& functor){
		TraverseUtilizedWithHashCode_Generic<VoxelBlockHash, HashEntry, TFunctor>(index, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilizedWithHashCode(const VoxelBlockHash& index, TFunctor& functor){
		TraverseUtilizedWithHashCode_Generic<const VoxelBlockHash, const HashEntry, TFunctor>(index, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseVisibleWithHashCode(VoxelBlockHash& index, TFunctor& functor){
		TraverseVisibleWithHashCode_Generic<VoxelBlockHash, HashEntry, TFunctor>(index, functor);
	}

	template<typename TFunctor>
	inline static void
	TraverseVisibleWithHashCode(const VoxelBlockHash& index, TFunctor& functor){
		TraverseVisibleWithHashCode_Generic<const VoxelBlockHash, const HashEntry, TFunctor>(index, functor);
	}
};

} // namespace ITMLib