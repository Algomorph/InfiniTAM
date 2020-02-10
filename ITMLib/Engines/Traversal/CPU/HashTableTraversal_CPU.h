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

namespace ITMLib {

template<>
class HashTableTraversalEngine<MEMORYDEVICE_CPU> {
public:
	template<typename TFunctor>
	inline static void
	TraverseWithHashCode(VoxelBlockHash& index, TFunctor& functor){
		HashEntry* hash_table = index.GetEntries();
		const int hash_entry_count = index.hashEntryCount;
#ifdef WITH_OPENMP
	#pragma omp parallel for default(none) shared(functor, hash_table)
#endif
		for (int hash_code = 0; hash_code < hash_entry_count; hash_code++){
			functor(hash_table[hash_code], hash_code);
		}
	}
};

} // namespace ITMLib


// TODO