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
#include "../Interface/MemoryBlockTraversal.h"
#include "../../../../ORUtils/MemoryBlock.h"

namespace ITMLib {

template<>
class MemoryBlockTraversalEngine<MEMORYDEVICE_CPU> {
public:
	template<typename T, typename TFunctor>
	inline static void
	Traverse(ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor){
		T* data = memory_block.GetData(MEMORYDEVICE_CPU);
		assert(element_count < memory_block.size());

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(data, functor)
#endif
		for (int i_item = 0; i_item < element_count; i_item++){
			functor(data[i_item], i_item);
		}
	}

	template<typename T, typename TFunctor>
	inline static void
	Traverse(const ORUtils::MemoryBlock<T>& memory_block, const unsigned int element_count, TFunctor& functor){
		const T* data = memory_block.GetData(MEMORYDEVICE_CPU);
		assert(element_count < memory_block.size());

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(data, functor)
#endif
		for (int i_item = 0; i_item < element_count; i_item++){
			functor(data[i_item], i_item);
		}
	}
};

} // namespace ITMLib