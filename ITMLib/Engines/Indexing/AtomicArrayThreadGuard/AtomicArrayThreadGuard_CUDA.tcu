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

namespace ITMLib{
template<>
class AtomicArrayThreadGuard<MEMORYDEVICE_CUDA>{
public:
	explicit AtomicArrayThreadGuard(int item_count) : lock_array_wrapper(item_count * sizeof(char), false, true)
	{
		lock_array_wrapper.Clear();
		lock_array = lock_array_wrapper.GetData(MEMORYDEVICE_CUDA);
	}

	_DEVICE_WHEN_AVAILABLE_
	void lock(int item_number){
		while(atomicCAS(lock_array + item_number, (char)0, (char)1) != 0){};
	}

	_DEVICE_WHEN_AVAILABLE_
	void release(int item_number){
		lock_array[item_number] = 0;
	}
private:
	ORUtils::MemoryBlock<char> lock_array_wrapper;
	char* lock_array;
};
} // namespace ITMLib


