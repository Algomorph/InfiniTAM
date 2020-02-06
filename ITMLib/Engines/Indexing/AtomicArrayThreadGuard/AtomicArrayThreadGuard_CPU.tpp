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
#include <atomic>
#include "AtomicArrayThreadGuard.h"
#include "../../../../ORUtils/PlatformIndependence.h"

namespace ITMLib {
template<>
class AtomicArrayThreadGuard<MEMORYDEVICE_CPU> {
public:
	explicit AtomicArrayThreadGuard(int item_count) : lock_array(new std::atomic<int>[item_count]) {
#ifdef WITH_OPENMP
#pragma omp parallel for default(none)
#endif
		for(int i_item =0; i_item < item_count; i_item++){
			lock_array[i_item].store((int)false);
		}
	}

	virtual ~AtomicArrayThreadGuard() {
		delete[] lock_array;
	};

	void lock(int item_number) {
		int expected = (int)false;
		while(lock_array[item_number].compare_exchange_weak(expected, (int)true)){}
	}

	void release(int item_number) {
		lock_array[item_number].store((false));
	}

private:
	std::atomic<int>* lock_array;
};

} // namespace ITMLib


