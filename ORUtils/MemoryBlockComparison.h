//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/21/20.
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

#include "MemoryDeviceType.h"
#include "PlatformIndependence.h"
#include "MemoryBlock.h"

#ifndef __METALC__

#ifndef COMPILE_WITHOUT_CUDA
#include "CUDADefines.h"
#endif

namespace ORUtils {

template<typename T>
bool operator==(const MemoryBlock<T>& l, const MemoryBlock<T>& r) {
	if (&r == &l) {
		return true;
	}
	if (r.element_count != l.element_count) return false;
	const typename MemoryBlock<T>::size_type element_count = r.element_count;

	MemoryCopyDirection direction = DetermineMemoryCopyDirection(l.GetAccessMode(), r.GetAccessMode());

	switch (direction) {
		case CPU_TO_CPU: {
			T* data_r = r.GetData(MEMORYDEVICE_CPU);
			T* data_l = l.GetData(MEMORYDEVICE_CPU);
			bool mismatch_found = false;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(data_l, data_r, mismatch_found)
#endif
			for (int i_element = 0; i_element < element_count; i_element++) {
				if(mismatch_found) continue;
				if (data_l[i_element] != data_r[i_element]) {
					mismatch_found = true;
				}
			}
		}
			return true;
		case CPU_TO_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		{
			MemoryBlock<T> r_CPU(element_count, MEMORYDEVICE_CPU);
			r_CPU.SetFrom(r);
			return l == r_CPU;
		}
#else
			assert(false);
			return false;
#endif
		case CUDA_TO_CPU:
#ifndef COMPILE_WITHOUT_CUDA
		{
			MemoryBlock<T> l_CPU(element_count, MEMORYDEVICE_CPU);
			l_CPU.SetFrom(l);
			return l_CPU == r;
		}
#else
			assert(false);
			return false;
#endif
		case CUDA_TO_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		{
			MemoryBlock<T> l_CPU(element_count, MEMORYDEVICE_CPU);
			l_CPU.SetFrom(l);
			MemoryBlock<T> r_CPU(element_count, MEMORYDEVICE_CPU);
			r_CPU.SetFrom(r);
			return l_CPU == r_CPU;
		}
#else
			assert(false);
			return false;
#endif
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unknown MemoryCopyDirection");
			return false;
	}
}

} // namespace ORUtils
#endif //__METALC__
