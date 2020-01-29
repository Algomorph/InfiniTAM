//  ================================================================
//  Created by Gregory Kramida on 12/5/19.
//  Copyright (c)  2019 Gregory Kramida
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
#include "Interface/EditAndCopyEngineInterface.h"
#include "CPU/EditAndCopyEngine_CPU.h"
#include "CUDA/EditAndCopyEngine_CUDA.h"

#pragma once
namespace ITMLib {

struct EditAndCopyEngineFactory {
	template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
	static EditAndCopyEngineInterface<TVoxel, TIndex>& Instance() {
		switch (TMemoryDeviceType) {
			default:
			case MEMORYDEVICE_CPU:
				return EditAndCopyEngine_CPU<TVoxel, TIndex>::Inst();
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				return EditAndCopyEngine_CUDA<TVoxel, TIndex>::Inst();
#else
			std::cerr << "Warning: compiled without CUDA but requesting an instance of CUDA manipulation engine. "
				"Defaulting to CPU manipulation engine." << std::endl;
			return EditAndCopyEngine_CPU<TVoxel,TIndex>::Inst();
#endif
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
#error METAL SUPPORT NOT FULLY IMPLEMENTED
#else
				std::cerr << "Warning: compiled without METAL but requesting an instance of METAL manipulation engine. "
				             "Defaulting to CPU manipulation engine." << std::endl;
				return EditAndCopyEngine_CPU<TVoxel, TIndex>::Inst();
#endif

		}
	}
};
}