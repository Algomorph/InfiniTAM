//  ================================================================
//  Created by Gregory Kramida on 2/21/20.
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

#include "VolumeStatisticsCalculator.h"
#include "../../Configuration.h"

namespace ITMLib {

struct VolumeStatisticsCalculatorFactory {
public:
	template<typename TVoxel, typename TIndex>
	static VolumeStatisticsCalculatorInterface<TVoxel, TIndex>&
	Get(MemoryDeviceType device_type = configuration::get().device_type) {
		switch (device_type) {
			case MEMORYDEVICE_CPU:
				return VolumeStatisticsCalculator<TVoxel,TIndex,MEMORYDEVICE_CPU>::Instance();
				break;
			case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
				DIEWITHEXCEPTION_REPORTLOCATION("Trying to access an instance of a class template specialized for CUDA, while code does not appear to be built with CUDA support.");
#else
				return VolumeStatisticsCalculator<TVoxel,TIndex,MEMORYDEVICE_CUDA>::Instance();
				break;
#endif
				break;
			case MEMORYDEVICE_METAL:
				DIEWITHEXCEPTION_REPORTLOCATION("Metal support not implemented.");
				break;
			default:
				DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
				break;
		}
	}
};

typedef VolumeStatisticsCalculatorFactory StatCalc_Accessor;

}

