//  ================================================================
//  Created by Gregory Kramida on 1/30/20.
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
#include "VolumeFusionEngine.h"
#include "../../Utils/Configuration.h"

#ifdef COMPILE_WITH_METAL
#error "NOT CURRENTLY SUPPORTED"
#endif

namespace ITMLib{
class VolumeFusionEngineFactory{
public:
	template<typename TVoxel, typename TWarp, typename TIndex>
	static VolumeFusionEngineInterface<TVoxel, TWarp, TIndex>*
	MakeVolumeFusionEngine(MemoryDeviceType memoryDeviceType = configuration::get().device_type){
		VolumeFusionEngineInterface<TVoxel, TWarp, TIndex>* volume_fusion_engine = nullptr;
		switch (memoryDeviceType) {
			case MEMORYDEVICE_CPU:
				volume_fusion_engine = new VolumeFusionEngine<TVoxel, TWarp, TIndex, MEMORYDEVICE_CPU>();
				break;
			case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
				DIEWITHEXCEPTION_REPORTLOCATION("Not built with CUDA but CUDA type requested, aborting!");
#else
				volume_fusion_engine = new VolumeFusionEngine<TVoxel, TWarp, TIndex, MEMORYDEVICE_CUDA>();
#endif

				break;
		}
		return volume_fusion_engine;
	}
};
} // namespace ITMLib
