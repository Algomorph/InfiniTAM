//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/3/20.
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

#include "TelemetryRecorder.h"
#include "../../../ORUtils/MemoryDeviceType.h"
#include "../../../ORUtils/PlatformIndependence.h"

namespace ITMLib {

class TelemetryRecorderFactory {
public:
	template<typename TVoxel, typename TWarp, typename TIndex>
	static TelemetryRecorderInterface<TVoxel, TWarp, TIndex>* Build(MemoryDeviceType memory_device_type) {
		TelemetryRecorderInterface<TVoxel, TWarp, TIndex>* recorder = nullptr;
		switch (memory_device_type) {
			case MEMORYDEVICE_CPU:
				recorder = new TelemetryRecorder<TVoxel, TWarp, TIndex, MEMORYDEVICE_CPU>();
				break;
			case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOU_CUDA
				DIEWITHEXCEPTION_REPORTLOCATION("Requested construction of CUDA-based TelemetryRecorder while code built without CUDA support.");
				break;
#else
				recorder = new TelemetryRecorder<TVoxel, TWarp, TIndex, MEMORYDEVICE_CUDA>();
				break;
#endif
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				DIEWITHEXCEPTION_REPORTLOCATION("Not implemented.");
#else
				DIEWITHEXCEPTION_REPORTLOCATION(
						"Requested construction of Metal-based TelemetryRecorder while code built without Metal support.");
				break;
#endif
			default:
				DIEWITHEXCEPTION_REPORTLOCATION(
						"Unsupported memory_device_type argument issued for construction of TelemetryRecorder.");
				break;
		}
		return recorder;
	}

	template<typename TVoxel, typename TWarp, typename TIndex>
	static TelemetryRecorderInterface<TVoxel, TWarp, TIndex>& GetDefault(MemoryDeviceType memory_device_type) {
		switch (memory_device_type) {
			case MEMORYDEVICE_CPU:
				return TelemetryRecorder<TVoxel, TWarp, TIndex, MEMORYDEVICE_CPU>::GetDefaultInstance();
			case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOU_CUDA
				DIEWITHEXCEPTION_REPORTLOCATION("Requested construction of CUDA-based TelemetryRecorder while code built without CUDA support.");
				return TelemetryRecorder<TVoxel,TIndex,MEMORYDEVICE_CPU>::GetDefaultInstance();
#else
				return TelemetryRecorder<TVoxel, TWarp, TIndex, MEMORYDEVICE_CUDA>::GetDefaultInstance();
#endif
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				DIEWITHEXCEPTION_REPORTLOCATION("Not implemented.");
				return TelemetryRecorder<TVoxel,TIndex,MEMORYDEVICE_CPU>::GetDefaultInstance();
#else
				DIEWITHEXCEPTION_REPORTLOCATION(
						"Requested construction of Metal-based TelemetryRecorder while code built without Metal support.");
				return TelemetryRecorder<TVoxel, TWarp, TIndex, MEMORYDEVICE_CPU>::GetDefaultInstance();
#endif
			default:
				DIEWITHEXCEPTION_REPORTLOCATION(
						"Unsupported memory_device_type argument issued for construction of TelemetryRecorder.");
				return TelemetryRecorder<TVoxel, TWarp, TIndex, MEMORYDEVICE_CPU>::GetDefaultInstance();
				break;
		}
	}
};

} // namespace ITMLib
