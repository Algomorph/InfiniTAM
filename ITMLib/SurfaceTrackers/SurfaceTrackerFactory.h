//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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

#include "Interface/SurfaceTrackerInterface.h"
#include "Interface/SurfaceTracker.h"

#ifdef COMPILE_WITH_METAL
#error "NOT CURRENTLY SUPPORTED"
#endif

namespace ITMLib {

namespace internal{
template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
static SurfaceTrackerInterface<TVoxel, TWarp, TIndex>* BuildSurfaceTrackerAux(const ExecutionMode& execution_mode){
	switch (execution_mode){
		case OPTIMIZED:
			return new SurfaceTracker<TVoxel, TWarp, TIndex, MEMORYDEVICE_CPU, OPTIMIZED>();
			break;
		case DIAGNOSTIC:
			return new SurfaceTracker<TVoxel, TWarp, TIndex, MEMORYDEVICE_CPU, DIAGNOSTIC>();
			break;
	}
	return nullptr;
}
}// namespace internal



class SurfaceTrackerFactory {
public:
/**
* \brief Makes a scene motion tracker.
*
* \param settings  settings to use
*/
	template<typename TVoxel, typename TWarp, typename TIndex>
	static SurfaceTrackerInterface<TVoxel, TWarp, TIndex>* Build() {
		SurfaceTrackerInterface<TVoxel, TWarp, TIndex>* surface_tracker = nullptr;
		auto& settings = configuration::Get();
		auto level_set_evolution_parameters = ExtractDeferrableSerializableStructFromPtreeIfPresent<LevelSetEvolutionParameters>(
				configuration::Get().source_tree,
				configuration::Get().origin
		);

		switch (settings.device_type) {
			case MEMORYDEVICE_CPU:
				surface_tracker = internal::BuildSurfaceTrackerAux<TVoxel, TWarp, TIndex, MEMORYDEVICE_CPU>(level_set_evolution_parameters.execution_mode);
				break;
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				surface_tracker = internal::BuildSurfaceTrackerAux<TVoxel, TWarp, TIndex, MEMORYDEVICE_METAL>(level_set_evolution_parameters.execution_mode);
#else
				DIEWITHEXCEPTION_REPORTLOCATION("Libary not built with CUDA, but construction of CUDA-based surface tracker requested, aborting!");
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				//TODO
					DIEWITHEXCEPTION("Motion Scene Tracking not yet implemented on Metal")
#endif
				break;
			case MEMORYDEVICE_NONE:
				DIEWITHEXCEPTION_REPORTLOCATION("For construction of a SurfaceTracker instance, device_type cannot be MEMORYDEVICE_NONE.");
				break;
		}

		return surface_tracker;
	}
};
}//namespace ITMLib