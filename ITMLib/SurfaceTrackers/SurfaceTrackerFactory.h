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
class SurfaceTrackerFactory {
public:
/**
* \brief Makes a scene motion tracker.
*
* \param settings  settings to use
*/
	template<typename TVoxel, typename TWarp, typename TIndex>
	static SurfaceTrackerInterface<TVoxel, TWarp, TIndex>*
	MakeSceneMotionTracker() {
		SurfaceTrackerInterface<TVoxel, TWarp, TIndex>* motionTracker = nullptr;
		auto& settings = configuration::get();
		switch (settings.device_type) {
			case MEMORYDEVICE_CPU:
				switch (settings.non_rigid_tracking_parameters.functor_type){
					case TRACKER_SLAVCHEVA_OPTIMIZED:
						motionTracker = new SurfaceTracker<TVoxel, TWarp, TIndex, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_OPTIMIZED>();
						break;
					case TRACKER_SLAVCHEVA_DIAGNOSTIC:
						motionTracker = new SurfaceTracker<TVoxel, TWarp, TIndex, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>();
						break;
				}
				break;
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				switch (settings.non_rigid_tracking_parameters.functor_type) {
					case TRACKER_SLAVCHEVA_OPTIMIZED:
						motionTracker = new SurfaceTracker<TVoxel, TWarp, TIndex, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_OPTIMIZED>();
						break;
					case TRACKER_SLAVCHEVA_DIAGNOSTIC:
						motionTracker = new SurfaceTracker<TVoxel, TWarp, TIndex, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>();
						break;
				}
#else
				DIEWITHEXCEPTION_REPORTLOCATION("Not built with CUDA but CUDA type requested, aborting!");
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				//TODO
					DIEWITHEXCEPTION("Motion Scene Tracking not yet implemented on Metal")
#endif
				break;
		}

		return motionTracker;
	}
};
}//namespace ITMLib