//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 6/08/20.
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

#include "RenderingEngine.h"

namespace ITMLib {

/**
 * \brief This struct provides functions that can be used to construct Rendering engines.
 */
struct RenderingEngineFactory {
	/**
	 * \brief Makes a Rendering engine.
	 *
	 * \param device_type  The device on which the Rendering engine should operate.
	 */
	template<typename TVoxel, typename TIndex>
	static RenderingEngineBase<TVoxel, TIndex>* Build(MemoryDeviceType device_type = configuration::get().device_type) {
		RenderingEngineBase<TVoxel, TIndex>* rendering_engine = nullptr;

		switch (device_type) {
			case MEMORYDEVICE_CPU:
				rendering_engine = new RenderingEngine<TVoxel, TIndex, MEMORYDEVICE_CPU>;
				break;
			case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
				DIEWITHEXCEPTION_REPORTLOCATION("Attempting to construct a CUDA-based RenderingEngine while built without CUDA support, aborting.");
#else
				rendering_engine = new RenderingEngine<TVoxel, TIndex, MEMORYDEVICE_CUDA>;
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				visualization_engine = new VisualizationEngine_Metal<TVoxelCanonical,TIndex>;
#endif
				break;
			default:
				break;
		}

		return rendering_engine;
	}
};

} // namespace ITMLib
