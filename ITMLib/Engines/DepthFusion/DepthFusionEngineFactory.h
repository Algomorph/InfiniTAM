//  ================================================================
//  Created by Gregory Kramida on 5/22/18.
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

#include "../../Utils/Configuration.h"

#include "DepthFusionEngine.h"

namespace ITMLib {

/**
 * \brief This struct provides functions that can be used to construct scene reconstruction engines.
 */
struct DepthFusionEngineFactory {
	//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

	/**
	 * \brief Makes a scene reconstruction engine.
	 *
	 * \param deviceType  The device on which the scene reconstruction engine should operate.
	 */
	template<typename TVoxel, typename TWarp, typename TIndex>
	static DepthFusionEngineInterface<TVoxel, TWarp, TIndex>*
	Build(MemoryDeviceType deviceType) {
		DepthFusionEngineInterface<TVoxel, TWarp, TIndex>* depth_fusion_engine = nullptr;

		switch (deviceType) {
			case MEMORYDEVICE_CPU:
				depth_fusion_engine = new DepthFusionEngine<TVoxel, TWarp, TIndex, MEMORYDEVICE_CPU>;
				break;
			case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
				DIEWITHEXCEPTION_REPORTLOCATION("Requested instantiation of a CUDA-based specialization, but code was compiled without CUDA. Aborting.");
#else
				depth_fusion_engine = new DepthFusionEngine<TVoxel, TWarp, TIndex, MEMORYDEVICE_CUDA>;
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				depth_fusion_engine = new DepthFusionEngine_Metal<TVoxelA,TIndex>;
#endif
				break;
		}

		return depth_fusion_engine;
	}
};

}
