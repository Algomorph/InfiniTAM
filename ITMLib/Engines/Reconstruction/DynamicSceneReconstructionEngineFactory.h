//  ================================================================
//  Created by Gregory Kramida on 5/22/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "CPU/DynamicSceneReconstructionEngine_CPU.h"

#ifndef COMPILE_WITHOUT_CUDA

#include "CUDA/DynamicSceneReconstructionEngine_CUDA.h"

#endif
#ifdef COMPILE_WITH_METAL
#error "NOT CURRENTLY SUPPORTED"
#endif

namespace ITMLib {

/**
 * \brief This struct provides functions that can be used to construct scene reconstruction engines.
 */
struct DynamicSceneReconstructionEngineFactory {
	//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

	/**
	 * \brief Makes a scene reconstruction engine.
	 *
	 * \param deviceType  The device on which the scene reconstruction engine should operate.
	 */
	template<typename TVoxel, typename TWarp, typename TIndex>
	static DynamicSceneReconstructionEngine<TVoxel, TWarp, TIndex>*
	MakeSceneReconstructionEngine(MemoryDeviceType deviceType) {
		DynamicSceneReconstructionEngine<TVoxel, TWarp, TIndex>* sceneRecoEngine = NULL;

		switch (deviceType) {
			case MEMORYDEVICE_CPU:
				sceneRecoEngine = new DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, TIndex>;
				break;
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				sceneRecoEngine = new DynamicSceneReconstructionEngine_CUDA<TVoxel, TWarp, TIndex>;
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<TVoxelA,TIndex>;
#endif
				break;
		}

		return sceneRecoEngine;
	}
};

}
