//  ================================================================
//  Created by Gregory Kramida on 2/13/20.
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
#include "Interface/IndexingEngine.h"
#include "../../Utils/Configuration/Configuration.h"
#include "VBH/IndexingEngine_VoxelBlockHash.h"
#include "PVA/IndexingEngine_PlainVoxelArray.h"
#include "VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"
#endif
#include "../../Utils/Metacoding/DefferableStructUtilities.h"

namespace ITMLib {

/**
 * \brief This struct provides functions that can be used to construct scene reconstruction engines.
 */
struct IndexingEngineFactory {
	//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

	/**
	 * \brief Makes an indexing engine.
	 *
	 * \param device_type  The device on which the scene reconstruction engine should operate.
	 */
	template<typename TVoxel, typename TIndex>
	static IndexingEngineInterface<TVoxel, TIndex>*
	Build(MemoryDeviceType device_type = configuration::get().device_type,
			IndexingSettings settings = BuildDeferrableFromParentIfPresent<IndexingSettings>(configuration::get())) {
		IndexingEngineInterface<TVoxel, TIndex>* indexing_engine = nullptr;

		switch (device_type) {
			case MEMORYDEVICE_CPU:
				switch(settings.execution_mode){
					case OPTIMIZED:
						indexing_engine = new IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CPU, OPTIMIZED>();
						break;
					case DIAGNOSTIC:
						indexing_engine = new IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CPU, DIAGNOSTIC>();
			            break;
					default:
						DIEWITHEXCEPTION_REPORTLOCATION("Unsupported execution mode.");
						break;
				}
				break;
			case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
				DIEWITHEXCEPTION_REPORTLOCATION("Requested instantiation of a CUDA-based specialization, but code was compiled without CUDA. Aborting.");
#else
				switch(settings.execution_mode){
					case OPTIMIZED:
						indexing_engine = new IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CUDA, OPTIMIZED>();
						break;
					case DIAGNOSTIC:
						indexing_engine = new IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CUDA, DIAGNOSTIC>();
						break;
					default:
						DIEWITHEXCEPTION_REPORTLOCATION("Unsupported execution mode.");
						break;
				}
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				DIEWITHEXCEPTION_REPORTLOCATION("Metal support not implemented.");
#endif
				break;
		}

		return indexing_engine;
	}

	template<typename TVoxel, typename TIndex>
	static IndexingEngineInterface<TVoxel, TIndex>&
	GetDefault(MemoryDeviceType device_type = configuration::get().device_type,
	           IndexingSettings settings = BuildDeferrableFromParentIfPresent<IndexingSettings>(configuration::get())) {
		switch (device_type) {
			case MEMORYDEVICE_CPU:
				switch(settings.execution_mode){
					case OPTIMIZED:
						return IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CPU, OPTIMIZED>::Instance();
						break;
					case DIAGNOSTIC:
						return IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CPU, DIAGNOSTIC>::Instance();
						break;
					default:
						DIEWITHEXCEPTION_REPORTLOCATION("Unsupported execution mode.");
						return IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CPU>::Instance();
				}

			case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
				DIEWITHEXCEPTION_REPORTLOCATION("Requested instantiation of a CUDA-based specialization, but code was compiled without CUDA. Aborting.");
				return IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CPU>::Instance();
#else
				switch(settings.execution_mode){
					case OPTIMIZED:
						return IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CUDA, OPTIMIZED>::Instance();
						break;
					case DIAGNOSTIC:
						return IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CUDA, DIAGNOSTIC>::Instance();
						break;
					default:
						DIEWITHEXCEPTION_REPORTLOCATION("Unsupported execution mode.");
						return IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CUDA>::Instance();
				}
#endif
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				DIEWITHEXCEPTION_REPORTLOCATION("Metal indexing support not yet implemented.");
				return IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_METAL>::Instance();
#else
				DIEWITHEXCEPTION_REPORTLOCATION("Requested instantiation of a Metal-based specialization, but code was compiled without Metal. Aborting.");
				return IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CPU>::Instance();
#endif
			default:
				return IndexingEngine<TVoxel, TIndex, MEMORYDEVICE_CPU>::Instance();
		}
	}
};

} // namespace ITMLib