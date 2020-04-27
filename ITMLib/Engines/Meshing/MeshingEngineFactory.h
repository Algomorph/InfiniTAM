// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CPU/MeshingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/MeshingEngine_CUDA.h"
#endif

namespace ITMLib
{

	/**
	 * \brief This struct provides functions that can be used to construct meshing engines.
	 */
	struct MeshingEngineFactory
	{
		//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

		/**
		 * \brief Makes a meshing engine.
		 *
		 * \param device_type  The device on which the meshing engine should operate.
		 */
		template <typename TVoxel, typename TIndex>
		static MeshingEngine<TVoxel, TIndex>* Build(MemoryDeviceType device_type){
			MeshingEngine<TVoxel, TIndex>* meshing_engine = nullptr;

			switch (device_type){
			case MEMORYDEVICE_CPU:
				meshing_engine = new MeshingEngine_CPU<TVoxel, TIndex>();
				break;
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				meshing_engine = new MeshingEngine_CUDA<TVoxel, TIndex>();
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				meshing_engine = new MeshingEngine_CPU<TVoxelCanonical, TIndex>();
#endif
				break;
			default:
				DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type");
			}

			return meshing_engine;
		}
	};
}