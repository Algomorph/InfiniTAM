// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CPU/MultiMeshingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/MultiMeshingEngine_CUDA.h"
#endif

namespace ITMLib
{

	/**
	 * \brief This struct provides functions that can be used to construct meshing engines.
	 */
	struct MultiMeshingEngineFactory
	{
		//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

		/**
		 * \brief Makes a meshing engine.
		 *
		 * \param deviceType  The device on which the meshing engine should operate.
		 */
		template <typename TVoxel, typename TIndex>
		static MultiMeshingEngine<TVoxel, TIndex> *MakeMeshingEngine(MemoryDeviceType deviceType, const TIndex& index)
		{
			MultiMeshingEngine<TVoxel, TIndex> *meshingEngine = nullptr;

			switch (deviceType)
			{
			case MEMORYDEVICE_CPU:
				meshingEngine = new MultiMeshingEngine_CPU<TVoxel, TIndex>(index);
				break;
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				meshingEngine = new MultiMeshingEngine_CUDA<TVoxel, TIndex>(index);
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				meshing_engine = new MultiMeshingEngine_CPU<TVoxelCanonical, TIndex>(index);
#endif
				break;
			}

			return meshingEngine;
		}
	};
}
