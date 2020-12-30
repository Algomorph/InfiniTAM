// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CPU/MultiVisualizationEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/MultiVisualizationEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Metal/ITMMultiVisualizationEngine_Metal.h"
#endif

namespace ITMLib
{

	/**
	 * \brief This struct provides functions that can be used to construct Rendering engines.
	 */
	struct MultiVisualizationEngineFactory
	{
		//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

		/**
		 * \brief Makes a Rendering engine.
		 *
		 * \param deviceType  The device on which the Rendering engine should operate.
		 */
		template <typename TVoxel, typename TIndex>
		static MultiVisualizationEngine<TVoxel, TIndex> *MakeVisualizationEngine(MemoryDeviceType deviceType)
		{
			MultiVisualizationEngine<TVoxel, TIndex> *visualization_engine = nullptr;

			switch (deviceType)
			{
			case MEMORYDEVICE_CPU:
				visualization_engine = new MultiVisualizationEngine_CPU<TVoxel, TIndex>;
				break;
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				visualization_engine = new MultiVisualizationEngine_CUDA<TVoxel, TIndex>;
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				visualization_engine = new ITMMultiVisualizationEngine_CPU<TVoxelCanonical, TIndex>;
#endif
				break;
			}

			return visualization_engine;
		}
	};

}
